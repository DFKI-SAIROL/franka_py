#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include "custom_interfaces/msg/rig_snapshot.hpp"

#include <map>
#include <string>
#include <vector>
#include <mutex>
#include <cmath>
#include <unordered_map>
#include <algorithm>
#include <initializer_list>


class ZedRigAggregator : public rclcpp::Node {
public:
    ZedRigAggregator() : Node("zed_rig_aggregator") {
        // --- Parameters ---
        // 30Hz Grid (33.33ms) matches hardware target
        this->declare_parameter("time_grid", 0.03333); 
        this->declare_parameter("camera_names", std::vector<std::string>());
        this->declare_parameter("publish_topic", std::string("rig_snapshot"));
        
        this->declare_parameter("rgb_topics", std::vector<std::string>());
        this->declare_parameter("depth_topics", std::vector<std::string>());
        this->declare_parameter("camera_info_topics", std::vector<std::string>());

        time_grid_ = this->get_parameter("time_grid").as_double();
        cam_names_ = this->get_parameter("camera_names").as_string_array();
        
        // QoS: Keep Last 1 + Volatile for speed
        auto qos_video = rclcpp::SensorDataQoS();
        qos_video.keep_last(1); 
        auto qos_info = rclcpp::QoS(1).reliable().durability_volatile();

        cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        auto sub_opts = rclcpp::SubscriptionOptions();
        sub_opts.callback_group = cb_group_;

        auto rgb_tops = this->get_parameter("rgb_topics").as_string_array();
        auto depth_tops = this->get_parameter("depth_topics").as_string_array();
        auto info_tops = this->get_parameter("camera_info_topics").as_string_array();

        if (cam_names_.empty()) {
            RCLCPP_ERROR(this->get_logger(), "CRITICAL: No camera names found!");
            return;
        }

        // --- Subscriptions ---
        for (size_t i = 0; i < cam_names_.size(); ++i) {
            std::string name = cam_names_[i];
            
            // 1. Camera Info (One-shot optimization)
            info_subs_.push_back(this->create_subscription<sensor_msgs::msg::CameraInfo>(
                info_tops[i], qos_info,
                [this, name](sensor_msgs::msg::CameraInfo::SharedPtr msg) {
                    if (has_info_[name]) return; 
                    std::lock_guard<std::mutex> lock(info_mutex_);
                    if (camera_infos_.find(name) == camera_infos_.end()) {
                        camera_infos_[name] = msg;
                        has_info_[name] = true;
                        RCLCPP_INFO(this->get_logger(), "Received Intrinsics for: %s", name.c_str());
                    }
                }, sub_opts));

            // 2. Image Streams
            auto sub_img = [&](std::string t, int type) {
                return this->create_subscription<sensor_msgs::msg::Image>(
                    t, qos_video,
                    [this, name, type](sensor_msgs::msg::Image::ConstSharedPtr msg) {
                        this->ingest(msg, name, type);
                    }, sub_opts);
            };
            img_subs_.push_back(sub_img(rgb_tops[i], 0)); // Type 0 = RGB
            img_subs_.push_back(sub_img(depth_tops[i], 1)); // Type 1 = Depth
            
            has_info_[name] = false;
        }
        auto pub_qos = rclcpp::SensorDataQoS(); // Default: Best Effort, Volatile
        pub_qos.keep_last(1);
        pub_ = this->create_publisher<custom_interfaces::msg::RigSnapshot>(
            this->get_parameter("publish_topic").as_string(),
            pub_qos
        );
            
        RCLCPP_INFO(this->get_logger(), "Aggregator: Greedy Forward Scan. Max Buffer Size: 8");
    }

private:
    struct CameraSet {
        sensor_msgs::msg::Image::ConstSharedPtr rgb, depth;
        bool ready() const { return rgb && depth; }
    };

    struct Bucket { std::unordered_map<std::string, CameraSet> cams; };

    // Helper: Check if a specific bucket has ALL cameras ready.
    bool is_bucket_complete(std::map<long, Bucket>::iterator it) {
        if (it == buckets_.end()) return false;
        if (it->second.cams.size() < cam_names_.size()) return false; 
        for (const auto& name : cam_names_) {
            if (it->second.cams.find(name) == it->second.cams.end() || 
                !it->second.cams[name].ready()) {
                return false;
            }
        }
        return true;
    }

    void ingest(sensor_msgs::msg::Image::ConstSharedPtr msg, std::string name, int type) {
        // Fast-fail if missing intrinsics
        {
            std::lock_guard<std::mutex> lk(info_mutex_);
            if (camera_infos_.size() < cam_names_.size()) return;
        }

        double t = rclcpp::Time(msg->header.stamp).seconds();
        long bid = std::lround(t / time_grid_);

        std::map<std::string, std::pair<CameraSet, long>> bundle;
        bool found = false;
        rclcpp::Time master_stamp;

        {   // --- LOCK ---
            std::lock_guard<std::mutex> lock(mtx_);
            stats_ingested_++;

            // 1. Insert Data
            auto& s = buckets_[bid].cams[name];
            if (type == 0) s.rgb = msg; 
            else if (type == 1) s.depth = msg; 

            // 2. GREEDY FORWARD SCAN
            auto it = buckets_.begin();
            while (it != buckets_.end()) {
                
                // Strategy A: Pure Match
                if (is_bucket_complete(it)) {
                    for(auto& cn : cam_names_) bundle[cn] = {it->second.cams[cn], it->first};
                    found = true;
                    break; 
                }

                // Strategy B: Fuzzy Repair
                auto it_next = std::next(it);
                if (it_next != buckets_.end()) {
                     bool possible = true;
                     std::map<std::string, std::pair<CameraSet, long>> temp_bundle;
                     
                     for (const auto& cn : cam_names_) {
                        if (it->second.cams.count(cn) && it->second.cams[cn].ready()) {
                            temp_bundle[cn] = {it->second.cams[cn], it->first};
                        }
                        else if (it_next->second.cams.count(cn) && it_next->second.cams[cn].ready()) {
                            temp_bundle[cn] = {it_next->second.cams[cn], it_next->first};
                        } else {
                            possible = false; break;
                        }
                     }

                     if (possible) {
                         bundle = temp_bundle;
                         found = true;
                         break;
                     }
                }
                it++;
            }

            // 3. CLEANUP & MONOTONICITY
            if (found) {
                master_stamp = bundle.begin()->second.first.rgb->header.stamp;
                for (auto const& [name, data] : bundle) {
                    if (rclcpp::Time(data.first.rgb->header.stamp) < rclcpp::Time(master_stamp)) {
                        master_stamp = data.first.rgb->header.stamp;
                    }
                }

                if (!first_publication_ && rclcpp::Time(master_stamp) <= last_pub_ts_) {
                    found = false; 
                    stats_dropped_++; 
                } else {
                    last_pub_ts_ = master_stamp;
                    first_publication_ = false;
                    stats_matches_++;
                }

                if (found) {
                    for (auto& item : bundle) {
                        long b_id = item.second.second;
                        buckets_[b_id].cams.erase(item.first);
                    }
                    
                    long master_bid = std::lround(rclcpp::Time(master_stamp).seconds() / time_grid_);
                    auto cleanup_it = buckets_.begin();
                    while (cleanup_it != buckets_.end()) {
                        if (cleanup_it->first <= master_bid) {
                            cleanup_it = buckets_.erase(cleanup_it);
                        } else {
                            ++cleanup_it;
                        }
                    }
                }
            }

            // 4. Garbage Collection
            if (buckets_.size() > 8) {
                buckets_.erase(buckets_.begin());
                stats_dropped_++;
            }
            
            print_stats_if_needed();

        } // --- UNLOCK ---

        // PUBLISH (Deep Copy Phase)
        if (found) {
            custom_interfaces::msg::RigSnapshot out;
            out.header.stamp = master_stamp;
            out.camera_names = cam_names_;

            for (auto& n : cam_names_) {
                const auto& [cam_data, _] = bundle[n];

                auto img_r = *cam_data.rgb; img_r.header.stamp = master_stamp;
                auto img_d = *cam_data.depth; img_d.header.stamp = master_stamp;

                out.rgbs.push_back(std::move(img_r));
                out.depths.push_back(std::move(img_d));
            
                {
                    std::lock_guard<std::mutex> lk(info_mutex_);
                    auto info = *camera_infos_[n];
                    info.header.stamp = master_stamp;
                    out.cam_infos.push_back(std::move(info));
                }
            }
            pub_->publish(out);
        }
    }

    void print_stats_if_needed() {
        if (stats_ingested_ % 300 == 0) {
            double success_rate = 0.0;
            // Note: Divided by 6.0 (3 cams * 2 topics)
            if (stats_ingested_ > 0) success_rate = (double)stats_matches_ / (stats_ingested_ / 6.0) * 100.0; 
            RCLCPP_INFO(this->get_logger(), 
                "STATS | Ing: %ld | Match: %ld | Rate: %.1f%% | Drop: %ld | Buf: %zu",
                stats_ingested_, stats_matches_, success_rate, stats_dropped_, buckets_.size());
        }
    }

    // Members
    double time_grid_;
    std::vector<std::string> cam_names_;
    
    std::mutex mtx_, info_mutex_;
    std::map<long, Bucket> buckets_;
    
    rclcpp::Time last_pub_ts_;
    bool first_publication_ = true;
    
    std::map<std::string, sensor_msgs::msg::CameraInfo::SharedPtr> camera_infos_;
    std::unordered_map<std::string, bool> has_info_;

    std::vector<rclcpp::SubscriptionBase::SharedPtr> img_subs_, info_subs_;
    rclcpp::Publisher<custom_interfaces::msg::RigSnapshot>::SharedPtr pub_;
    rclcpp::CallbackGroup::SharedPtr cb_group_;

    long stats_ingested_ = 0, stats_matches_ = 0, stats_dropped_ = 0;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), 8);
    auto node = std::make_shared<ZedRigAggregator>();
    exec.add_node(node);
    exec.spin();
    rclcpp::shutdown();
    return 0;
}