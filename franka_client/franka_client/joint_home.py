"""Joint-space homing of one arm through the safety node's joint-velocity input.

The safety node (robot_safety_layer) takes joint velocities on ``/<arm>/policy_target_dq`` and
integrates them into its joint command; a Cartesian ``/<arm>/target_pose`` call preempts them,
and 0.25 s after the last one control falls back to the IK node. ``JointHome`` drives all seven
joints from where they are to ``home_joints`` along one synchronized minimum-jerk trajectory (the
fastest joint peaking at ``max_speed``), settles on the measured joints, and finally re-targets
the IK node at the pose it reached, so the IK does not pull the arm back to its old target.

The home joints live in the teleop config of the arm (``franka_meta_quest/config/
teleop_<side>.yaml``: ``home_joints``, ``home_joint_max_speed``, ``home_joint_min_duration``),
the one place used by the teleop's B button, the studio's Home and the clients' H key.

    python -m franka_client.joint_home            # prints the current joints of both arms
"""

from __future__ import annotations

import threading
from pathlib import Path

import numpy as np
import yaml
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
from rclpy.time import Time
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool

from franka_custom_msgs.srv import SetPoseStamped

RATE_HZ = 50.0
SETTLE_GAIN = 2.0          # 1/s, joint error -> velocity once the trajectory has ended
SETTLE_MAX_SPEED = 0.1     # rad/s
SETTLE_TOL = 0.005         # rad, done below this on every joint
SETTLE_TIMEOUT_S = 1.5
MAX_DEVIATION = 0.3        # rad between measured and planned joints: something else moves the arm
JOINT_STATES_STALE_S = 0.25
HALT_TOPIC = "/robot_halt"   # latched std_msgs/Bool of the studio's Stop (the safety node holds the arms)
HALT_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)


def check_joints(values) -> np.ndarray:
    q = np.array([float(v) for v in values], dtype=float)
    if q.shape != (7,) or not np.all(np.isfinite(q)):
        raise ValueError(f"home joints need 7 finite values, got {list(values)!r}")
    return q


def teleop_config_path(ns: str) -> Path | None:
    """franka_meta_quest's teleop_<side>.yaml for the arm namespace (franka_right -> right): the
    installed share, else the source tree next to this package (a process without the sourced
    workspace, e.g. the studio started from another shell)."""
    side = ns.strip("/").rsplit("_", 1)[-1]
    dirs = []
    try:
        from ament_index_python.packages import get_package_share_directory
        dirs.append(Path(get_package_share_directory("franka_meta_quest")) / "config")
    except Exception:
        pass
    # src/franka_py/franka_client/franka_client/joint_home.py -> src/franka_py/franka_meta_quest/config
    dirs.append(Path(__file__).resolve().parents[2] / "franka_meta_quest" / "config")
    for d in dirs:
        if (d / f"teleop_{side}.yaml").is_file():
            return d / f"teleop_{side}.yaml"
    return None


def load_home(ns: str, config: dict | None = None) -> dict | None:
    """{joints, max_speed, min_duration} from ``config`` (a teleop config dict) or the arm's
    teleop config; None without ``home_joints``. Raises ValueError for invalid values."""
    if config is None:
        path = teleop_config_path(ns)
        if path is None:
            return None
        config = yaml.safe_load(path.read_text()) or {}
    if config.get("home_joints") is None:
        return None
    home = {"joints": check_joints(config["home_joints"]),
            "max_speed": float(config.get("home_joint_max_speed", 0.3)),
            "min_duration": float(config.get("home_joint_min_duration", 2.0))}
    if home["max_speed"] <= 0.0:
        raise ValueError("home_joint_max_speed must be > 0")
    return home


class JointHome:
    def __init__(self, node, ns: str, home: dict, callback_group=None, tf_buffer=None) -> None:
        self.node = node
        self.arm = ns.strip("/")
        self.ns = "/" + self.arm
        self.home = home
        self.joint_names = [f"{self.arm}_fr3_joint{i}" for i in range(1, 8)]
        self._lock = threading.Lock()
        self._q: np.ndarray | None = None
        self._q_time = 0.0
        self._traj: dict | None = None
        self.message = ""
        self._dq_pub = node.create_publisher(JointState, self.ns + "/policy_target_dq", 1)
        self._pose_client = node.create_client(SetPoseStamped, self.ns + "/target_pose",
                                               callback_group=callback_group)
        node.create_subscription(JointState, self.ns + "/franka/joint_states", self._on_joints, 1,
                                 callback_group=callback_group)
        self.halted = False
        node.create_subscription(Bool, HALT_TOPIC, self._on_halt, HALT_QOS, callback_group=callback_group)
        if tf_buffer is None:
            from tf2_ros import Buffer, TransformListener
            tf_buffer = Buffer()
            self._tf_listener = TransformListener(tf_buffer, node)
        self._tf = tf_buffer
        node.create_timer(1.0 / RATE_HZ, self._tick, callback_group=callback_group)

    # ---- state -----------------------------------------------------------------------------------

    def _now(self) -> float:
        return self.node.get_clock().now().nanoseconds * 1e-9

    def _on_joints(self, msg: JointState) -> None:
        try:
            q = np.array([msg.position[msg.name.index(n)] for n in self.joint_names], dtype=float)
        except ValueError:
            return
        with self._lock:
            self._q, self._q_time = q, self._now()

    def _on_halt(self, msg: Bool) -> None:
        self.halted = bool(msg.data)
        if self.halted:
            with self._lock:
                self._traj = None   # the safety node ignores us now; no IK re-target either
            self.message = f"{self.arm}: homing stopped (halted)."

    def measured(self) -> np.ndarray | None:
        with self._lock:
            return None if self._q is None else self._q.copy()

    @property
    def active(self) -> bool:
        with self._lock:
            return self._traj is not None

    # ---- control ---------------------------------------------------------------------------------

    def start(self, target=None) -> bool:
        """Plans from the measured joints to ``target`` (default: the home joints)."""
        q1 = self.home["joints"] if target is None else check_joints(target)
        if self.halted:
            self.message = f"{self.arm}: halted (Stop) -- resume first."
            self.node.get_logger().warning(self.message)
            return False
        with self._lock:
            q0 = None if self._q is None else self._q.copy()
            fresh = self._now() - self._q_time < JOINT_STATES_STALE_S
        if q0 is None or not fresh:
            self.message = f"{self.arm}: no joint states, not homing."
            self.node.get_logger().warning(self.message)
            return False
        dist = float(np.abs(q1 - q0).max())
        # Minimum jerk peaks at 1.875x the average speed.
        T = max(1.875 * dist / self.home["max_speed"], self.home["min_duration"])
        with self._lock:
            self._traj = {"q0": q0, "q1": q1, "t0": self._now(), "T": T}
        self.message = f"{self.arm}: homing, max joint distance {dist:.2f} rad, {T:.1f} s."
        self.node.get_logger().info(self.message)
        return True

    def stop(self, reason: str = "stopped") -> None:
        with self._lock:
            was = self._traj is not None
            self._traj = None
        if was:
            self._publish_dq(np.zeros(7))
            self.message = f"{self.arm}: homing {reason}."
            self.node.get_logger().info(self.message)
            self._retarget_ik()

    def _publish_dq(self, dq: np.ndarray) -> None:
        msg = JointState()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.name = list(self.joint_names)
        msg.position = [float(v) for v in dq]   # the safety node reads velocities from position
        self._dq_pub.publish(msg)

    def _tick(self) -> None:
        with self._lock:
            traj = None if self._traj is None else dict(self._traj)
            q = None if self._q is None else self._q.copy()
            stale = self._now() - self._q_time > JOINT_STATES_STALE_S
        if traj is None:
            return
        if q is None or stale:
            self.stop("aborted: joint states stopped")
            return
        t = self._now() - traj["t0"]
        q0, q1, T = traj["q0"], traj["q1"], traj["T"]
        if t < T:
            tau = t / T
            s = 10 * tau**3 - 15 * tau**4 + 6 * tau**5
            ds = (30 * tau**2 - 60 * tau**3 + 30 * tau**4) / T
            q_ref = q0 + (q1 - q0) * s
            if float(np.abs(q - q_ref).max()) > MAX_DEVIATION:
                self.stop("aborted: the arm does not follow (moved by something else?)")
                return
            self._publish_dq((q1 - q0) * ds)
            return
        err = q1 - q
        if float(np.abs(err).max()) < SETTLE_TOL or t > T + SETTLE_TIMEOUT_S:
            self.stop(f"done (max joint error {np.degrees(np.abs(err).max()):.2f} deg)")
            return
        self._publish_dq(np.clip(SETTLE_GAIN * err, -SETTLE_MAX_SPEED, SETTLE_MAX_SPEED))

    def _retarget_ik(self) -> None:
        """Gives the IK node the reached pose as its target (its end-effector frame: grasp_point)."""
        frame = f"{self.arm}_grasp_point"
        try:
            tf = self._tf.lookup_transform("world", frame, Time(), timeout=Duration(seconds=0.2))
        except Exception as e:
            self.node.get_logger().warning(
                f"{self.arm}: no TF world <- {frame} ({e}); the IK keeps its old target and may pull the arm back.")
            return
        req = SetPoseStamped.Request()
        req.pose = PoseStamped()
        req.pose.header.stamp = self.node.get_clock().now().to_msg()
        req.pose.header.frame_id = "world"
        t, r = tf.transform.translation, tf.transform.rotation
        req.pose.pose.position.x, req.pose.pose.position.y, req.pose.pose.position.z = t.x, t.y, t.z
        req.pose.pose.orientation = r
        if self._pose_client.service_is_ready():
            self._pose_client.call_async(req)
        else:
            self.node.get_logger().warning(f"{self.arm}: {self.ns}/target_pose not ready; IK not re-targeted.")


def make_joint_home(node, home_cfg: dict, callback_group=None) -> JointHome | None:
    """The joint home for a client's ``home_pose`` entry (arm from ``ns`` or its ``topic``:
    /franka_right/target_pose -> franka_right), or None (then the Cartesian home is used)."""
    home_cfg = home_cfg or {}
    ns = home_cfg.get("ns") or str(home_cfg.get("topic") or "").strip("/").split("/")[0]
    if not ns:
        return None
    try:
        keys = ("home_joints", "home_joint_max_speed", "home_joint_min_duration")
        own = {k: home_cfg[k] for k in keys if k in home_cfg}
        spec = load_home(ns, own if "home_joints" in own else None)
    except Exception as e:
        node.get_logger().error(f"Joint home of {ns} disabled ({e}); H uses the Cartesian home.")
        return None
    if spec is None:
        return None
    node.get_logger().info(f"Joint home of {ns}: {spec['joints'].round(4).tolist()}")
    return JointHome(node, ns, spec, callback_group=callback_group)


def main() -> None:
    """Prints the measured joints of both arms as home_joints lines."""
    import time

    import rclpy
    rclpy.init()
    node = rclpy.create_node("joint_home_capture")
    homes = {ns: JointHome(node, ns, {"joints": np.zeros(7), "max_speed": 1.0, "min_duration": 1.0})
             for ns in ("franka_right", "franka_left")}
    t0 = time.time()
    while time.time() - t0 < 3.0 and any(h.measured() is None for h in homes.values()):
        rclpy.spin_once(node, timeout_sec=0.1)
    for ns, h in homes.items():
        q = h.measured()
        print(f"{ns}: " + ("no joint states" if q is None else
                           f"home_joints: [{', '.join(f'{v:.4f}' for v in q)}]   # joint1..7 [rad]"))
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
