from droid.controllers.oculus_controller import VRPolicy
from droid.robot_env_teleop import RobotEnv
from droid.trajectory_utils.misc_teleop import collect_trajectory


# Make the robot env
controller = VRPolicy()
env = RobotEnv(action_space="cartesian_velocity")

collect_trajectory(env, controller=controller, atacom_agent=None, save_filepath=None)

print("Collected trajectory")
controller.close()