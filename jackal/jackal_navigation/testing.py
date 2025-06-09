import os
import argparse
from datetime import datetime
import sys
sys.path.append("/isaac-sim/IsaacSim-ros_workspaces/noetic_ws/src/jackal/jackal_navigation/jackal_launcher.py")
sys.path.append("/isaac-sim/IsaacSim-ros_workspaces/noetic_ws/src/jackal/jackal_navigation/utils")

import numpy as np
import rospy

# from utils.logger import init_logger
from utils.goal_setter import GoalSetter
# from utils.statistics import Statistics
from jackal_launcher import JackalEnv


launch_time = None
time_str = None

TIMEOUT = 300
reward_dist_scale = 1.25
reset_flag = True
OBS_SIZE = 512
MAX_EPISODES = 100
stats_print_interval = 20
stageros = False


class MissionPlanner():
    # def __init__(self, time_str, logger, write_outputs):
    def __init__(self):
        self.parser = argparse.ArgumentParser()
        self.parser.add_argument("--test", default=False, action="store_true", help="Run in test mode")
        self.args, self.unknown = self.parser.parse_known_args()
        # self.logger = logger
        self.env = None
        self.env_class = JackalEnv(args=self.args)
        self.scene_config_list = None
        # navigator = GoalSetter()

        # self.time_str = time_str
        # self.read_scene_configs()
        # self.stats_recorder = Statistics(self.logger, write_outputs, self.time_str,
        #                                  MAX_EPISODES)  # Object to record statistics
        print("ready")
        self.scene_description = None

    def run_batch(self):
        """
        run a batch episodes
        """
        # localise objects
        ep_id = 0
        rospy.sleep(1.0)
        # self.stats_recorder.initialize_stat()

        while not rospy.is_shutdown() and ep_id < MAX_EPISODES:
            # localise objects
            env = self.env
            result = ''

            self.reset_env_episode(result, ep_id)  # reset env & episode variables
            env.obstacles.step()
            terminal = False
            step = 1
            navigator = GoalSetter()
            navigator.set_goal(env.goal_pose, ep_id)

            while not terminal:
                r, terminal, result = env.get_reward_and_terminate(step, 0, TIMEOUT, reward_dist_scale)
                step += 1
                env.obstacles.step()
                env.ready = False
                speed_next = np.asarray(env.get_self_speed())
                env.speed_list.append(speed_next)
                while not env.ready:
                    continue
            if terminal:
                if result in ['Crashed', 'Time out']:
                    env.control_vel([0, 0])
                    navigator.cancel_goal()

            distance = np.sqrt((env.goal_pose[0] - env.init_pose[0]) ** 2 +
                               (env.goal_pose[1] - env.init_pose[1]) ** 2)
            self.logger.info(
                'Env %02d, Goal (%2.1f, %2.1f), Ep %03d, Steps %03d, Dist %2.1f, %s'
                % (
                    env.index, env.goal_pose[0], env.goal_pose[1], ep_id + 1, step,
                    distance, result))

            # statistics
            # self.stats_recorder.store_results(step, result,
            #                                   ep_id)
            # if (((ep_id + 1) % stats_print_interval) == 0) or (
            #         ep_id == MAX_EPISODES - 1):
            #     self.stats_recorder.print_stats(self.env.index)
            #     if ep_id == MAX_EPISODES - 1:
            #         self.stats_recorder.write_all('TEB-Navigation', self.scene_description, stageros)
            ep_id += 1
            # sleep for 2 seconds to keep consistency with RL
            rospy.sleep(2)


if __name__ == '__main__':

    try:
        MP = MissionPlanner()
        MP.env_class.play()
    except rospy.ROSInterruptException:
        pass