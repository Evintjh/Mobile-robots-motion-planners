import os
import argparse
from datetime import datetime
import sys
import time
import math
sys.path.append("/isaac-sim/IsaacSim-ros_workspaces/noetic_ws/src/jackal/jackal_navigation/ros_env.py")
sys.path.append("/isaac-sim/IsaacSim-ros_workspaces/noetic_ws/src/jackal/jackal_navigation/utils")

import matplotlib.pyplot as plt
import numpy as np
import rospy
import torch

# from utils.logger import init_logger
from utils.goal_setter import MissionPlanner
# from utils.statistics import Statistics
from collections import deque

from ros_env import ROSEnv
from tf.transformations import quaternion_from_euler
import actionlib
from actionlib_msgs.msg import GoalStatusArray, GoalID
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseFeedback
from std_srvs.srv import Empty



launch_time = None
time_str = None

TIMEOUT = 300
reward_dist_scale = 1.25
reset_flag = True
OBS_SIZE = 512
MAX_EPISODES = 3
# MAX_EPISODES = 5
stats_print_interval = 20
stageros = False


class JackalEnv(ROSEnv):
    # def __init__(self, time_str, logger, write_outputs):
    def __init__(self):
        self._action_topic = "/move_base"
        self._cancel_topic = "/move_base/cancel"
        self._clear_costmaps = "/move_base/clear_costmaps"
        self.goal_frame_id = "odom"
        self.parser = argparse.ArgumentParser()
        self.parser.add_argument("--test", default=False, action="store_true", help="Run in test mode")
        self.args, self.unknown = self.parser.parse_known_args()
        super().__init__(self.args)  # This calls JackalEnv's __init__

        # self.logger = logger
        self.env = None
        # self.env_class = JackalEnv(args=self.args)
        self.scene_config_list = None
        ## [1.78, 9.66, 0.0645], [1.78, 9.66, 0.0645], [-6.84, 9.14, 0.0645], [-11.7, 9.66, 0.0645]
        # self.spawn_location = None
        self.goal_ready = True
        self.timer_start = rospy.get_time()
        # self.goal = [2.6, 22.5]
        self.goal = [5.58738, 22.5]

        # self.local_goal = [2.6, 15.0]
        self.success_array = deque(maxlen=100)
        self.success_rate = 0
        self.goal_status = 0
        self.ep_id = 0
        # self.first_time = True

        # Reward design
        self.goal_radius = 0.5
        self.collision_distance = 0.2
        self.collision_count = 0
        self.ave_speed = 0.25
        self.timeout_count = 0
        self.final_dist_reward = 0
        self.social_penalty = 5
        self.timeout = 60       # 60s

        self.goal_reward = 0
        self.collision_reward = 0
        self.timeout_reward = 0
        self.distance_reward = 0
        self.social_reward = 0
        self.reward = 0

        # Data collection for rewards
        self.episode_rewards = []
        self.distance_rewards = []
        self.collision_rewards = []
        self.goal_rewards = []
        self.timeout_rewards = []
        self.social_rewards = []
        self.epoch_rewards = []  # Stores rewards over time in one epoch



        self._client = actionlib.SimpleActionClient(
            self._action_topic, MoveBaseAction)
        self.move_base_cancel_goal_pub = rospy.Publisher(
            self._cancel_topic, GoalID, queue_size=1)


        # self.spawn_location = torch.zeros((2, 3), device=self._device)

        # self.time_str = time_str
        # self.read_scene_configs()
        # self.stats_recorder = Statistics(self.logger, write_outputs, self.time_str,
        #                                  MAX_EPISODES)  # Object to record statistics
        print("ready")
        self.scene_description = None


    def goal_status_callback(self, msg):
        """
        Callback for the /move_base/status topic to check the goal status.

        Args:
            msg: GoalStatusArray message containing the list of goal statuses.
        """
        # Iterate through the status list in the message
        for status in msg.status_list:
            if status.status == 3:  # Status 3 means SUCCEEDED
                self.goal_status = status.status
                print("Goal succeeded")
            else:
                self.goal_status = 0
            

    def set_goal(self, pose, ep):
        # print("done1")
        rospy.wait_for_service(self._clear_costmaps)
        serv = rospy.ServiceProxy(self._clear_costmaps, Empty)
        serv()
        print("done2")

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = self.goal_frame_id

        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = pose[0]
        goal.target_pose.pose.position.y = pose[1]
        goal.target_pose.pose.position.z = 0
        q = quaternion_from_euler(0, 0, 0)
        goal.target_pose.pose.orientation.x = q[0]
        goal.target_pose.pose.orientation.y = q[1]
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]
        rospy.loginfo('Sending goal id %d to move_base', ep)

        # Send the action goal for execution
        try:
            self._client.send_goal(goal)
        except Exception as e:
            rospy.logwarn("Unable to send navigation action goal:\n%s" % str(e))
            self._failed = True


    def step_RL(self):
        reward_new = 0.0
        # action = np.clip(action, [0, -self.args.w_max], [self.args.v_max, self.args.w_max])
        # set_vel_msgs = Twist()
        # set_vel_msgs.linear.x = action[0]
        # set_vel_msgs.angular.z = action[1]
        # for i in range(self._skip_frame):
        #     self.action_pub.publish(set_vel_msgs)
        #     self.rate.sleep()
        # print("running")
        self.jackal.apply_wheel_actions(self.jackal_controller.forward(command=[self.linear_vel, self.angular_vel]))

        # observations = self.get_observations()
        # self.last_action = action
        info = {}
        done = False
        done_info = [False, 0] 

        if self.local_goal:
            reward_new, done_info = self.compute_reward(self.local_goal, self.scan)
        done = done_info[0]
        info["done_reason"] = done_info
        if done:
            self.success_array.append(done_info[1] == 1)
            self.success_rate = np.mean(self.success_array)
        
        self.num_steps += 1
        
        
        return reward_new, done, info

        # return observations, reward_new, done, info

    def compute_reward(self, local_goal, scan):

        timeout = False
        time_taken = 0
        done_info = [False, 0]
        # Skip reward calculation for the first epoch
        if self.ep_id == 1:
            print("Skipping reward calculation for the first epoch.")
            self.goal_ready = True
            return 0.0, done_info
            # return 
        # goal_reward = 0
        # collision_reward = 0
        # timeout_reward = 0
        # distance_reward = 0
        # social_reward = 0

        
        dist_to_goal = math.hypot(local_goal[0], local_goal[1])
        # print("dist to goal: ", dist_to_goal)
        # if self.first_time:
        #     initial_dist = dist_to_goal 
        #     self.first_time = False

        if dist_to_goal < self.goal_radius:
            
            print("Reached goal")
            time_taken = rospy.get_time() - self.timer_start    ###
            # print("Time taken: ", time_taken)                   ###

            # self.first_time = True                              ###
            done_info = [True, 1]  # Reach Goalum_steps
            

        elif np.amin(scan) < self.collision_distance:
            print("Collsion")
            self.collision_count += 1                           ###
            # print("Collision count: ",self.collision_count)     ###

            time_taken = rospy.get_time() - self.timer_start
            # self.first_time = True                              ###
            done_info = [True, 2]  # Collision

        elif self.spawn_distance:
            # print("spawn distance: ", self.spawn_distance)
            # print("last dist: ", self.last_distance)
            # print("timeout limit:", self.spawn_distance / self.ave_speed)
            if self.num_steps > 3000:       #####
                print("steps: ", self.num_steps)
                print("Timeout")
                self.timeout_count += 1                             ###
                # print("Timeout count: ",self.timeout_count)         ###
                
                # self.first_time = True                              ###
                # done_info = [True, 3]  # Timeout
                timeout = True
            # else:
            #     done_info = [False, 0]  # Nothing


        # if self.final_dist_reward:
        #     if done_info[0]: #from spawn to goal
        #         self.distance_reward = self.spawn_distance - dist_to_goal
        # else:
        self.distance_reward += (self.last_distance - dist_to_goal) 
        #     if abs(self.distance_reward) > 0.5:
        #         self.distance_reward = 0  # To prevent init bug
    
        self.last_distance = dist_to_goal

        # put the rewards separately instead of being in if, elif conditions because rewards can come from each category in an epoch
        if dist_to_goal < self.goal_radius:
            self.goal_reward = 15  # Reach Goal
            # time.sleep(30)
            # delay_steps = 50000  # Example: 300 steps for ~5 seconds at 60 Hz
            # for _ in range(delay_steps):
            #     self._my_world.step(render=True)
            self.goal_ready = False
        if np.amin(scan) < self.collision_distance:
            self.collision_reward = -15  # Collision
            self.goal_ready = False
        if timeout:
            self.timeout_reward = -15  # Timeout
            self.goal_ready = False
        

        if np.amin(scan) < 0.3:
            self.social_reward = -self.social_penalty  # Collision for getting to near

            #big rotation? can just ignore for now

        self.reward = (self.distance_reward + self.collision_reward + self.goal_reward + self.timeout_reward + self.social_reward) - 3.9503042862910487

        print("ep id: ", self.ep_id, ". reward: ", self.reward)                      ###
        # self.num_steps += 1

        # Collect rewards for plotting
        self.distance_rewards.append(self.distance_reward)
        self.collision_rewards.append(self.collision_reward)
        self.goal_rewards.append(self.goal_reward)
        self.timeout_rewards.append(self.timeout_reward)
        self.social_rewards.append(self.social_reward)
        self.episode_rewards.append(self.reward)

        # Store the reward at each timestep
        self.epoch_rewards.append(self.reward)

        return float(self.reward), done_info

    def plot_epoch_rewards(self):
        """
        Plots the rewards over time within a single epoch.
        """
        timesteps = list(range(len(self.epoch_rewards)))

        plt.figure(figsize=(10, 6))
        plt.plot(timesteps, self.epoch_rewards, label='Reward per Timestep', color='blue', linewidth=2)
        plt.xlabel("Timestep")
        plt.ylabel("Reward")
        plt.title("Reward Over Time (Single Epoch)")
        plt.legend()
        plt.grid(True)

        # ✅ Save the plot
        plt.savefig("epoch_rewards_plot.png")
        plt.show()

        rospy.loginfo("Epoch reward plot saved as 'epoch_rewards_plot.png'")


    

    def reset_jackal(self):
        self.goal_reward = 0
        self.collision_reward = 0
        self.timeout_reward = 0
        self.distance_reward = 0
        self.social_reward = 0
        self.timer_start = rospy.get_time()
        self.ep_id += 1
        self.num_steps = 0
        # self.reward = 0
        # self.first_time = True
        #####
        # reset jackal robot
        #####
        # Randomize jackal position and fixed orientation
        jackal_position = torch.zeros((1, 3)).to('cuda')
        # jackal_random_angle = 2 * torch.pi * torch.rand(size=(1,)).to('cuda')
        # # random_distance = 1.7 + 0.5 * torch.rand(size=(num_resets,), device=self._device)
        # jackal_random_distance = 2.5 + torch.rand(size=(1,)).to('cuda') * (4 - 2.5)

        # jackal_position[:, 0] = jackal_random_distance[:] * torch.cos(jackal_random_angle[:])
        # jackal_position[:, 1] = jackal_random_distance[:] * torch.sin(jackal_random_angle[:])
        # jackal_position[:, 0] = torch.tensor(self.spawn_location[0][0], device='cuda')
        # jackal_position[:, 1] = torch.tensor(self.spawn_location[0][0], device='cuda')
        jackal_position[:, 0] = 1.78
        jackal_position[:, 1] = 9.66

        # # jackal_position += self.spawn_location[env_ids]

        jackal_position[:, 2] = 0.0645
        jackal_orientation = torch.zeros((1, 4)).to('cuda')
        jackal_orientation[:, 0] = 0.70711
        # jackal_orientation[:, 0] = 1
        jackal_orientation[:, 3] = 0.70711
        # self._jackals.set_world_poses(jackal_position, jackal_orientation, indices)

        # Convert tensors to CPU NumPy arrays 
        jackal_position_np = jackal_position.cpu().numpy()
        jackal_orientation_np = jackal_orientation.cpu().numpy()

        self.jackal.set_world_pose(jackal_position_np, jackal_orientation_np)
        current_pose = self.jackal.get_world_pose()
        current_position = current_pose[0]
        # # Wait until the simulator updates the Jackal's position
        # max_attempts = 50
        # attempts = 0
        # while attempts < max_attempts:
        #     current_pose = self.jackal.get_world_pose()
        #     current_position = current_pose[0]
        #     if (
        #         abs(current_position[0] - 2.6) > 0.5 and
        #         abs(current_position[1] - 22.5) > 0.5 
        #     ):
        #         print(f"Reposition confirmed: {current_pose}")
        #         break
        #     time.sleep(0.1)  # Wait for a short time before checking again
        #     attempts += 1

        # if attempts == max_attempts:
        #     print("Warning: Jackal repositioning not fully confirmed.")

        # Update distances for reward calculation
        if self.local_goal:
            self.last_distance = math.hypot(self.goal[0] - current_position[0], self.goal[1] - current_position[1])
            self.spawn_distance = math.hypot(self.goal[0] - current_position[0], self.goal[1] - current_position[1])
            print(f"Spawn distance: {self.spawn_distance}, Last distance: {self.last_distance}")
        else:
            self.spawn_distance = 0
            self.last_distance = 0

        print("Reset complete")
        # print(f"Current position after reset: {current_pose}")
        print("repositioned done")
        # time.sleep(10.0)

        # self.set_goal([2.6, 22.5], self.ep_id)
        print("reset done")
        # self.ready.wait()
        # self.ready.clear()
        # time.sleep(2.0)



        # #####
        # # randomize the position of target
        # #####

        # target_position = torch.zeros((1, 3)).to('cuda')
        # random_angle = 3 * torch.pi / 4 + torch.rand(size=(1,), device=self._device) * (
        #             5 * torch.pi / 4 - 3 * torch.pi / 4) + jackal_random_angle  # 135 degree to 225 degree
        # # random_distance = 1.7 + 0.5 * torch.rand(size=(num_resets,), device=self._device)
        # random_distance = 2.5 + torch.rand(size=(1,), device=self._device) * (4 - 2.5)
        # target_position[:, 0] = random_distance[:] * torch.cos(random_angle[:])
        # target_position[:, 1] = random_distance[:] * torch.sin(random_angle[:])

        # target_position += self.spawn_location[0]
        # # target_position += self.spawn_location[env_ids]
        # target_position[:, 2] = 0.05
        # target_orientation = jackal_orientation.clone()

        # if self.random_target:
        #     # self._targets.set_world_poses(target_position, target_orientation, indices)
        #     self._targets.set_world_poses(target_position, target_orientation, 0)

        # #####
        # # randomize the position of obstacle
        # #####

        # # # randomly init the obstacle
        # # obstacle_position = torch.zeros((1, 3), device=self._device)
        # # obstacle_position[:, 0] = torch.rand(size=(1,), device=self._device) * 2 - 1
        # # obstacle_position[:, 1] = torch.rand(size=(1,), device=self._device) * 2 - 1
        # # obstacle_position += self.spawn_location[env_ids]
        # # obstacle_position[:, 2] = 0
        # # obstacle_orientation = jackal_orientation.clone()

        # # if self.random_obstacle:
        # #     self._obstacles.set_world_poses(obstacle_position, obstacle_orientation, indices)

        # #####
        # # according the limitation of omnigym, we should reset the velocities.
        # # reference: https://github.com/NVIDIA-Omniverse/OmniIsaacGymEnvs/blob/main/docs/framework/limitations.md
        # # This part should be verified later
        # #####
        # # TODO: reset the velocities of self._jackals, this reset seems weired since it may be covered by other actions.
        # zero_joint_velocites = torch.zeros(size=(1, 2), device=self._device)
        # zero_joint_velocites = zero_joint_velocites.detach().cpu()

        # articulation_action = self._articulation_controller.forward(zero_joint_velocites)
        # articulation_action = torch.from_numpy(articulation_action).float()
        # # self._jackals.set_joint_velocities(articulation_action, indices=env_ids)
        # self._jackals.set_joint_velocities(articulation_action, indices=0)

        # zero_velocites = torch.zeros(size=(1, 6), device=self._device)
        # self._jackals.set_velocities(zero_velocites, indices=0)





        # #####
        # # reset jackal robot
        # #####
        # # Randomize jackal position and fixed orientation
        # jackal_position = torch.zeros((1, 3), device=self._device)
        # jackal_random_angle = 2 * torch.pi * torch.rand(size=(1,), device=self._device)
        # # random_distance = 1.7 + 0.5 * torch.rand(size=(num_resets,), device=self._device)
        # jackal_random_distance = 2.5 + torch.rand(size=(1,), device=self._device) * (4 - 2.5)

        # jackal_position[:, 0] = jackal_random_distance[:] * torch.cos(jackal_random_angle[:])
        # jackal_position[:, 1] = jackal_random_distance[:] * torch.sin(jackal_random_angle[:])

        # # jackal_position += self.spawn_location[env_ids]

        # jackal_position[:, 2] = 0.0645
        # jackal_orientation = torch.zeros((num_resets, 4), device=self._device)
        # jackal_orientation[:, 0] = 0.70711
        # # jackal_orientation[:, 0] = 1
        # jackal_orientation[:, 3] = 0.70711
        # self._jackals.set_world_poses(jackal_position, jackal_orientation, indices)

        # #####
        # # randomize the position of target
        # #####

        # target_position = torch.zeros((num_resets, 3), device=self._device)
        # random_angle = 3 * torch.pi / 4 + torch.rand(size=(num_resets,), device=self._device) * (
        #             5 * torch.pi / 4 - 3 * torch.pi / 4) + jackal_random_angle  # 135 degree to 225 degree
        # # random_distance = 1.7 + 0.5 * torch.rand(size=(num_resets,), device=self._device)
        # random_distance = 2.5 + torch.rand(size=(num_resets,), device=self._device) * (4 - 2.5)
        # target_position[:, 0] = random_distance[:] * torch.cos(random_angle[:])
        # target_position[:, 1] = random_distance[:] * torch.sin(random_angle[:])

        # target_position += self.spawn_location[env_ids]
        # target_position[:, 2] = 0.05
        # target_orientation = jackal_orientation.clone()

        # if self.random_target:
        #     self._targets.set_world_poses(target_position, target_orientation, indices)

        # #####
        # # randomize the position of obstacle
        # #####

        # # randomly init the obstacle
        # obstacle_position = torch.zeros((num_resets, 3), device=self._device)
        # obstacle_position[:, 0] = torch.rand(size=(num_resets,), device=self._device) * 2 - 1
        # obstacle_position[:, 1] = torch.rand(size=(num_resets,), device=self._device) * 2 - 1
        # obstacle_position += self.spawn_location[env_ids]
        # obstacle_position[:, 2] = 0
        # obstacle_orientation = jackal_orientation.clone()

        # if self.random_obstacle:
        #     self._obstacles.set_world_poses(obstacle_position, obstacle_orientation, indices)

        # #####
        # # according the limitation of omnigym, we should reset the velocities.
        # # reference: https://github.com/NVIDIA-Omniverse/OmniIsaacGymEnvs/blob/main/docs/framework/limitations.md
        # # This part should be verified later
        # #####
        # # TODO: reset the velocities of self._jackals, this reset seems weired since it may be covered by other actions.
        # zero_joint_velocites = torch.zeros(size=(num_resets, 2), device=self._device)
        # zero_joint_velocites = zero_joint_velocites.detach().cpu()

        # articulation_action = self._articulation_controller.forward(zero_joint_velocites)
        # articulation_action = torch.from_numpy(articulation_action).float()
        # self._jackals.set_joint_velocities(articulation_action, indices=env_ids)

        # zero_velocites = torch.zeros(size=(num_resets, 6), device=self._device)
        # self._jackals.set_velocities(zero_velocites, indices=env_ids)

        
    # def reset(self):
    #     self.scan = np.ones((360,))*10
    #     # self.num_steps = 0
    #     self.reset_counter = 0
        
    #     observations = self.get_observations()
    #     self.last_distance = math.hypot(self.local_goal[0], self.local_goal[1])
    #     self.spawn_distance = math.hypot(self.local_goal[0], self.local_goal[1])
    
    #     return observations
    

    def run_batch(self):

        """
        run a batch episodes
        """
        # localise objects
        # self.stats_recorder.initialize_stat()
        # while self.simulation_app.is_running():
        #     self._my_world.step(render=True)
        #     if self._my_world.is_playing():

        while self.simulation_app.is_running():
            self._my_world.step(render=True)
            if self._my_world.is_playing():
                if self._my_world.current_time_step_index == 0:
                    self._my_world.reset()
                    self.jackal_controller.reset()
            # print(self.linear_vel, self.angular_vel)
            # self.spawn_location = self.jackal.get_world_pose()

            if self.ep_id == MAX_EPISODES:
                print("End of evaluation. Loading results.")
                self.plot_epoch_rewards()  # Plot rewards when done
                break

            # if ep_id == 0 or not ep_id % 1000:
            # elif self.ep_id == 0 or self.goal_status:
            #     print("reset")
            #     self.reset_jackal()

        
            if rospy.get_time() - self.timer_start > 5.0 and self.goal_ready:
                # self.set_goal([2.6, 22.5], self.ep_id)
                self.reset_jackal()

                # Non-blocking delay using simulation steps
                delay_steps = 50  # Example: 300 steps for ~5 seconds at 60 Hz
                for _ in range(delay_steps):
                    self._my_world.step(render=True)

                self.set_goal(self.goal, self.ep_id)

                print("ep ready")
                # self.set_goal([2.6, 15.0], self.ep_id)
                self.goal_ready = False
            
            if not self.goal_ready:
                self.step_RL()
            # print("pos: ", self.jackal.get_world_pose())


            
            # while not rospy.is_shutdown() and ep_id < MAX_EPISODES:
            #     self._my_world.step(render=True)

            #     # localise objects
            #     # env = self.env
            #     result = ''

            #     # self.reset_env_episode(result, ep_id)  # reset env & episode variables
            #     # env.obstacles.step()
            #     terminal = False
            #     step = 1
            #     print("good1")


            #     # while not terminal:
            #     #     print("good3")
            #     #     r, terminal, result = env.get_reward_and_terminate(step, 0, TIMEOUT, reward_dist_scale)
            #     #     step += 1
            #     #     # env.obstacles.step()
            #     #     self.step()
            #     #     env.ready = False
            #     #     speed_next = np.asarray(env.get_self_speed())
            #     #     env.speed_list.append(speed_next)
            #     #     while not env.ready:
            #     #         continue
            #     # if terminal:
            #     #     if result in ['Crashed', 'Time out']:
            #     #         env.control_vel([0, 0])
            #     #         navigator.cancel_goal()

            #     # distance = np.sqrt((env.goal_pose[0] - env.init_pose[0]) ** 2 +
            #     #                    (env.goal_pose[1] - env.init_pose[1]) ** 2)
            #     # self.logger.info(
            #     #     'Env %02d, Goal (%2.1f, %2.1f), Ep %03d, Steps %03d, Dist %2.1f, %s'
            #     #     % (
            #     #         env.index, env.goal_pose[0], env.goal_pose[1], ep_id + 1, step,
            #     #         distance, result))





            # # statistics
            # # self.stats_recorder.store_results(step, result,
            # #                                   ep_id)
            # # if (((ep_id + 1) % stats_print_interval) == 0) or (
            # #         ep_id == MAX_EPISODES - 1):
            # #     self.stats_recorder.print_stats(self.env.index)
            # #     if ep_id == MAX_EPISODES - 1:
            # #         self.stats_recorder.write_all('TEB-Navigation', self.scene_description, stageros)
            #     ep_id += 1
            #     # sleep for 2 seconds to keep consistency with RL
            #     rospy.sleep(2)




    # def play(self):
    #     counter = 0
    #     while self.simulation_app.is_running():
    #         self._my_world.step(render=True)
    #         if self._my_world.is_playing():
    #             if self._my_world.current_time_step_index == 0:
    #                 self._my_world.reset()
    #                 self.jackal_controller.reset()

    #             print(self.linear_vel, self.angular_vel)
    #             self.jackal.apply_wheel_actions(self.jackal_controller.forward(command=[self.linear_vel, self.angular_vel]))
    #             print("Linear Velocity: ", self.jackal.get_linear_velocity())
    #             print("Angular Velocity: ", self.jackal.get_angular_velocity())

    #             # if counter <= 500:
    #             #     self.obstacles.set_linear_velocity(np.array([0.0, 0.5, 0.0]))
    #             #     self.obstacles2.set_linear_velocity(np.array([0.0, 0.5, 0.0]))
                
    #             # elif 500 < counter <= 1000:
    #             #     self.obstacles.set_linear_velocity(np.array([0.0, -0.5, 0.0]))
    #             #     self.obstacles2.set_linear_velocity(np.array([0.0, -0.5, 0.0]))



    #             counter += 1
    #             if counter == 1000:
    #                 counter = 0





    #             # if counter <= 800:
    #             #     self.obstacles.set_linear_velocity(np.array([0.0, 0.25, 0.0]))
                
    #             # elif 800 < counter <= 1600:
    #             #     self.obstacles.set_linear_velocity(np.array([0.0, -0.25, 0.0]))


    #             # counter += 1
    #             # if counter == 1600:
    #             #     counter = 0

    #         if self.args.test is True:
    #             break
    #     self.simulation_app.close()


if __name__ == '__main__':

    try:
        env = JackalEnv()
        env.run_batch()

    except rospy.ROSInterruptException:
        pass


# import os
# import argparse
# import sys
# import time
# import math
# import matplotlib.pyplot as plt
# from collections import deque

# import numpy as np
# import rospy
# import torch
# from tf.transformations import quaternion_from_euler
# import actionlib
# from actionlib_msgs.msg import GoalID
# from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
# from std_srvs.srv import Empty

# sys.path.append("/isaac-sim/IsaacSim-ros_workspaces/noetic_ws/src/jackal/jackal_navigation/ros_env.py")
# sys.path.append("/isaac-sim/IsaacSim-ros_workspaces/noetic_ws/src/jackal/jackal_navigation/utils")
# from ros_env import ROSEnv


# TIMEOUT = 300
# MAX_EPISODES = 100

# class JackalEnv(ROSEnv):
#     def __init__(self):
#         self._action_topic = "/move_base"
#         self._cancel_topic = "/move_base/cancel"
#         self._clear_costmaps = "/move_base/clear_costmaps"
#         self.goal_frame_id = "odom"
        
#         self.parser = argparse.ArgumentParser()
#         self.parser.add_argument("--test", default=False, action="store_true", help="Run in test mode")
#         self.args, _ = self.parser.parse_known_args()
#         super().__init__(self.args)

#         self.goal_ready = True
#         self.timer_start = rospy.get_time()
#         self.goal = [2.6, 22.5]
#         self.success_array = deque(maxlen=100)
#         self.success_rate = 0
#         self.ep_id = 0

#         # Reward components
#         self.goal_radius = 0.5
#         self.collision_distance = 0.2
#         self.social_penalty = 5
#         self.timeout = 60

#         self.goal_reward = 0
#         self.collision_reward = 0
#         self.timeout_reward = 0
#         self.distance_reward = 0
#         self.social_reward = 0
#         self.reward = 0

#         self._client = actionlib.SimpleActionClient(self._action_topic, MoveBaseAction)
#         self.move_base_cancel_goal_pub = rospy.Publisher(self._cancel_topic, GoalID, queue_size=1)

#         # Data for plotting
#         self.episode_rewards = []
#         self.distance_rewards = []
#         self.collision_rewards = []
#         self.goal_rewards = []
#         self.timeout_rewards = []
#         self.social_rewards = []

#         print("Environment initialized and ready.")

#     def compute_reward(self, local_goal, scan):
#         timeout = False
#         done_info = [False, 0]
#         dist_to_goal = math.hypot(local_goal[0], local_goal[1])

#         if dist_to_goal < self.goal_radius:
#             self.goal_reward = 15
#             done_info = [True, 1]  # Reached goal
#         elif np.amin(scan) < self.collision_distance:
#             self.collision_reward = -15
#             done_info = [True, 2]  # Collision
#         elif self.num_steps > 1000:
#             self.timeout_reward = -15
#             timeout = True

#         self.distance_reward += (self.last_distance - dist_to_goal)
#         self.last_distance = dist_to_goal

#         if np.amin(scan) < 0.3:
#             self.social_reward = -self.social_penalty

#         self.reward = (
#             self.distance_reward + self.collision_reward + self.goal_reward + self.timeout_reward + self.social_reward
#         )

#         # Append rewards for plotting
#         self.episode_rewards.append(self.reward)
#         self.distance_rewards.append(self.distance_reward)
#         self.collision_rewards.append(self.collision_reward)
#         self.goal_rewards.append(self.goal_reward)
#         self.timeout_rewards.append(self.timeout_reward)
#         self.social_rewards.append(self.social_reward)

#         return float(self.reward), done_info

#     def plot_rewards(self):
#         plt.figure(figsize=(10, 6))
#         plt.plot(self.episode_rewards, label="Overall Reward", color="blue")
#         plt.plot(self.distance_rewards, label="Distance Reward", color="green")
#         plt.plot(self.collision_rewards, label="Collision Reward", color="red")
#         plt.plot(self.goal_rewards, label="Goal Reward", color="orange")
#         plt.plot(self.timeout_rewards, label="Timeout Reward", color="purple")
#         plt.plot(self.social_rewards, label="Social Reward", color="brown")

#         plt.xlabel("Episodes")
#         plt.ylabel("Reward")
#         plt.title("Reward Components per Episode")
#         plt.legend()
#         plt.grid()
#         plt.show()

#     def run_batch(self):
#         while self.ep_id < MAX_EPISODES:
#             self.reset_jackal()
#             self.set_goal(self.goal, self.ep_id)
#             done = False

#             while not done:
#                 _, done_info = self.compute_reward(self.goal, np.ones(360))  # Example scan data
#                 done = done_info[0]

#             self.ep_id += 1

#         # Plot rewards after all episodes
#         self.plot_rewards()

# if __name__ == "__main__":
#     try:
#         env = JackalEnv()
#         env.run_batch()
#     except rospy.ROSInterruptException:
#         pass