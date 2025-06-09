import argparse
import numpy as np
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from omni.isaac.kit import SimulationApp
import subprocess
# from isaac_ros_messages.srv import IsaacPose
# from isaac_ros_messages.srv import IsaacPoseRequest
import math
import asyncio
import omni
from actionlib_msgs.msg import GoalStatusArray, GoalStatus
from move_base_msgs.msg import MoveBaseActionGoal
from std_msgs.msg import Header
from actionlib_msgs.msg import GoalID
import threading
import rospkg 


# Path for saving and retrieving the pose.csv file 
output_file_path = rospkg.RosPack().get_path('jackal_navigation') + "/saved_path/pose3.csv"


class ROSEnv:

    def __init__(self, args, skip_frame=1, physics_dt=1.0 / 60.0, rendering_dt=1.0 / 60.0, seed=0, headless=True) -> None:
        CONFIG = {
            "width": 1280,
            "height": 720,
            "window_width": 1920,
            "window_height": 1080,
            "headless": headless,
            "renderer": "RayTracedLighting",
            "display_options": 3286,
        }

        self.simulation_app = SimulationApp(launch_config=CONFIG)
        from omni.isaac.core.utils.extensions import enable_extension

        self.simulation_app.set_setting("/app/window/drawMouse", True)
        self.simulation_app.set_setting("/ngx/enabled", False)
        self.simulation_app.set_setting("/exts/omni.services.transport.server.http/port", 8201)
        enable_extension("omni.services.streamclient.webrtc")
        enable_extension("omni.isaac.ros_bridge")

        # open stage
        omni.usd.get_context().open_stage("/isaac-sim/IsaacSim-ros_workspaces/noetic_ws/src/jackal/jackal_description/urdf/jackal_warehouse.usda")

        # wait two frames so that stage starts loading
        self.simulation_app.update()
        self.simulation_app.update()

        print("Loading stage...")
        from omni.isaac.core.utils.stage import is_stage_loading

        while is_stage_loading():
            self.simulation_app.update()
        print("Loading Complete")

        self.args = args
        from omni.isaac.core import World
        from omni.isaac.wheeled_robots.robots import WheeledRobot
        from omni.isaac.wheeled_robots.controllers.differential_controller import DifferentialController
        from omni.isaac.sensor import RotatingLidarPhysX
        from omni.isaac.core.objects import FixedCuboid
        from omni.isaac.core.objects import DynamicCuboid
        from omni.isaac.core.prims.xform_prim import XFormPrim
        import omni.graph.core as og
        
        self.og = og
        # vel attributes
        self.left_wheel_vel = []
        self.right_wheel_vel = []
        self.linear_vel = 0
        self.angular_vel = 0
        self.jackal_vel = 0
        self.jackal_alpha = 0
        self.jackal_pos = None
        self.goal_position = None
        self.local_goal = []
        self.scan = np.ones((360,)) * 10
        # self.ready = threading.Event()
        self.last_position = None  # To track the last recorded position
        self.position_threshold = 0.1  # Minimum distance threshold to save a waypoint
        self.recorded_poses = []  # List to store robot's waypoints
        
        self._my_world = World(stage_units_in_meters=1.0)
        # self._my_world.scene.add_default_ground_plane()

        # theta = 90 * np.pi / 180
        # q_rot = np.array([np.cos(theta / 2), 0, 0, np.sin(theta / 2)])

        self.jackal = self._my_world.scene.add(WheeledRobot(prim_path="/World/jackal_1", 
                                                            name="jackal_1",
                                                            wheel_dof_names=["front_right_wheel_joint", "rear_right_wheel_joint", "rear_left_wheel_joint", "front_left_wheel_joint"]))

        # jackal_asset_path = "/isaac-sim/IsaacSim-ros_workspaces/noetic_ws/src/jackal/jackal_description/jackal/jackal.usd"
        # jackal_asset_path = "/isaac-sim/IsaacSim-ros_workspaces/noetic_ws/src/jackal/jackal_description/urdf/jackal_1.usda"
        # self.jackal = self._my_world.scene.add(
        #     WheeledRobot(
        #         prim_path="/jackal",
        #         name="my_jackal",
        #         # wheel_dof_names=["front_right_wheel", "rear_right_wheel", "rear_left_wheel", "front_left_wheel"],
        #         wheel_dof_names=["front_right_wheel_joint", "rear_right_wheel_joint", "rear_left_wheel_joint", "front_left_wheel_joint"],
        #         create_robot=True,
        #         usd_path=jackal_asset_path,
        #         position=np.array([-1, 0, 0.07]),
        #         orientation=q_rot
        #     )
        # )
        self.jackal_controller = DifferentialController(name="simple_control", wheel_radius=0.098, wheel_base=1.4)
        self._my_world.reset()

        # self.obstacles = self._my_world.scene.add(
        #     FixedCuboid(
        #         prim_path="/obstacles_1",
        #         name="obstacles_cube",
        #         scale=np.array([3, 3, 20]),
        #         position=np.array([-4, 0, 0]),
        #         size=0.1,
        #         color=np.array([1.0, 1, 1]),
        #     )
        # )
        self.obstacles = self._my_world.scene.add(
            DynamicCuboid(
                prim_path="/obstacles_1",
                name="obstacles_cube",
                scale=np.array([5, 5, 5]),
                position=np.array([-4, 0, 0]),
                size=0.1,
                color=np.array([1.0, 1, 1]),
            )
        )

        self.obstacles2 = self._my_world.scene.add(
            DynamicCuboid(
                prim_path="/obstacles_2",
                name="obstacles_cube2",
                scale=np.array([5, 5, 5]),
                position=np.array([-8, 0, 0]),
                size=0.1,
                color=np.array([1.0, 1, 1]),
            )
        )
        # self.obstacles3 = self._my_world.scene.get_object("/World/full_warehouse/SM_CardBoxA_3")
        # self.obstacles3.set_world_pose(position=np.array([1.81827, 13.8008,-0.0079]))
        self.obstacles3 = self._my_world.scene.add(
            XFormPrim(
                prim_path="/World/full_warehouse/SM_CardBoxA_3",
                name="SM_CardBoxA_3",
                # scale=np.array([5, 5, 5]),
                position=np.array([1.927, 15.0008, -0.0079]),
            )
        )

        self.obstacles4 = self._my_world.scene.add(
            XFormPrim(
                prim_path="/World/full_warehouse/SM_CardBoxB_01_29",
                name="SM_CardBoxB_01_29",
                # scale=np.array([5, 5, 5]),
                position=np.array([2.49392, 21.888, -0.0211]),
            )
        )

        # self.obstacles5 = self._my_world.scene.add(
        #     XFormPrim(
        #         prim_path="/World/full_warehouse/S_WetFloorSign2",
        #         name="S_WetFloorSign2",
        #         # scale=np.array([5, 5, 5]),
        #         position=np.array([3.20956, 21.77, 0.0]),
        #     )
        # )

        # self.obstacles6 = self._my_world.scene.add(
        #     XFormPrim(
        #         prim_path="/World/full_warehouse/SM_CardBoxA_02_2",
        #         name="SM_CardBoxA_02_2",
        #         # scale=np.array([5, 5, 5]),
        #         position=np.array([1.83674, 18.5015, 0.31025]),
        #     )
        # )

        self.obstacles7 = self._my_world.scene.add(
            XFormPrim(
                prim_path="/World/full_warehouse/SM_CardBoxB_3",
                name="SM_CardBoxB_3",
                # scale=np.array([5, 5, 5]),
                position=np.array([3.58738, 16.0018, 0.25125]),
            )
        )


        # self.lidar = self._my_world.scene.add(
        #     RotatingLidarPhysX(
        #         prim_path="/jackal/front_mount/Lidar",
        #         name="Lidar",
        #     )
        # )

        # self.lidar.enable_visualization(high_lod=False, draw_points=True, draw_lines=True)

        rospy.init_node('ros_env')
        self.laser_sub = rospy.Subscriber('laser_scan', LaserScan, self.laser_scan_callback)
        self.cmd_vel_sub = rospy.Subscriber('cmd_vel', Twist ,self.cmd_vel_callback)
        self.odom_sub = rospy.Subscriber('odom', Odometry ,self.odom_callback)
        self.goal_status_pub = rospy.Publisher('/move_base/status', GoalStatusArray, queue_size=10)
        self.goal_sub = rospy.Subscriber('/move_base/goal', MoveBaseActionGoal ,self.goal_status_eval_callback)
        self.action_graph_setup()

    def cmd_vel_callback(self, msg):
        self.linear_vel = msg.linear.x
        self.angular_vel = msg.angular.z

    # def odom_callback(self, msg):
    #     self.jackal_pos = msg.pose.pose.position
    #     self.jackal_alpha = msg.pose.pose.orientation.z
    #     self.jackal_vel = msg.twist.twist.linear.x
    #     if self.goal_position:
    #         # print("good")
    #         self.local_goal = [self.goal_position.x - self.jackal_pos.x, self.goal_position.y - self.jackal_pos.y]
    #         # self.ready.set()

    def odom_callback(self, msg):
        """
        Callback function to record the robot's position and orientation whenever it moves beyond a threshold.
        """
        self.jackal_pos = msg.pose.pose.position
        self.jackal_alpha = msg.pose.pose.orientation
        self.jackal_vel = msg.twist.twist.linear.x

        if self.goal_position:
            # print("good")
            self.local_goal = [self.goal_position.x - self.jackal_pos.x, self.goal_position.y - self.jackal_pos.y]
            # self.ready.set()

        if self.last_position is None:
            # Initialize last_position on the first callback
            self.last_position = self.jackal_pos
            return

        # Calculate the Euclidean distance between current and last recorded positions
        distance = ((self.jackal_pos.x - self.last_position.x) ** 2 +
                    (self.jackal_pos.y - self.last_position.y) ** 2 +
                    (self.jackal_pos.z - self.last_position.z) ** 2) ** 0.5

        if distance > self.position_threshold:
            # Capture the robot's position as a waypoint
            waypoint = PoseStamped()
            waypoint.header.stamp = rospy.Time.now()
            waypoint.header.frame_id = "odom"  # Adjust the frame_id as needed
            waypoint.pose.position = self.jackal_pos
            waypoint.pose.orientation = self.jackal_alpha
            self.recorded_poses.append(waypoint)

            # Save waypoints to file
            self.save_poses_to_file()

            # Update the last recorded position
            self.last_position = self.jackal_pos

    def save_poses_to_file(self):
        """
        Save recorded poses to a CSV file.
        """
        with open(output_file_path, 'w') as file:
            for pose in self.recorded_poses:
                file.write(
                    f"{pose.pose.position.x},{pose.pose.position.y},{pose.pose.position.z},"
                    f"{pose.pose.orientation.x},{pose.pose.orientation.y},{pose.pose.orientation.z},{pose.pose.orientation.w}\n"
                )
        rospy.loginfo(f"Recorded poses written to {output_file_path}")

    def goal_status_eval_callback(self, msg):
        """
        Evaluates the distance to the goal based on the current position of the Jackal robot.

        Args:
            msg: The MoveBaseActionGoal message containing the goal position.

        Returns:
            None
        """
        self.goal_position = msg.goal.target_pose.pose.position
        print("goal received: ", self.goal_position)
        current_position = self.jackal_pos

        # Calculate Euclidean distance to the goal. Use math.hypot on self.local_goal?
        distance_to_goal = math.sqrt(
            (self.goal_position.x - current_position.x) ** 2 +
            (self.goal_position.y - current_position.y) ** 2
        )

        # print(f"Distance to goal: {distance_to_goal}")

        if distance_to_goal < 0.5:
            print("Goal reached. Publishing goal status.")

            # Create and populate GoalStatusArray message
            status_msg = GoalStatusArray()
            status_msg.header = Header()
            status_msg.header.stamp = rospy.Time.now()

            # Populate GoalStatus
            goal_status = GoalStatus()
            goal_status.goal_id = GoalID()
            goal_status.goal_id.stamp = rospy.Time.now()
            goal_status.goal_id.id = "Success"  # Replace with actual goal ID if available
            goal_status.status = GoalStatus.SUCCEEDED  # Status 3: Succeeded
            goal_status.text = "Goal reached"

            # Add GoalStatus to the array
            status_msg.status_list.append(goal_status)

            # Publish the status
            self.goal_status_pub.publish(status_msg)
            print("Published goal status: SUCCEEDED")


    # def randomise(self):
    #     alpha = 3 * math.pi / 4 + np.random.rand() * (5 * math.pi / 4 - 3 * math.pi / 4) + self.jackal_alpha
    #     # print("alpha:  ",alpha.shape)
    #     r = np.random.uniform(low=2.5, high=4.0)
    #     # print("r:  ",r.shape)

    #     goal_reset_pos = np.multiply(np.sin(alpha) ,r).reshape((1,-1))
    #     goal_reset_pos = np.hstack((goal_reset_pos,np.multiply(np.cos(alpha),r).reshape((1,-1))))
    #     goal_reset_pos = np.hstack((goal_reset_pos,np.ones(1).reshape((1,-1))*0.05))     

    #     # goal_reset_pos = goal_reset_pos + np.array(self.env_pos[reset_idx])
    #     # print("env_pos:  ", np.array(self.env_pos).shape)
    #     # print("goal_reset_pos: ",goal_reset_pos.shape)
    #     # self.goal_prim.set_world_poses(goal_reset_pos,orientations=np.array([1,0,0,0]).reshape((-1,4)))



    #     #random obstacles position
    #     obstacles_reset_pos = np.random.uniform(low=-1, high=1)
    #     obstacles_reset_pos = np.hstack((obstacles_reset_pos, np.random.uniform(low=-1, high=1)))
    #     obstacles_reset_pos = np.hstack((obstacles_reset_pos,0.05))
    #     # obstacles_reset_pos = obstacles_reset_pos + np.array(self.env_pos[reset_idx])
    #     # self.obstacle_prim.set_world_poses(obstacles_reset_pos.reshape((-1,3)),orientations=np.array([1,0,0,0]).reshape((-1,4)))

    #     return goal_reset_pos, obstacles_reset_pos


    # def teleport_client(self, msg):
    #     rospy.wait_for_service("teleport")
    #     try:
    #         teleport = rospy.ServiceProxy("teleport", IsaacPose)
    #         teleport(msg)
    #         return
    #     except rospy.ServiceException as e:
    #         print("Service call failed: %s" % e)


    def laser_scan_callback(self, scan):
        self.scan = np.array(scan.ranges)

    def action_graph_setup(self):
        self.og.Controller.edit(
            {"graph_path": "/ActionGraph", "evaluator_name": "execution"},
            {
                self.og.Controller.Keys.CREATE_NODES: [
                    ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                    ("ClockPublisher", "omni.isaac.ros_bridge.ROS1PublishClock"),
                    ("ReadSimTime", "omni.isaac.core_nodes.IsaacReadSimulationTime"),
                    ("ReadLidarBeams", "omni.isaac.range_sensor.IsaacReadLidarBeams"),
                    ("PublishLaserScan", "omni.isaac.ros_bridge.ROS1PublishLaserScan"),
                    ("ComputeOdom", "omni.isaac.core_nodes.IsaacComputeOdometry"),
                    ("PublishOdometry", "omni.isaac.ros_bridge.ROS1PublishOdometry"),
                    ("PublishOdometryTfXform", "omni.isaac.ros_bridge.ROS1PublishRawTransformTree"),
                    ("tf_baselink_chassis", "omni.isaac.ros_bridge.ROS1PublishTransformTree"),
                    ("tf_frontmount_lidar", "omni.isaac.ros_bridge.ROS1PublishTransformTree"),
                    ("tf_chassis_down", "omni.isaac.ros_bridge.ROS1PublishTransformTree"),
                ],
                self.og.Controller.Keys.CONNECT: [
                    ("OnPlaybackTick.outputs:tick", "tf_baselink_chassis.inputs:execIn"),
                    ("ReadSimTime.outputs:simulationTime", "tf_baselink_chassis.inputs:timeStamp"),
                    ("OnPlaybackTick.outputs:tick", "tf_frontmount_lidar.inputs:execIn"),
                    ("ReadSimTime.outputs:simulationTime", "tf_frontmount_lidar.inputs:timeStamp"),
                    ("OnPlaybackTick.outputs:tick", "tf_chassis_down.inputs:execIn"),
                    ("ReadSimTime.outputs:simulationTime", "tf_chassis_down.inputs:timeStamp"),
                    ("OnPlaybackTick.outputs:tick", "ComputeOdom.inputs:execIn"),
                    ("OnPlaybackTick.outputs:tick", "PublishOdometry.inputs:execIn"),
                    ("OnPlaybackTick.outputs:tick", "PublishOdometryTfXform.inputs:execIn"),
                    ("ReadSimTime.outputs:simulationTime", "PublishOdometry.inputs:timeStamp"),
                    ("ReadSimTime.outputs:simulationTime", "PublishOdometryTfXform.inputs:timeStamp"),
                    ("ComputeOdom.outputs:angularVelocity", "PublishOdometry.inputs:angularVelocity"),
                    ("ComputeOdom.outputs:linearVelocity", "PublishOdometry.inputs:linearVelocity"),
                    ("ComputeOdom.outputs:orientation", "PublishOdometry.inputs:orientation"),
                    ("ComputeOdom.outputs:position", "PublishOdometry.inputs:position"),
                    ("ComputeOdom.outputs:orientation", "PublishOdometryTfXform.inputs:rotation"),
                    ("ComputeOdom.outputs:position", "PublishOdometryTfXform.inputs:translation"),
                    ("OnPlaybackTick.outputs:tick", "ClockPublisher.inputs:execIn"),
                    ("ReadSimTime.outputs:simulationTime", "ClockPublisher.inputs:timeStamp"),
                    ("OnPlaybackTick.outputs:tick", "ReadLidarBeams.inputs:execIn"),
                    ("ReadSimTime.outputs:simulationTime", "PublishLaserScan.inputs:timeStamp"),
                    ("ReadLidarBeams.outputs:execOut", "PublishLaserScan.inputs:execIn"),
                    ("ReadLidarBeams.outputs:azimuthRange", "PublishLaserScan.inputs:azimuthRange"),
                    ("ReadLidarBeams.outputs:depthRange", "PublishLaserScan.inputs:depthRange"),
                    ("ReadLidarBeams.outputs:horizontalFov", "PublishLaserScan.inputs:horizontalFov"),
                    ("ReadLidarBeams.outputs:horizontalResolution", "PublishLaserScan.inputs:horizontalResolution"),
                    ("ReadLidarBeams.outputs:intensitiesData", "PublishLaserScan.inputs:intensitiesData"),
                    ("ReadLidarBeams.outputs:linearDepthData", "PublishLaserScan.inputs:linearDepthData"),
                    ("ReadLidarBeams.outputs:numCols", "PublishLaserScan.inputs:numCols"),
                    ("ReadLidarBeams.outputs:numRows", "PublishLaserScan.inputs:numRows"),
                    ("ReadLidarBeams.outputs:rotationRate", "PublishLaserScan.inputs:rotationRate"),
                ],
                self.og.Controller.Keys.SET_VALUES: [
                    ("ReadLidarBeams.inputs:lidarPrim", "/World/jackal_1/base_link/sick_lms1xx_lidar_frame/Lidar"),
                    ("PublishLaserScan.inputs:topicName", "/laser_scan"),
                    ("PublishLaserScan.inputs:frameId", "Lidar"),
                    ("ComputeOdom.inputs:chassisPrim", "/World/jackal_1"),
                    ("PublishOdometryTfXform.inputs:parentFrameId", "/odom"),
                    ("PublishOdometryTfXform.inputs:childFrameId", "/base_link"),
                    ("tf_baselink_chassis.inputs:parentPrim", "/World/jackal_1/base_link"),
                    ("tf_baselink_chassis.inputs:targetPrims", "/World/jackal_1/base_link/sick_lms1xx_lidar_frame"),
                    ("tf_frontmount_lidar.inputs:parentPrim", "/World/jackal_1/base_link/sick_lms1xx_lidar_frame"),
                    ("tf_frontmount_lidar.inputs:targetPrims", "/World/jackal_1/base_link/sick_lms1xx_lidar_frame/Lidar"),
                    ("tf_chassis_down.inputs:parentPrim", "/World/jackal_1/base_link"),
                    ("tf_chassis_down.inputs:targetPrims", [
                        "/World/jackal_1/front_left_wheel_link","/World/jackal_1/front_right_wheel_link",
                        "/World/jackal_1/rear_left_wheel_link","/World/jackal_1/rear_right_wheel_link"
                    ]),
                ],
            },
        )



# import argparse
# import numpy as np
# import rospy
# from sensor_msgs.msg import LaserScan
# from geometry_msgs.msg import Twist
# from geometry_msgs.msg import Pose
# from geometry_msgs.msg import PoseStamped
# from nav_msgs.msg import Odometry
# from omni.isaac.kit import SimulationApp
# import subprocess
# # from isaac_ros_messages.srv import IsaacPose
# # from isaac_ros_messages.srv import IsaacPoseRequest
# import math
# import asyncio
# import omni
# from actionlib_msgs.msg import GoalStatusArray, GoalStatus
# from move_base_msgs.msg import MoveBaseActionGoal
# from std_msgs.msg import Header
# from actionlib_msgs.msg import GoalID
# import threading

# class ROSEnv:

#     def __init__(self, args, skip_frame=1, physics_dt=1.0 / 60.0, rendering_dt=1.0 / 60.0, seed=0, headless=True) -> None:
#         CONFIG = {
#             "width": 1280,
#             "height": 720,
#             "window_width": 1920,
#             "window_height": 1080,
#             "headless": headless,
#             "renderer": "RayTracedLighting",
#             "display_options": 3286,
#         }

#         self.simulation_app = SimulationApp(launch_config=CONFIG)
#         from omni.isaac.core.utils.extensions import enable_extension

#         self.simulation_app.set_setting("/app/window/drawMouse", True)
#         self.simulation_app.set_setting("/ngx/enabled", False)
#         self.simulation_app.set_setting("/exts/omni.services.transport.server.http/port", 8201)
#         enable_extension("omni.services.streamclient.webrtc")
#         enable_extension("omni.isaac.ros_bridge")

#         # open stage
#         omni.usd.get_context().open_stage("/isaac-sim/IsaacSim-ros_workspaces/noetic_ws/src/jackal/jackal_description/urdf/jackal_warehouse.usda")

#         # wait two frames so that stage starts loading
#         self.simulation_app.update()
#         self.simulation_app.update()

#         print("Loading stage...")
#         from omni.isaac.core.utils.stage import is_stage_loading

#         while is_stage_loading():
#             self.simulation_app.update()
#         print("Loading Complete")

#         self.args = args
#         from omni.isaac.core import World
#         from omni.isaac.wheeled_robots.robots import WheeledRobot
#         from omni.isaac.wheeled_robots.controllers.differential_controller import DifferentialController
#         from omni.isaac.sensor import RotatingLidarPhysX
#         from omni.isaac.core.objects import FixedCuboid
#         from omni.isaac.core.objects import DynamicCuboid
#         import omni.graph.core as og
        
#         self.og = og
#         # vel attributes
#         self.left_wheel_vel = []
#         self.right_wheel_vel = []
#         self.linear_vel = 0
#         self.angular_vel = 0
#         self.jackal_vel = 0
#         self.jackal_alpha = 0
#         self.jackal_pos = None
#         self.goal_position = None
#         self.local_goal = []
#         self.scan = np.ones((360,)) * 10
#         self.ready = threading.Event()
        
#         self._my_world = World(stage_units_in_meters=1.0)
#         # self._my_world.scene.add_default_ground_plane()

#         # theta = 90 * np.pi / 180
#         # q_rot = np.array([np.cos(theta / 2), 0, 0, np.sin(theta / 2)])

#         self.jackal = self._my_world.scene.add(WheeledRobot(prim_path="/World/jackal_1", 
#                                                             name="jackal_1",
#                                                             wheel_dof_names=["front_right_wheel_joint", "rear_right_wheel_joint", "rear_left_wheel_joint", "front_left_wheel_joint"]))

#         # jackal_asset_path = "/isaac-sim/IsaacSim-ros_workspaces/noetic_ws/src/jackal/jackal_description/jackal/jackal.usd"
#         # jackal_asset_path = "/isaac-sim/IsaacSim-ros_workspaces/noetic_ws/src/jackal/jackal_description/urdf/jackal_1.usda"
#         # self.jackal = self._my_world.scene.add(
#         #     WheeledRobot(
#         #         prim_path="/jackal",
#         #         name="my_jackal",
#         #         # wheel_dof_names=["front_right_wheel", "rear_right_wheel", "rear_left_wheel", "front_left_wheel"],
#         #         wheel_dof_names=["front_right_wheel_joint", "rear_right_wheel_joint", "rear_left_wheel_joint", "front_left_wheel_joint"],
#         #         create_robot=True,
#         #         usd_path=jackal_asset_path,
#         #         position=np.array([-1, 0, 0.07]),
#         #         orientation=q_rot
#         #     )
#         # )
#         self.jackal_controller = DifferentialController(name="simple_control", wheel_radius=0.098, wheel_base=1.4)
#         self._my_world.reset()

#         # self.obstacles = self._my_world.scene.add(
#         #     FixedCuboid(
#         #         prim_path="/obstacles_1",
#         #         name="obstacles_cube",
#         #         scale=np.array([3, 3, 20]),
#         #         position=np.array([-4, 0, 0]),
#         #         size=0.1,
#         #         color=np.array([1.0, 1, 1]),
#         #     )
#         # )
#         self.obstacles = self._my_world.scene.add(
#             DynamicCuboid(
#                 prim_path="/obstacles_1",
#                 name="obstacles_cube",
#                 scale=np.array([5, 5, 5]),
#                 position=np.array([-4, 0, 0]),
#                 size=0.1,
#                 color=np.array([1.0, 1, 1]),
#             )
#         )

#         self.obstacles2 = self._my_world.scene.add(
#             DynamicCuboid(
#                 prim_path="/obstacles_2",
#                 name="obstacles_cube2",
#                 scale=np.array([5, 5, 5]),
#                 position=np.array([-8, 0, 0]),
#                 size=0.1,
#                 color=np.array([1.0, 1, 1]),
#             )
#         )



#         # self.lidar = self._my_world.scene.add(
#         #     RotatingLidarPhysX(
#         #         prim_path="/jackal/front_mount/Lidar",
#         #         name="Lidar",
#         #     )
#         # )

#         # self.lidar.enable_visualization(high_lod=False, draw_points=True, draw_lines=True)

#         rospy.init_node('ros_env')
#         self.laser_sub = rospy.Subscriber('laser_scan', LaserScan, self.laser_scan_callback)
#         self.cmd_vel_sub = rospy.Subscriber('cmd_vel', Twist ,self.cmd_vel_callback)
#         self.odom_sub = rospy.Subscriber('odom', Odometry ,self.odom_callback)
#         self.goal_status_pub = rospy.Publisher('/move_base/status', GoalStatusArray, queue_size=10)
#         self.goal_sub = rospy.Subscriber('/move_base/goal', MoveBaseActionGoal ,self.goal_status_eval_callback)
#         self.action_graph_setup()

#     def cmd_vel_callback(self, msg):
#         self.linear_vel = msg.linear.x
#         self.angular_vel = msg.angular.z

#     def odom_callback(self, msg):
#         self.jackal_pos = msg.pose.pose.position
#         self.jackal_alpha = msg.pose.pose.orientation.z
#         self.jackal_vel = msg.twist.twist.linear.x
#         if self.goal_position:
#             # print("good")
#             self.local_goal = [self.goal_position.x - self.jackal_pos.x, self.goal_position.y - self.jackal_pos.y]
#             # self.ready.set()

#     def goal_status_eval_callback(self, msg):
#         """
#         Evaluates the distance to the goal based on the current position of the Jackal robot.

#         Args:
#             msg: The MoveBaseActionGoal message containing the goal position.

#         Returns:
#             None
#         """
#         self.goal_position = msg.goal.target_pose.pose.position
#         print("goal received: ", self.goal_position)
#         current_position = self.jackal_pos

#         # Calculate Euclidean distance to the goal. Use math.hypot on self.local_goal?
#         distance_to_goal = math.sqrt(
#             (self.goal_position.x - current_position.x) ** 2 +
#             (self.goal_position.y - current_position.y) ** 2
#         )

#         # print(f"Distance to goal: {distance_to_goal}")

#         if distance_to_goal < 0.5:
#             print("Goal reached. Publishing goal status.")

#             # Create and populate GoalStatusArray message
#             status_msg = GoalStatusArray()
#             status_msg.header = Header()
#             status_msg.header.stamp = rospy.Time.now()

#             # Populate GoalStatus
#             goal_status = GoalStatus()
#             goal_status.goal_id = GoalID()
#             goal_status.goal_id.stamp = rospy.Time.now()
#             goal_status.goal_id.id = "Success"  # Replace with actual goal ID if available
#             goal_status.status = GoalStatus.SUCCEEDED  # Status 3: Succeeded
#             goal_status.text = "Goal reached"

#             # Add GoalStatus to the array
#             status_msg.status_list.append(goal_status)

#             # Publish the status
#             self.goal_status_pub.publish(status_msg)
#             print("Published goal status: SUCCEEDED")


#     # def randomise(self):
#     #     alpha = 3 * math.pi / 4 + np.random.rand() * (5 * math.pi / 4 - 3 * math.pi / 4) + self.jackal_alpha
#     #     # print("alpha:  ",alpha.shape)
#     #     r = np.random.uniform(low=2.5, high=4.0)
#     #     # print("r:  ",r.shape)

#     #     goal_reset_pos = np.multiply(np.sin(alpha) ,r).reshape((1,-1))
#     #     goal_reset_pos = np.hstack((goal_reset_pos,np.multiply(np.cos(alpha),r).reshape((1,-1))))
#     #     goal_reset_pos = np.hstack((goal_reset_pos,np.ones(1).reshape((1,-1))*0.05))     

#     #     # goal_reset_pos = goal_reset_pos + np.array(self.env_pos[reset_idx])
#     #     # print("env_pos:  ", np.array(self.env_pos).shape)
#     #     # print("goal_reset_pos: ",goal_reset_pos.shape)
#     #     # self.goal_prim.set_world_poses(goal_reset_pos,orientations=np.array([1,0,0,0]).reshape((-1,4)))



#     #     #random obstacles position
#     #     obstacles_reset_pos = np.random.uniform(low=-1, high=1)
#     #     obstacles_reset_pos = np.hstack((obstacles_reset_pos, np.random.uniform(low=-1, high=1)))
#     #     obstacles_reset_pos = np.hstack((obstacles_reset_pos,0.05))
#     #     # obstacles_reset_pos = obstacles_reset_pos + np.array(self.env_pos[reset_idx])
#     #     # self.obstacle_prim.set_world_poses(obstacles_reset_pos.reshape((-1,3)),orientations=np.array([1,0,0,0]).reshape((-1,4)))

#     #     return goal_reset_pos, obstacles_reset_pos


#     # def teleport_client(self, msg):
#     #     rospy.wait_for_service("teleport")
#     #     try:
#     #         teleport = rospy.ServiceProxy("teleport", IsaacPose)
#     #         teleport(msg)
#     #         return
#     #     except rospy.ServiceException as e:
#     #         print("Service call failed: %s" % e)


#     def laser_scan_callback(self, scan):
#         self.scan = np.array(scan.ranges)

#     def action_graph_setup(self):
#         self.og.Controller.edit(
#             {"graph_path": "/ActionGraph", "evaluator_name": "execution"},
#             {
#                 self.og.Controller.Keys.CREATE_NODES: [
#                     ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
#                     ("ClockPublisher", "omni.isaac.ros_bridge.ROS1PublishClock"),
#                     ("ReadSimTime", "omni.isaac.core_nodes.IsaacReadSimulationTime"),
#                     ("ReadLidarBeams", "omni.isaac.range_sensor.IsaacReadLidarBeams"),
#                     ("PublishLaserScan", "omni.isaac.ros_bridge.ROS1PublishLaserScan"),
#                     ("ComputeOdom", "omni.isaac.core_nodes.IsaacComputeOdometry"),
#                     ("PublishOdometry", "omni.isaac.ros_bridge.ROS1PublishOdometry"),
#                     ("PublishOdometryTfXform", "omni.isaac.ros_bridge.ROS1PublishRawTransformTree"),
#                     ("tf_baselink_chassis", "omni.isaac.ros_bridge.ROS1PublishTransformTree"),
#                     ("tf_frontmount_lidar", "omni.isaac.ros_bridge.ROS1PublishTransformTree"),
#                     ("tf_chassis_down", "omni.isaac.ros_bridge.ROS1PublishTransformTree"),
#                 ],
#                 self.og.Controller.Keys.CONNECT: [
#                     ("OnPlaybackTick.outputs:tick", "tf_baselink_chassis.inputs:execIn"),
#                     ("ReadSimTime.outputs:simulationTime", "tf_baselink_chassis.inputs:timeStamp"),
#                     ("OnPlaybackTick.outputs:tick", "tf_frontmount_lidar.inputs:execIn"),
#                     ("ReadSimTime.outputs:simulationTime", "tf_frontmount_lidar.inputs:timeStamp"),
#                     ("OnPlaybackTick.outputs:tick", "tf_chassis_down.inputs:execIn"),
#                     ("ReadSimTime.outputs:simulationTime", "tf_chassis_down.inputs:timeStamp"),
#                     ("OnPlaybackTick.outputs:tick", "ComputeOdom.inputs:execIn"),
#                     ("OnPlaybackTick.outputs:tick", "PublishOdometry.inputs:execIn"),
#                     ("OnPlaybackTick.outputs:tick", "PublishOdometryTfXform.inputs:execIn"),
#                     ("ReadSimTime.outputs:simulationTime", "PublishOdometry.inputs:timeStamp"),
#                     ("ReadSimTime.outputs:simulationTime", "PublishOdometryTfXform.inputs:timeStamp"),
#                     ("ComputeOdom.outputs:angularVelocity", "PublishOdometry.inputs:angularVelocity"),
#                     ("ComputeOdom.outputs:linearVelocity", "PublishOdometry.inputs:linearVelocity"),
#                     ("ComputeOdom.outputs:orientation", "PublishOdometry.inputs:orientation"),
#                     ("ComputeOdom.outputs:position", "PublishOdometry.inputs:position"),
#                     ("ComputeOdom.outputs:orientation", "PublishOdometryTfXform.inputs:rotation"),
#                     ("ComputeOdom.outputs:position", "PublishOdometryTfXform.inputs:translation"),
#                     ("OnPlaybackTick.outputs:tick", "ClockPublisher.inputs:execIn"),
#                     ("ReadSimTime.outputs:simulationTime", "ClockPublisher.inputs:timeStamp"),
#                     ("OnPlaybackTick.outputs:tick", "ReadLidarBeams.inputs:execIn"),
#                     ("ReadSimTime.outputs:simulationTime", "PublishLaserScan.inputs:timeStamp"),
#                     ("ReadLidarBeams.outputs:execOut", "PublishLaserScan.inputs:execIn"),
#                     ("ReadLidarBeams.outputs:azimuthRange", "PublishLaserScan.inputs:azimuthRange"),
#                     ("ReadLidarBeams.outputs:depthRange", "PublishLaserScan.inputs:depthRange"),
#                     ("ReadLidarBeams.outputs:horizontalFov", "PublishLaserScan.inputs:horizontalFov"),
#                     ("ReadLidarBeams.outputs:horizontalResolution", "PublishLaserScan.inputs:horizontalResolution"),
#                     ("ReadLidarBeams.outputs:intensitiesData", "PublishLaserScan.inputs:intensitiesData"),
#                     ("ReadLidarBeams.outputs:linearDepthData", "PublishLaserScan.inputs:linearDepthData"),
#                     ("ReadLidarBeams.outputs:numCols", "PublishLaserScan.inputs:numCols"),
#                     ("ReadLidarBeams.outputs:numRows", "PublishLaserScan.inputs:numRows"),
#                     ("ReadLidarBeams.outputs:rotationRate", "PublishLaserScan.inputs:rotationRate"),
#                 ],
#                 self.og.Controller.Keys.SET_VALUES: [
#                     ("ReadLidarBeams.inputs:lidarPrim", "/World/jackal_1/base_link/sick_lms1xx_lidar_frame/Lidar"),
#                     ("PublishLaserScan.inputs:topicName", "/laser_scan"),
#                     ("PublishLaserScan.inputs:frameId", "Lidar"),
#                     ("ComputeOdom.inputs:chassisPrim", "/World/jackal_1"),
#                     ("PublishOdometryTfXform.inputs:parentFrameId", "/odom"),
#                     ("PublishOdometryTfXform.inputs:childFrameId", "/base_link"),
#                     ("tf_baselink_chassis.inputs:parentPrim", "/World/jackal_1/base_link"),
#                     ("tf_baselink_chassis.inputs:targetPrims", "/World/jackal_1/base_link/sick_lms1xx_lidar_frame"),
#                     ("tf_frontmount_lidar.inputs:parentPrim", "/World/jackal_1/base_link/sick_lms1xx_lidar_frame"),
#                     ("tf_frontmount_lidar.inputs:targetPrims", "/World/jackal_1/base_link/sick_lms1xx_lidar_frame/Lidar"),
#                     ("tf_chassis_down.inputs:parentPrim", "/World/jackal_1/base_link"),
#                     ("tf_chassis_down.inputs:targetPrims", [
#                         "/World/jackal_1/front_left_wheel_link","/World/jackal_1/front_right_wheel_link",
#                         "/World/jackal_1/rear_left_wheel_link","/World/jackal_1/rear_right_wheel_link"
#                     ]),
#                 ],
#             },
#         )

#     # async def move_cuboid(self, duration, vel, counter):
#     #     if counter<=500:
#     #         print("Cuboid movement started")
#     #         self.obstacles.set_linear_velocity(np.array([0.0, vel, 0.0]))
#     #         await asyncio.sleep(duration)
#     #     elif counter<=1000:
#     #         self.obstacles.set_linear_velocity(np.array([0.0, -vel, 0.0]))
#     #         await asyncio.sleep(duration)            
#     #     # self.obstacles.set_linear_velocity(np.array([0.0, 0.0, 0.0]))
#     #     print("Cuboid movement stopped")

#     # def play(self):
#     #     counter = 0
#     #     while self.simulation_app.is_running():
#     #         self._my_world.step(render=True)
#     #         if self._my_world.is_playing():
#     #             if self._my_world.current_time_step_index == 0:
#     #                 self._my_world.reset()
#     #                 self.jackal_controller.reset()

#     #             print(self.linear_vel, self.angular_vel)
#     #             self.jackal.apply_wheel_actions(self.jackal_controller.forward(command=[self.linear_vel, self.angular_vel]))
#     #             print("Linear Velocity: ", self.jackal.get_linear_velocity())
#     #             print("Angular Velocity: ", self.jackal.get_angular_velocity())

#     #             if counter <= 500:
#     #                 self.obstacles.set_linear_velocity(np.array([0.0, 0.5, 0.0]))
#     #                 self.obstacles2.set_linear_velocity(np.array([0.0, 0.5, 0.0]))
                
#     #             elif 500 < counter <= 1000:
#     #                 self.obstacles.set_linear_velocity(np.array([0.0, -0.5, 0.0]))
#     #                 self.obstacles2.set_linear_velocity(np.array([0.0, -0.5, 0.0]))



#     #             counter += 1
#     #             if counter == 1000:
#     #                 counter = 0

#     #             # if counter <= 800:
#     #             #     self.obstacles.set_linear_velocity(np.array([0.0, 0.25, 0.0]))
                
#     #             # elif 800 < counter <= 1600:
#     #             #     self.obstacles.set_linear_velocity(np.array([0.0, -0.25, 0.0]))


#     #             # counter += 1
#     #             # if counter == 1600:
#     #             #     counter = 0

#     #         if self.args.test is True:
#     #             break
#     #     self.simulation_app.close()

# # if __name__ == '__main__':
# #     parser = argparse.ArgumentParser()
# #     parser.add_argument("--test", default=False, action="store_true", help="Run in test mode")
# #     args, unknown = parser.parse_known_args()

# #     try:
# #         jackal_launch = JackalEnv(args=args)
# #         jackal_launch.play()
# #     except rospy.ROSInterruptException:
# #         pass
