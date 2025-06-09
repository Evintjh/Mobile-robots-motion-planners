from move_base_msgs.msg import *
from std_srvs.srv import Empty
import rospy
from tf.transformations import quaternion_from_euler
import actionlib
from actionlib_msgs.msg import GoalStatusArray, GoalID
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseFeedback
# from geometry_msgs import PoseStamped

class MissionPlanner:
    def __init__(self, reference="odom"):
        self._action_topic = "/move_base"
        self._cancel_topic = "/move_base/cancel"
        self._clear_costmaps = "/move_base/clear_costmaps"

        self._client = actionlib.SimpleActionClient(
            self._action_topic, MoveBaseAction)
        self.move_base_cancel_goal_pub = rospy.Publisher(
            self._cancel_topic, GoalID, queue_size=1)
        # self._client = actionlib.SimpleActionClient(self._action_topic, MoveBaseAction)
        self.reference = reference
        print(self._client)
        rospy.loginfo("Waiting for move_base action server...")
        # self._client.wait_for_server()
        print("ready")
        self.ep_id = 0
        self.goal_pose = [10, 10]

    def generate_random_goal(self):
        [x, y, _] = self.get_self_stateGT()
        x_cor, y_cor = self.get_goal_point(x)
        self.goal_pose = [x_cor, y_cor]
        self.direction = -1

        # Markers
        self.marker.pose.position.x = self.goal_pose[0]
        self.marker.pose.position.y = self.goal_pose[1]
        self.vis_marker.publish(self.marker)
        if not self.goal_spawn:
            self.visualise_goal()
        self.ready = False

        # Stats
        self.init_pose = [x, y]
        self.prev_pose = [x, y]
        self.total_dist = 0
        self.speed_list = []
        return self.goal_pose
    
    # ready will be given everytime there's a reset
    def ready_callback(self, msg):
        if msg.data: 
            self.set_goal(self.goal_pose, self.ep_id)

            if self.ep_id == 100:
                self.cancel_goal()
            
            else:
                self.ep_id += 1



    # def place_holder(self):
    #     while True:
    #         self.set_goal(self.goal_pose, self.ep_id)

    def set_goal(self, pose, ep):
        print("done1")
        rospy.wait_for_service(self._clear_costmaps)
        print("done2")
        serv = rospy.ServiceProxy(self._clear_costmaps, Empty)
        serv()
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = self.reference

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

    def _wait_for_navigation(self):
        """Wait for navigation system to be up
        """
        services = [
            'move_base/local_costmap/set_parameters',
            'move_base/global_costmap/set_parameters',
            'move_base/set_parameters'
        ]
        for service in services:
            while True:
                try:
                    rospy.loginfo('Waiting for service %s to be up', service)
                    rospy.wait_for_service(service)
                    break
                except rospy.ROSException:
                    rospy.sleep(0.5)

    def cancel_goal(self):
        self.move_base_cancel_goal_pub.publish(GoalID())
        rospy.loginfo('Cancelled active action goal.')
        rospy.sleep(0.2)

if __name__ == '__main__':
    try:
        rospy.init_node('mission_planner', anonymous=True) 
        MP = MissionPlanner()
        # MP.place_holder()
    except rospy.ROSInterruptException:
        pass
