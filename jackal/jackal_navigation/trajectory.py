import os
import csv
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Pose
import rospy
from visualization_msgs.msg import Marker
import rospkg 


# Path for saving and retrieving the pose.csv file 
input_file_path = rospkg.RosPack().get_path('jackal_navigation') + "/saved_path/pose3.csv"

def construct_path(poses):
    # file_path = os.path.expanduser('~/catkin_ws/src/follow_waypoints/saved_path/{}.csv'.format(poses))
    
    path = Path()
    path.header.frame_id = "odom"  # Set the appropriate frame ID

    with open(input_file_path) as csv_file:
        csv_reader = csv.reader(csv_file, delimiter = ',')  # Use '\t' as the delimiter for your CSV
        for row in csv_reader:

            pose = PoseStamped()
            pose.header.frame_id = "odom" 
            pose.pose.position.x = float(row[0])
            pose.pose.position.y = float(row[1])
            pose.pose.position.z = float(row[2])
            pose.pose.orientation.x = float(row[3])
            pose.pose.orientation.y = float(row[4])
            pose.pose.orientation.z = float(row[5])
            pose.pose.orientation.w = float(row[6])

            # Append the pose to the path
            path.poses.append(pose)

    return path


def main():
    rospy.init_node('waypoints_path')

    rospy.loginfo("generating path...")

    path_pub = rospy.Publisher('/target_path', Path, queue_size=1)

    rate = rospy.Rate(10)  # Adjust the rate as needed
    pose = 'pose2'
    while not rospy.is_shutdown():
        path = construct_path(pose)
        path_pub.publish(path)
        rate.sleep()

    rospy.loginfo("path generated")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass