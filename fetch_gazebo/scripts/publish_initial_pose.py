import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose
from std_msgs.msg import Header
import tf

def publish_initial_pose():
    rospy.init_node("initial_pose_publisher")
    tf_listener = tf.TransformListener()
    initial_pose_pub = rospy.Publisher("initialpose", PoseWithCovarianceStamped, queue_size= 10)
    while True:
        try:
            print(f"waiting for transform")
            (trans, rot) = tf_listener.lookupTransform("odom", "base_link", rospy.Time(0))
            break
        except:
            continue
    initial_pose = PoseWithCovarianceStamped()
    initial_pose.header = Header()
    initial_pose.header.frame_id = "map"
    initial_pose.header.stamp = rospy.get_rostime()
    initial_pose.pose.pose.position.x = trans[0]
    initial_pose.pose.pose.position.y = trans[1]
    initial_pose.pose.pose.position.z = trans[2]

    quaternions = tf.transformations.quaternion_from_euler(rot[0], rot[1], rot[2])
    initial_pose.pose.pose.orientation.x = quaternions[0]
    initial_pose.pose.pose.orientation.y = quaternions[1]
    initial_pose.pose.pose.orientation.z = quaternions[2]
    initial_pose.pose.pose.orientation.w = quaternions[3]

    initial_pose.pose.covariance = [
        0.1, 0, 0, 0, 0, 0,
        0, 0.1, 0, 0, 0, 0,
        0, 0, 0.1, 0, 0, 0,
        0, 0, 0, 0.1, 0, 0,
        0, 0, 0, 0, 0.1, 0,
        0, 0, 0, 0, 0, 0.1
    ]

    while not rospy.is_shutdown():
        initial_pose_pub.publish(initial_pose)
        rospy.Rate(10)


if __name__=="__main__":
    publish_initial_pose()



