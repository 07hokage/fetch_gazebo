import rospy
import tf
from geometry_msgs.msg import PoseStamped

def save_pose(listener, saved_poses):
    """ Save the current pose with the current timestamp. """
    timestamp = rospy.Time.now().to_sec()
    try:
        listener.waitForTransform("/map", "/base_link", rospy.Time.now(), rospy.Duration(1.0))
        (trans, rot) = listener.lookupTransform("/map", "/base_link", rospy.Time(0))
        saved_poses[timestamp] = {'position': trans, 'orientation': rot}
        rospy.loginfo(f"Pose saved at time {timestamp}: {saved_poses[timestamp]}")
        return timestamp  # Return the timestamp of the saved pose
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        rospy.logwarn("Could not save the pose at the current time.")
        return None

def print_transform(listener, timestamp):
    """ Print the transform between map and base_link at the saved timestamp. """
    try:
        time = rospy.Time.from_sec(timestamp)
        latest_common_time = listener.getLatestCommonTime("/map", "/base_link")
        if time > latest_common_time:
            rospy.logwarn(f"Requested time {timestamp} is in the future relative to available tf data.")
            return

        listener.waitForTransform("/map", "/base_link", time, rospy.Duration(1.0))
        (trans, rot) = listener.lookupTransform("/map", "/base_link", time)
        print(f"Transform at time {timestamp}: Position: {trans}, Orientation: {rot}")
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
        rospy.logwarn(f"Could not get transform at time {timestamp}: {e}")

def main():
    rospy.init_node('pose_saver_and_transform_printer')

    listener = tf.TransformListener()
    saved_poses = {}

    user_input = input("Would you like to save the current pose? (yes/no): ").strip().lower()
    
    if user_input == "yes":
        saved_timestamp = save_pose(listener, saved_poses)
        if saved_timestamp is None:
            rospy.logwarn("Pose saving failed. Exiting.")
            return
    else:
        rospy.loginfo("Exiting without saving any pose.")
        return

    # Continuously print the transform at the saved timestamp
    rate = rospy.Rate(0.25)  # 1 Hz
    while not rospy.is_shutdown():
        print_transform(listener, saved_timestamp)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
