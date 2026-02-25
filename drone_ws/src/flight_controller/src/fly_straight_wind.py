#!/usr/bin/env python3

import rospy
import time
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest, CommandTOL, CommandTOLRequest

current_state = State()
current_pose = PoseStamped()

def state_cb(msg):
    global current_state
    current_state = msg

def pose_cb(msg):
    global current_pose
    current_pose = msg

def fly_straight():
    rospy.init_node('fly_straight_wind_node', anonymous=True)

    # Subscribers
    rospy.Subscriber("mavros/state", State, state_cb)
    rospy.Subscriber("mavros/local_position/pose", PoseStamped, pose_cb)
    local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)

    # Services
    rospy.wait_for_service("/mavros/cmd/arming")
    arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)

    rospy.wait_for_service("/mavros/set_mode")
    set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)

    rospy.wait_for_service("/mavros/cmd/takeoff")
    takeoff_client = rospy.ServiceProxy("mavros/cmd/takeoff", CommandTOL)

    # Wait for Flight Controller connection
    rate = rospy.Rate(20)
    while not rospy.is_shutdown() and not current_state.connected:
        rospy.loginfo("Waiting for FCU connection...")
        rate.sleep()

    rospy.loginfo("FCU Connected")

    # Target Pose (Forward 5m, Up 2m)
    target_pose = PoseStamped()
    target_pose.header.frame_id = "map"
    target_pose.pose.position.x = 5.0
    target_pose.pose.position.y = 0.0
    target_pose.pose.position.z = 2.0
    target_pose.pose.orientation.w = 1.0 # Valid orientation

    # Takeoff Pose (Vertical only)
    takeoff_pose = PoseStamped()
    takeoff_pose.header.frame_id = "map"
    takeoff_pose.pose.position.x = 0.0
    takeoff_pose.pose.position.y = 0.0
    takeoff_pose.pose.position.z = 2.0
    takeoff_pose.pose.orientation.w = 1.0

    # 1. Set Mode to GUIDED
    # ArduPilot requires GUIDED mode for offboard control
    last_req = rospy.Time.now()
    
    # Send setpoints before switch
    for i in range(100):
        takeoff_pose.header.stamp = rospy.Time.now()
        local_pos_pub.publish(takeoff_pose)
        rate.sleep()

    while not rospy.is_shutdown() and current_state.mode != "GUIDED":
        if (rospy.Time.now() - last_req) > rospy.Duration(5.0):
            rospy.loginfo("Setting mode to GUIDED...")
            try:
                if set_mode_client(custom_mode="GUIDED").mode_sent:
                    rospy.loginfo("GUIDED enabled")
            except rospy.ServiceException as e:
                rospy.logwarn(f"Service call failed: {e}")
            last_req = rospy.Time.now()
        rate.sleep()

    # 2. Arm
    last_req = rospy.Time.now()
    while not rospy.is_shutdown() and not current_state.armed:
        if (rospy.Time.now() - last_req) > rospy.Duration(5.0):
            rospy.loginfo("Arming vehicle...")
            try:
                if arming_client(value=True).success:
                    rospy.loginfo("Vehicle armed")
            except rospy.ServiceException as e:
                rospy.logwarn(f"Service call failed: {e}")
            last_req = rospy.Time.now()
        local_pos_pub.publish(takeoff_pose) # Keep sending setpoints
        rate.sleep()

    # 3. Takeoff
    rospy.loginfo("Taking off to 2m...")
    try:
        # Latitude/Longitude/Yaw are 0 for local takeoff
        # Note: MAV_CMD_NAV_TAKEOFF (22)
        resp = takeoff_client(altitude=2.0, latitude=0, longitude=0, min_pitch=0, yaw=0)
        if resp.success:
            rospy.loginfo("Takeoff command sent successfully")
        else:
            rospy.logwarn(f"Takeoff command failed (result code: {resp.result})")
    except rospy.ServiceException as e:
        rospy.logwarn(f"Takeoff service call failed: {e}")

    # Allow time to rise
    rospy.loginfo("Waiting for altitude > 1.5m...")
    
    start_wait = rospy.Time.now()
    while not rospy.is_shutdown():
        # Check altitude
        if current_pose.pose.position.z > 1.5:
            rospy.loginfo("Reached safe altitude")
            break
            
        # Timeout after 20 seconds
        if (rospy.Time.now() - start_wait) > rospy.Duration(20.0):
            rospy.logwarn("Takeoff timed out!")
            break
            
        rate.sleep()



    rospy.loginfo("Flying to target: x=5.0, y=0.0, z=2.0")

    # 4. Fly to Target
    while not rospy.is_shutdown():
        target_pose.header.stamp = rospy.Time.now()
        local_pos_pub.publish(target_pose)
        rate.sleep()

if __name__ == '__main__':
    try:
        fly_straight()
    except rospy.ROSInterruptException:
        pass
