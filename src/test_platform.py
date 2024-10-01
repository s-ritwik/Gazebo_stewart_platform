#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import math
import time

def main():
    # Initialize the ROS node
    rospy.init_node('stewart_platform_pose_publisher', anonymous=True)

    # Create a publisher to the '/stewart/platform_pose' topic
    pub = rospy.Publisher('/stewart/platform_pose', Twist, queue_size=10)

    # Set the publishing rate in Hz
    rate = rospy.Rate(50)  # 50 Hz

    # Duration for each phase in seconds
    duration_phase1 = 100.0
    duration_phase2 = 20.0
    duration_phase3 = 20.0
    total_duration = duration_phase1 + duration_phase2 + duration_phase3

    # Motion parameters
    amplitude_z = 0.8        # Amplitude in meters for vertical motion
    amplitude_roll_deg = 20.0    # Amplitude in degrees for roll motion
    amplitude_roll_rad = math.radians(amplitude_roll_deg)  # Convert degrees to radians
    frequency = 0.2              # Frequency in Hz

    # Use real time instead of simulation time
    start_time = time.time()

    # Send the platform to the initial position
    initial_msg = Twist()
    initial_msg.linear.x = 0.0
    initial_msg.linear.y = 0.0
    initial_msg.linear.z = 0.0
    initial_msg.angular.x = 0.0
    initial_msg.angular.y = 0.0
    initial_msg.angular.z = 0.0
    rospy.loginfo("Moving platform to initial position.")
    for _ in range(50):  # Publish initial position for 1 second at 50 Hz
        pub.publish(initial_msg)
        rate.sleep()

    while not rospy.is_shutdown():
        # Get the current time
        current_time = time.time()
        elapsed_time = current_time - start_time

        # Create a new Twist message
        msg = Twist()
        
        if elapsed_time < 20:
            z=0
            roll=0
            rospy.loginfo("Stable mode Z")
        elif elapsed_time < duration_phase1:
            
            # Phase 1: Vertical motion
            t = elapsed_time
            z = amplitude_z * math.sin(2 * math.pi * frequency * t)
            roll = 0.0
            rospy.loginfo("Phase 1 - Vertical Motion: z = {:.3f} m".format(z))

        elif elapsed_time < duration_phase1 + duration_phase2:
            # Phase 2: Roll motion
            t = elapsed_time - duration_phase1
            z = 0.0
            roll = amplitude_roll_rad * math.sin(2 * math.pi * frequency * t)
            rospy.loginfo("Phase 2 - Roll Motion: roll = {:.3f} rad".format(roll))

        elif elapsed_time < total_duration:
            # Phase 3: Combined motion
            t = elapsed_time - duration_phase1 - duration_phase2
            z = amplitude_z * math.sin(2 * math.pi * frequency * t)
            roll = amplitude_roll_rad * math.sin(2 * math.pi * frequency * t)
            rospy.loginfo("Phase 3 - Combined Motion: z = {:.3f} m, roll = {:.3f} rad".format(z, roll))

        else:
            # Motion completed
            rospy.loginfo("Motion completed. Shutting down node.")
            break

        # Assign computed values to the message
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = z
        msg.angular.x = roll  # Roll angle in radians
        msg.angular.y = 0.0   # Pitch angle
        msg.angular.z = 0.0   # Yaw angle

        # Publish the message
        pub.publish(msg)

        # Sleep to maintain the loop rate
        rate.sleep()

    rospy.signal_shutdown("Motion sequence completed.")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
