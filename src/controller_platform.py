#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
import math
import time
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import threading
import csv  # To handle CSV reading

# Global variables for storing the data
time_list = []
desired_heave_list = []
current_heave_list = []
desired_roll_list = []  # List to store desired roll values

# New lists to store desired_heave and roll values from CSV files
csv_desired_heave_values = []
csv_desired_roll_values = []

def read_csv_file(file_path, store_list):
    try:
        with open(file_path, 'r') as csvfile:
            csvreader = csv.reader(csvfile)
            for row in csvreader:
                if row:  # Ensure there's data in the row
                    store_list.append(float(row[0]))  # Assuming values are in the first column
    except FileNotFoundError:
        rospy.logerr(f"CSV file {file_path} not found.")
        return

def send_sinusoidal_command():
    pub = rospy.Publisher('stewart/platform_pose', Twist, queue_size=10)
    rospy.init_node('stewart_command_sinusoidal', anonymous=True)
    rate = rospy.Rate(20)  # Set rate to 20Hz

    start_time = time.time()

    # PD controller previous errors
    roll_error_prev = 0
    pitch_error_prev = 0
    heave_error_prev = 0
    yaw_error_prev = 0
    x_error_prev = 0
    y_error_prev = 0

    # Control gains
    Kp = 0.1  # Proportional gain
    Kd = 0  # Derivative gain

    # Initialize last command values for filtering
    last_roll_command = 0
    last_pitch_command = 0
    last_heave_command = 0
    last_yaw_command = 0
    last_x_command = 0
    last_y_command = 0

    # Filtering factor for low-pass filter
    alpha = 0.1

    heave_index = 0  # Track the index for reading from the heave CSV
    roll_index = 0   # Track the index for reading from the roll CSV

    while not rospy.is_shutdown():
        current_time = time.time() - start_time

        # Ensure we have enough data in both the CSV files
        if heave_index >= len(csv_desired_heave_values) or roll_index >= len(csv_desired_roll_values):
            rospy.logwarn("Reached the end of CSV heave or roll values.")
            break

        # Get desired heave and roll from the CSV files
        desired_heave = csv_desired_heave_values[heave_index] / 1.4
        desired_roll = 0#math.pi/180*csv_desired_roll_values[roll_index] 
        heave_index += 1  # Move to the next heave value at each iteration
        roll_index += 1  # Move to the next roll value at each iteration

        # Desired values for pitch, and other axes (constant for now)
        desired_pitch = 0
        desired_x = 0
        desired_y = 0
        desired_yaw = 0

        # Mock feedback (replace with actual feedback from sensors)
        feedback_roll = last_roll_command
        feedback_pitch = last_pitch_command
        feedback_heave = last_heave_command
        feedback_x = last_x_command
        feedback_y = last_y_command
        feedback_yaw = last_y_command

        # Calculate errors
        roll_error = desired_roll - feedback_roll
        pitch_error = desired_pitch - feedback_pitch
        heave_error = desired_heave - feedback_heave
        yaw_error = desired_yaw - feedback_yaw
        x_error = desired_x - feedback_x
        y_error = desired_y - feedback_y

        # PD control for roll, pitch, and heave
        roll_command = desired_roll + Kp * roll_error + Kd * (roll_error - roll_error_prev)
        pitch_command = desired_pitch + Kp * pitch_error + Kd * (pitch_error - pitch_error_prev)
        heave_command = desired_heave + Kp * heave_error + Kd * (heave_error - heave_error_prev)
        yaw_command = desired_yaw + Kp * yaw_error + Kd * (yaw_error - yaw_error_prev)
        x_command = desired_x + Kp * x_error + Kd * (x_error - x_error_prev)
        y_command = desired_y + Kp * y_error + Kd * (y_error - y_error_prev)

        # Update previous errors
        roll_error_prev = roll_error
        pitch_error_prev = pitch_error
        heave_error_prev = heave_error
        yaw_error_prev = yaw_error
        x_error_prev = x_error
        y_error_prev = y_error

        # Apply low-pass filter to smooth the commands
        filtered_roll_command = alpha * roll_command + (1 - alpha) * last_roll_command
        filtered_pitch_command = alpha * pitch_command + (1 - alpha) * last_pitch_command
        filtered_heave_command = alpha * heave_command + (1 - alpha) * last_heave_command
        filtered_yaw_command = alpha * yaw_command + (1 - alpha) * last_yaw_command
        filtered_x_command = alpha * x_command + (1 - alpha) * last_x_command
        filtered_y_command = alpha * y_command + (1 - alpha) * last_y_command

        # Update last commands
        last_roll_command = filtered_roll_command
        last_pitch_command = filtered_pitch_command
        last_heave_command = filtered_heave_command
        last_yaw_command = filtered_yaw_command
        last_x_command = filtered_x_command
        last_y_command = filtered_y_command

        # Store data for plotting
        time_list.append(current_time)
        desired_heave_list.append(desired_heave)
        current_heave_list.append(filtered_heave_command)
        desired_roll_list.append(desired_roll)

        # Prepare the Twist message
        command = Twist()
        command.angular.x = filtered_roll_command
        command.angular.y = filtered_pitch_command
        command.linear.z = filtered_heave_command
        command.angular.z = filtered_yaw_command
        command.linear.x = filtered_x_command
        command.linear.y = filtered_y_command

        rospy.loginfo("Sending r: %s, p: %s, Z: %s, X: %s, Y: %s, Yaw: %s", filtered_roll_command,
                     filtered_pitch_command, filtered_heave_command, filtered_x_command, filtered_y_command, filtered_yaw_command)
        pub.publish(command)
        rate.sleep()

def animate(i):
    # Update the plot data
    plt.cla()  # Clear the previous data
    plt.plot(time_list, desired_heave_list, label="Desired Heave")
    plt.plot(time_list, current_heave_list, label="Current Heave", linestyle='--')
    plt.plot(time_list, desired_roll_list, label="Desired Roll", linestyle='-.')
    plt.xlabel("Time (s)")
    plt.ylabel("Values")
    plt.legend()
    plt.title("Desired vs Current Heave and Roll")
    plt.tight_layout()

if __name__ == '__main__':
    try:
        # Initialize the ROS node in the main thread
        rospy.init_node('stewart_command_sinusoidal', anonymous=True)

        # Read desired heave values from the first CSV file
        heave_csv_file_path = "/home/ritwik/prediction_heave/GRU_generalised/train_data_normalised/D1H3_normalised.csv"  # Replace with your heave CSV file path
        read_csv_file(heave_csv_file_path, csv_desired_heave_values)

        # Read desired roll values from the second CSV file
        roll_csv_file_path = "/home/ritwik/catkin_ws/src/test_stewart/src/seastate_3.csv"  # Replace with your roll CSV file path
        read_csv_file(roll_csv_file_path, csv_desired_roll_values)

        # Run the command loop in a separate thread
        command_thread = threading.Thread(target=send_sinusoidal_command)
        command_thread.start()

        # Start the real-time plotting (commented out if not needed)
        # ani = FuncAnimation(plt.gcf(), animate, interval=100)
        # plt.show()

        # Wait for the command loop thread to finish
        command_thread.join()

    except rospy.ROSInterruptException:
        pass
