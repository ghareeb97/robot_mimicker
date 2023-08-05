#!/usr/bin/env python

import rospy
import rosbag
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import threading

class RobotMimicker:
    def __init__(self):
        rospy.init_node('robot_mimicker')

        # Initialize joint positions of slave robot
        self.slave_joint_positions = [0.0, 0.0, 0.0, 0.0]

        # Variables for trajectory recording
        self.trajectory = []
        self.recording = False
        self.playback_finished = False
        self.mimic_in_real_time = False

        # Define publishers and subscribers
        self.slave_joint_publisher = rospy.Publisher('/slave/arm_controller/command', JointTrajectory, queue_size=1)
        rospy.Subscriber('/master/joint_states', JointState, self.master_joint_callback)

    def master_joint_callback(self, data):
        # Update joint positions of the slave robot
        if len(data.position) >= 4:
            self.slave_joint_positions = data.position[1:5] 

        # Record the master's trajectory if recording is enabled
        if self.recording:
            self.trajectory.append(data)

    def publish_slave_joint_positions(self):
        # Create a JointTrajectory message to publish the joint positions
        joint_msg = JointTrajectory()
        joint_msg.joint_names = ['joint1', 'joint2', 'joint3', 'joint4']
        point = JointTrajectoryPoint()
        point.positions = self.slave_joint_positions
        point.time_from_start = rospy.Duration(0.1)
        joint_msg.points = [point]

        # Publish the joint positions for the slave robot
        self.slave_joint_publisher.publish(joint_msg)

    def record_trajectory(self, filename):
        # Save the recorded trajectory to a ROS bag file
        bag = rosbag.Bag(filename, 'w')
        for data in self.trajectory:
            bag.write('/master/joint_states', data)
        bag.close()
        rospy.loginfo('Trajectory recorded and saved to {}'.format(filename))

    def menu(self):
        print("Menu:")
        print("0. Stop Mimic in real time")
        print("1. Mimic in real time")
        print("2. Record master trajectory")
        print("3. Stop recording")
        print("4. Play the trajectory on slave")

    def run(self):
        rate = rospy.Rate(10)  # Publish at 10 Hz
        mimic_thread = None
        while not rospy.is_shutdown():
            self.menu()
            choice = input("Enter your choice (0/1/2/3/4): ")

            if choice == '0':
                # Option 0: Exit
                if self.mimic_in_real_time:
                    self.mimic_in_real_time = False
                    mimic_thread.join()
                else:
                    rospy.loginfo('Exiting...')
                    break

            elif choice == '1':
                # Option 1: Mimic in real time
                self.mimic_in_real_time = not self.mimic_in_real_time
                if self.mimic_in_real_time:
                    mimic_thread = threading.Thread(target=self.real_time_mimicking)
                    mimic_thread.start()
                    rospy.loginfo('Mimicking in real time...')
                else:
                    mimic_thread.join()
                    rospy.loginfo('Stopped mimicking.')
            elif choice == '2':
                # Option 2: Start recording master trajectory
                print("Recording master trajectory... (Press 'Ctrl+C' to stop recording)")
                self.recording = True
                self.trajectory = []

            elif choice == '3':
                # Option 3: Stop recording
                self.recording = False
                print("Recording stopped.")
                filename = input("Enter the filename to save the recorded trajectory (e.g., master_trajectory.bag): ")
                self.record_trajectory(filename)

            elif choice == '4':
                # Option 4: Play the trajectory on slave
                filename = input("Enter the filename of the recorded trajectory to play (e.g., master_trajectory.bag): ")
                self.play_trajectory(filename)

            else:
                print("Invalid choice. Please try again.")

            rate.sleep()

    def real_time_mimicking(self):
        while self.mimic_in_real_time:
            self.publish_slave_joint_positions()
            rospy.sleep(0.1)  # duration to control the real-time mimicking speed

    def play_trajectory(self, filename):
        # Load the recorded trajectory from the ROS bag file
        bag = rosbag.Bag(filename, 'r')
        for topic, msg, t in bag.read_messages(topics=['/master/joint_states']):
            self.slave_joint_positions = msg.position[1:5]  
            self.publish_slave_joint_positions()
            rospy.sleep(0.01)  # sleep duration for playback speed (lower is faster)
        bag.close()
        self.playback_finished = True
        rospy.loginfo('Trajectory playback complete.')

if __name__ == '__main__':
    try:
        robot_mimicker = RobotMimicker()
        robot_mimicker.run()
    except rospy.ROSInterruptException:
        pass