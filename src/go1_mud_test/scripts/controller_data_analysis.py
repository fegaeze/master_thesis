#!/usr/bin/env python

import rospy
from go1_mud_test.msg import ControllerData
from geometry_msgs.msg import WrenchStamped
import tf.transformations as transformations
import tf2_ros
import numpy as np

class ControllerNode:
    def __init__(self):
        rospy.init_node('controller_node', anonymous=True)

        self.controller_pub = rospy.Publisher('/controller/data_analysis', ControllerData, queue_size=10)

        rospy.Subscriber('/ati_ft_data', WrenchStamped, self.force_callback)

        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(60.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.ctrl_output = 0.0
        self.initial_position = 0.0
        self.current_position = 0.0
        self.error_cumulative = 0.0
        self.prev_command = 0.0
        self.prev_error = 0.0
        self.prev_time = rospy.Time.now()
        self.contact_initiated = False

        self.rate = rospy.Rate(10)
        self.run()


    def calculate_pid_control_output(self, feedback_force):
        current_time = rospy.Time.now()
        time_elapsed = current_time - self.prev_time
        delta_time = time_elapsed.to_sec() * 1000.0

        # Check for zero delta_time to prevent division by zero
        if delta_time == 0:
            return self.prev_command

        # Calculate the error between the setpoint and feedback
        error = 30 - feedback_force

        # Calculate the error rate (derivative term)
        error_rate = (error - self.prev_error) / delta_time

        # Update the integral term (anti-windup mechanism)
        self.error_cumulative += error * delta_time

        # Initialize the control command
        command = 0

        # Determine the control action based on the feedback
        kp_pull = 0.03
        kp_push = 0.00035
        ki_pull = 0.000003
        ki_push = 0
        kd_pull = 0.001
        kd_push = 0

        if feedback_force < -30:
            command = (kp_pull * error) + (kd_pull * error_rate) + (ki_pull * self.error_cumulative)
        else:
            command = (kp_push * error) + (kd_push * error_rate) + (ki_push * self.error_cumulative)

        # Store the current command, error, and time for the next iteration
        self.prev_command = command
        self.prev_error = error
        self.prev_time = current_time

        # Return the computed control command
        return command

    def force_callback(self, msg):
        force_feedback = msg.wrench.force.z 
        foot_displacement = 0.0

        try:

            trunk_to_foot_transform = self.tf_buffer.lookup_transform("trunk", "FR_foot", rospy.Time(0))
            trunk_to_hip_transform = self.tf_buffer.lookup_transform("trunk", "FR_hip", rospy.Time(0))

            # Extract translation and rotation data
            foot_position = np.array([
                trunk_to_foot_transform.transform.translation.x,
                trunk_to_foot_transform.transform.translation.y,
                trunk_to_foot_transform.transform.translation.z
            ])

            hip_position = np.array([
                trunk_to_hip_transform.transform.translation.x,
                trunk_to_hip_transform.transform.translation.y,
                trunk_to_hip_transform.transform.translation.z
            ])

            rotation = np.array([
                trunk_to_hip_transform.transform.rotation.w,
                trunk_to_hip_transform.transform.rotation.x,
                trunk_to_hip_transform.transform.rotation.y,
                trunk_to_hip_transform.transform.rotation.z
            ])

            # Convert rotation to a rotation matrix
            rotation_matrix = transformations.quaternion_matrix(rotation)[:3, :3]

            # Create transformation matrices
            T_hip_hip_static = np.eye(4)
            T_hip_hip_static[:3, :3] = rotation_matrix

            T_hip_static_trunk = np.eye(4)
            T_hip_static_trunk[:3, 3] = hip_position

            T_foot_trunk = np.eye(4)
            T_foot_trunk[:3, 3] = foot_position

            T_trunk_hip_static = np.linalg.inv(T_hip_static_trunk)
            T_foot_hip_static = np.dot(T_trunk_hip_static, T_foot_trunk)

            self.current_position = T_foot_hip_static[2, 3]

            if self.contact_initiated == False and -force_feedback > 2:
                self.contact_initiated = True
                self.prev_time = rospy.Time.now()
                self.initial_position = self.current_position

            if -force_feedback < 2:
                return

            foot_displacement = self.initial_position - self.current_position

        except tf2_ros.LookupException as ex:
            rospy.logwarn(f"Lookup transform failed: {ex}")

        self.ctrl_output = self.calculate_pid_control_output(force_feedback)
        controller_msg = ControllerData()
        controller_msg.force_error = 30 - -force_feedback
        controller_msg.current_force = force_feedback
        controller_msg.initial_position = self.initial_position
        controller_msg.current_position = self.current_position
        controller_msg.foot_displacement = foot_displacement
        controller_msg.ctrl_output = self.ctrl_output

        self.controller_pub.publish(controller_msg)

    def run(self):
        while not rospy.is_shutdown():
            self.rate.sleep()

if __name__ == '__main__':
    try:
        ControllerNode()
    except rospy.ROSInterruptException:
        pass
