import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist
import numpy as np
np.set_printoptions(precision=3, suppress=True)

Kp = 3
Kd = 0.1

# norm2 = norma la patrat

class InverseKinematics(Node):

    def __init__(self):
        super().__init__('inverse_kinematics')
        self.joint_subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.listener_callback,
            10)
        self.joint_subscription  # prevent unused variable warning

        self.command_publisher = self.create_publisher(
            Float64MultiArray,
            '/forward_command_controller/commands',
            10
        )
        
        self.cmd_subscription = self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)
        self.current_linear_x = 0.0
        self.current_angular_z = 0.0

        self.pd_timer_period = 1.0 / 200  # 200 Hz
        self.ik_timer_period = 1.0 / 20   # 10 Hz
        self.pd_timer = self.create_timer(self.pd_timer_period, self.pd_timer_callback)
        self.ik_timer = self.create_timer(self.ik_timer_period, self.ik_timer_callback)

        self.joint_positions = None
        self.joint_velocities = None

        # array for all 12 joint positions (4 legs * 3 joints)
        self.target_joint_positions = [0.0] * 12

        self.base_triangle = np.array([
            [0.05, 0.0, -0.12],  # Touchdown
            [-0.05, 0.0, -0.12], # Liftoff
            [0.0, 0.0, -0.09]    # Mid-swing (ridica laba doar 3cm)
        ])

        self.center_fr = np.array([0.07500, -0.08350, 0])
        self.center_fl = np.array([0.07500, 0.08350, 0])
        self.center_br = np.array([-0.07500, -0.08350, 0])
        self.center_bl = np.array([-0.07500, 0.08350, 0])

        self.current_target = 0
        self.t = 0.0

    def cmd_callback(self, msg):
        self.current_linear_x = msg.linear.x
        self.current_angular_z = msg.angular.z

    def listener_callback(self, msg):
        joints_of_interest = ['leg_front_r_1_joint', 'leg_front_r_2_joint', 'leg_front_r_3_joint',
                              'leg_front_l_1_joint', 'leg_front_l_2_joint', 'leg_front_l_3_joint',
                              'leg_back_r_1_joint', 'leg_back_r_2_joint', 'leg_back_r_3_joint',
                              'leg_back_l_1_joint', 'leg_back_l_2_joint', 'leg_back_l_3_joint']
        self.joint_positions = np.array([msg.position[msg.name.index(joint)] for joint in joints_of_interest])
        self.joint_velocities = np.array([msg.velocity[msg.name.index(joint)] for joint in joints_of_interest])

    def forward_kinematics_front_right(self, theta1, theta2, theta3):
        ################################################################################################
        # TODO: Compute the forward kinematics for the front right leg (should be easy after lab 2!)
        ################################################################################################
        def rotation_x(angle):
            # rotation about the x-axis
            return np.array(
                [
                    [1, 0, 0, 0],
                    [0, np.cos(angle), -np.sin(angle), 0],
                    [0, np.sin(angle), np.cos(angle), 0],
                    [0, 0, 0, 1],
                ]
            )

        def rotation_y(angle):
            #rotation about the y-axis
            return np.array (
                [
                    [np.cos(angle), 0, np.sin(angle), 0],
                    [0, 1, 0, 0],
                    [-np.sin(angle), 0, np.cos(angle), 0],
                    [0, 0, 0, 1],
                ]
            )

        def rotation_z(angle):
            #rotation about the z-axis
            return np.array (
                [
                    [np.cos(angle), -np.sin(angle), 0, 0],
                    [np.sin(angle), np.cos(angle), 0, 0],
                    [0, 0, 1, 0],
                    [0, 0, 0, 1],
                ]
            )

        def translation(x, y, z):
            #the translation matrix
            return np.array (
                [
                    [1, 0, 0, x],
                    [0, 1, 0, y],
                    [0, 0, 1, z],
                    [0, 0, 0, 1],
                ]
            )

        # T_0_1 (base_link to leg_front_l_1)
        T_0_1 = translation(0.07500, -0.0445, 0) @ rotation_x(1.57080) @ rotation_z(theta1) # 1.57080 = pi/2

        # T_1_2 (leg_front_l_1 to leg_front_l_2)
        ## TODO: Implement the transformation matrix from leg_front_l_1 to leg_front_l_2
        T_1_2 = translation(0, 0, 0.0390) @ rotation_y(-1.57080) @ rotation_z(theta2)

        # T_2_3 (leg_front_l_2 to leg_front_l_3)
        ## TODO: Implement the transformation matrix from leg_front_l_2 to leg_front_l_3
        T_2_3 = translation(0, -0.0494, 0.0685) @ rotation_y(1.57080) @ rotation_z(theta3)

        # T_3_ee (leg_front_l_3 to end-effector)
        T_3_ee = translation(0.06231, -0.06216, 0.0180)

        # TODO: Compute the final transformation. T_0_ee is the multiplication of the previous transformation matrices
        T_0_ee = T_0_1 @ T_1_2 @ T_2_3 @ T_3_ee

        # TODO: Extract the end-effector position. The end effector position is a 3x1 vector (not in homogenous coordinates)
        end_effector_position = T_0_ee[0:3, 3]

        return end_effector_position

    def forward_kinematics_front_left(self, theta1, theta2, theta3):
        def rotation_x(angle):
            return np.array(
                [
                    [1, 0, 0, 0],
                    [0, np.cos(angle), -np.sin(angle), 0],
                    [0, np.sin(angle), np.cos(angle), 0],
                    [0, 0, 0, 1],
                ]
            )

        def rotation_y(angle):
            return np.array (
                [
                    [np.cos(angle), 0, np.sin(angle), 0],
                    [0, 1, 0, 0],
                    [-np.sin(angle), 0, np.cos(angle), 0],
                    [0, 0, 0, 1],
                ]
            )

        def rotation_z(angle):
            return np.array (
                [
                    [np.cos(angle), -np.sin(angle), 0, 0],
                    [np.sin(angle), np.cos(angle), 0, 0],
                    [0, 0, 1, 0],
                    [0, 0, 0, 1],
                ]
            )

        def translation(x, y, z):
            return np.array (
                [
                    [1, 0, 0, x],
                    [0, 1, 0, y],
                    [0, 0, 1, z],
                    [0, 0, 0, 1],
                ]
            )

        T_0_1 = translation(0.07500, 0.0445, 0) @ rotation_x(-1.57080) @ rotation_z(theta1)
        T_1_2 = translation(0, 0, 0.0390) @ rotation_y(-1.57080) @ rotation_z(theta2)
        T_2_3 = translation(0, 0.0494, 0.0685) @ rotation_y(1.57080) @ rotation_z(theta3)
        T_3_ee = translation(0.06231, 0.06216, 0.0180)

        T_0_ee = T_0_1 @ T_1_2 @ T_2_3 @ T_3_ee
        end_effector_position = T_0_ee[0:3, 3]
        return end_effector_position

    def forward_kinematics_back_right(self, theta1, theta2, theta3):
        def rotation_x(angle):
            return np.array(
                [
                    [1, 0, 0, 0],
                    [0, np.cos(angle), -np.sin(angle), 0],
                    [0, np.sin(angle), np.cos(angle), 0],
                    [0, 0, 0, 1],
                ]
            )

        def rotation_y(angle):
            return np.array (
                [
                    [np.cos(angle), 0, np.sin(angle), 0],
                    [0, 1, 0, 0],
                    [-np.sin(angle), 0, np.cos(angle), 0],
                    [0, 0, 0, 1],
                ]
            )

        def rotation_z(angle):
            return np.array (
                [
                    [np.cos(angle), -np.sin(angle), 0, 0],
                    [np.sin(angle), np.cos(angle), 0, 0],
                    [0, 0, 1, 0],
                    [0, 0, 0, 1],
                ]
            )

        def translation(x, y, z):
            return np.array (
                [
                    [1, 0, 0, x],
                    [0, 1, 0, y],
                    [0, 0, 1, z],
                    [0, 0, 0, 1],
                ]
            )

        T_0_1 = translation(-0.07500, -0.0445, 0) @ rotation_x(1.57080) @ rotation_z(theta1)
        T_1_2 = translation(0, 0, 0.0390) @ rotation_y(-1.57080) @ rotation_z(theta2)
        T_2_3 = translation(0, -0.0494, 0.0685) @ rotation_y(1.57080) @ rotation_z(theta3)
        T_3_ee = translation(0.06231, -0.06216, 0.0180)

        T_0_ee = T_0_1 @ T_1_2 @ T_2_3 @ T_3_ee
        end_effector_position = T_0_ee[0:3, 3]
        return end_effector_position
    
    def forward_kinematics_back_left(self, theta1, theta2, theta3):
        ################################################################################################
        # TODO: Compute the forward kinematics for the back left leg (should be easy after lab 2!)
        ################################################################################################
        def rotation_x(angle):
            # rotation about the x-axis
            return np.array(
                [
                    [1, 0, 0, 0],
                    [0, np.cos(angle), -np.sin(angle), 0],
                    [0, np.sin(angle), np.cos(angle), 0],
                    [0, 0, 0, 1],
                ]
            )

        def rotation_y(angle):
            #rotation about the y-axis
            return np.array (
                [
                    [np.cos(angle), 0, np.sin(angle), 0],
                    [0, 1, 0, 0],
                    [-np.sin(angle), 0, np.cos(angle), 0],
                    [0, 0, 0, 1],
                ]
            )

        def rotation_z(angle):
            #rotation about the z-axis
            return np.array (
                [
                    [np.cos(angle), -np.sin(angle), 0, 0],
                    [np.sin(angle), np.cos(angle), 0, 0],
                    [0, 0, 1, 0],
                    [0, 0, 0, 1],
                ]
            )

        def translation(x, y, z):
            #the translation matrix
            return np.array (
                [
                    [1, 0, 0, x],
                    [0, 1, 0, y],
                    [0, 0, 1, z],
                    [0, 0, 0, 1],
                ]
            )

        # Modificat: Semne corecte pentru oglindirea piciorului stânga-spate
        T_0_1 = translation(-0.07500, 0.0445, 0) @ rotation_x(-1.57080) @ rotation_z(theta1) # 1.57080 = pi/2

        # T_1_2 (leg_front_l_1 to leg_front_l_2)
        ## TODO: Implement the transformation matrix from leg_front_l_1 to leg_front_l_2
        T_1_2 = translation(0, 0, 0.0390) @ rotation_y(-1.57080) @ rotation_z(theta2)

        # T_2_3 (leg_front_l_2 to leg_front_l_3)
        ## TODO: Implement the transformation matrix from leg_front_l_2 to leg_front_l_3
        T_2_3 = translation(0, 0.0494, 0.0685) @ rotation_y(1.57080) @ rotation_z(theta3)

        # T_3_ee (leg_front_l_3 to end-effector)
        T_3_ee = translation(0.06231, 0.06216, 0.0180)

        # TODO: Compute the final transformation. T_0_ee is the multiplication of the previous transformation matrices
        T_0_ee = T_0_1 @ T_1_2 @ T_2_3 @ T_3_ee

        # TODO: Extract the end-effector position. The end effector position is a 3x1 vector (not in homogenous coordinates)
        end_effector_position = T_0_ee[0:3, 3]

        return end_effector_position

    def inverse_kinematics_front_right(self, target_ee, initial_guess=[0, 0, 0]):
        def cost_function(theta):
            # Compute the cost function and the squared L2 norm of the error
            # return the cost and the squared L2 norm of the error
            ################################################################################################
            # TODO: Implement the cost function
            # HINT: You can use the * notation on a list to "unpack" a list
            ################################################################################################
            aux_ee = self.forward_kinematics_front_right(*theta)
            error = target_ee - aux_ee
            cost = np.linalg.norm(error)**2
            return cost, error

        def gradient(theta, epsilon=1e-3):
            # Compute the gradient of the cost function using finite differences
            ################################################################################################
            # TODO: Implement the gradient computation
            ################################################################################################
            grad = np.zeros_like(theta)
            for i in range(len(theta)):
                theta_plus = np.copy(theta)
                theta_minus = np.copy(theta)
                theta_plus[i] += epsilon
                theta_minus[i] -= epsilon
                cost_plus, _ = cost_function(theta_plus)
                cost_minus, _ = cost_function(theta_minus)
                grad[i] = (cost_plus - cost_minus) / (2 * epsilon)
            return grad

        theta = np.array(initial_guess, dtype=float)
        learning_rate = 5.0 # TODO: Set the learning rate
        max_iterations = 80 # TODO: Set the maximum number of iterations
        tolerance = 1e-4 # TODO: Set the tolerance for the L1 norm of the error

        cost_l = []
        for _ in range(max_iterations):
            cost, err = cost_function(theta)
            cost_l.append(cost)
            if cost < tolerance:
                break

            grad = gradient(theta)

            # Update the theta (parameters) using the gradient and the learning rate
            ################################################################################################
            # TODO: Implement the gradient update. Use the cost function you implemented, and use tolerance t
            # to determine if IK has converged
            # TODO (BONUS): Implement the (quasi-)Newton's method instead of finite differences for faster convergence
            ################################################################################################
            theta = theta - learning_rate * grad

        # print(f'Cost: {cost_l}') # Use to debug to see if you cost function converges within max_iterations

        return theta

    def inverse_kinematics_front_left(self, target_ee, initial_guess=[0, 0, 0]):
        def cost_function(theta):
            aux_ee = self.forward_kinematics_front_left(*theta)
            error = target_ee - aux_ee
            cost = np.linalg.norm(error)**2
            return cost, error

        def gradient(theta, epsilon=1e-3):
            grad = np.zeros_like(theta)
            for i in range(len(theta)):
                theta_plus = np.copy(theta)
                theta_minus = np.copy(theta)
                theta_plus[i] += epsilon
                theta_minus[i] -= epsilon
                cost_plus, _ = cost_function(theta_plus)
                cost_minus, _ = cost_function(theta_minus)
                grad[i] = (cost_plus - cost_minus) / (2 * epsilon)
            return grad

        theta = np.array(initial_guess, dtype=float)
        learning_rate = 5.0
        max_iterations = 80
        tolerance = 1e-4

        for _ in range(max_iterations):
            cost, err = cost_function(theta)
            if cost < tolerance:
                break
            grad = gradient(theta)
            theta = theta - learning_rate * grad
        return theta

    def inverse_kinematics_back_right(self, target_ee, initial_guess=[0, 0, 0]):
        def cost_function(theta):
            aux_ee = self.forward_kinematics_back_right(*theta)
            error = target_ee - aux_ee
            cost = np.linalg.norm(error)**2
            return cost, error

        def gradient(theta, epsilon=1e-3):
            grad = np.zeros_like(theta)
            for i in range(len(theta)):
                theta_plus = np.copy(theta)
                theta_minus = np.copy(theta)
                theta_plus[i] += epsilon
                theta_minus[i] -= epsilon
                cost_plus, _ = cost_function(theta_plus)
                cost_minus, _ = cost_function(theta_minus)
                grad[i] = (cost_plus - cost_minus) / (2 * epsilon)
            return grad

        theta = np.array(initial_guess, dtype=float)
        learning_rate = 5.0
        max_iterations = 80
        tolerance = 1e-4

        for _ in range(max_iterations):
            cost, err = cost_function(theta)
            if cost < tolerance:
                break
            grad = gradient(theta)
            theta = theta - learning_rate * grad
        return theta
    
    def inverse_kinematics_back_left(self, target_ee, initial_guess=[0, 0, 0]):
        def cost_function(theta):
            # Compute the cost function and the squared L2 norm of the error
            # return the cost and the squared L2 norm of the error
            ################################################################################################
            # TODO: Implement the cost function
            # HINT: You can use the * notation on a list to "unpack" a list
            ################################################################################################
            aux_ee = self.forward_kinematics_back_left(*theta)
            error = target_ee - aux_ee
            cost = np.linalg.norm(error)**2
            return cost, error

        def gradient(theta, epsilon=1e-3):
            # Compute the gradient of the cost function using finite differences
            ################################################################################################
            # TODO: Implement the gradient computation
            ################################################################################################
            grad = np.zeros_like(theta)
            for i in range(len(theta)):
                theta_plus = np.copy(theta)
                theta_minus = np.copy(theta)
                theta_plus[i] += epsilon
                theta_minus[i] -= epsilon
                cost_plus, _ = cost_function(theta_plus)
                cost_minus, _ = cost_function(theta_minus)
                grad[i] = (cost_plus - cost_minus) / (2 * epsilon)
            return grad

        theta = np.array(initial_guess, dtype=float)
        learning_rate = 5.0 # TODO: Set the learning rate
        max_iterations = 80 # TODO: Set the maximum number of iterations
        tolerance = 1e-4 # TODO: Set the tolerance for the L1 norm of the error

        for _ in range(max_iterations):
            cost, err = cost_function(theta)
            if cost < tolerance:
                break

            grad = gradient(theta)

            # Update the theta (parameters) using the gradient and the learning rate
            ################################################################################################
            # TODO: Implement the gradient update. Use the cost function you implemented, and use tolerance t
            # to determine if IK has converged
            # TODO (BONUS): Implement the (quasi-)Newton's method instead of finite differences for faster convergence
            ################################################################################################
            theta = theta - learning_rate * grad

        # print(f'Cost: {cost_l}') # Use to debug to see if you cost function converges within max_iterations

        return theta


    def interpolate_triangle(self, t, target_triangle):
        # Intepolate between the three triangle positions in the self.ee_triangle_positions
        # based on the current time t
        ################################################################################################
        # TODO: Implement the interpolation function
        ################################################################################################
        phase = t % 3.0
        idx = int(phase)
        next_idx = (idx + 1) % 3
        local_t = phase - idx
        return target_triangle[idx] + (target_triangle[next_idx] - target_triangle[idx]) * local_t

    def ik_timer_callback(self):
        if self.joint_positions is not None:
            # step direction
            dir_right = -1.0 if self.current_angular_z < -0.1 else 1.0
            dir_left = -1.0 if self.current_angular_z > 0.1 else 1.0
            
            tri_fr = self.base_triangle * [dir_right, 1, 1] + self.center_fr
            tri_fl = self.base_triangle * [dir_left, 1, 1] + self.center_fl
            tri_br = self.base_triangle * [dir_right, 1, 1] + self.center_br
            tri_bl = self.base_triangle * [dir_left, 1, 1] + self.center_bl

            target_ee_fr = self.interpolate_triangle(self.t, tri_fr)
            target_ee_bl = self.interpolate_triangle(self.t, tri_bl)
            
            target_ee_fl = self.interpolate_triangle((self.t + 1.5) % 3.0, tri_fl)
            target_ee_br = self.interpolate_triangle((self.t + 1.5) % 3.0, tri_br)

            current_joints_fr = self.joint_positions[0:3]
            current_joints_fl = self.joint_positions[3:6]
            current_joints_br = self.joint_positions[6:9]
            current_joints_bl = self.joint_positions[9:12]

            target_joints_fr = self.inverse_kinematics_front_right(target_ee_fr, current_joints_fr)
            target_joints_fl = self.inverse_kinematics_front_left(target_ee_fl, current_joints_fl)
            target_joints_br = self.inverse_kinematics_back_right(target_ee_br, current_joints_br)
            target_joints_bl = self.inverse_kinematics_back_left(target_ee_bl, current_joints_bl)

            self.target_joint_positions[0:3] = target_joints_fr.tolist()
            self.target_joint_positions[3:6] = target_joints_fl.tolist()
            self.target_joint_positions[6:9] = target_joints_br.tolist()
            self.target_joint_positions[9:12] = target_joints_bl.tolist()

            # if it receives a command to move forward
            if self.current_linear_x > 0.05:
                self.t += self.ik_timer_period * 2.5
            # if it receives a command to turn left or right
            elif abs(self.current_angular_z) > 0.1:
                self.t += self.ik_timer_period * 2.5
            
            self.get_logger().info(f'Robotul merge! FR: {target_ee_fr[0]:.2f}, FL: {target_ee_fl[0]:.2f}')

    def pd_timer_callback(self):
        if self.target_joint_positions is not None:
            command_msg = Float64MultiArray()
            command_msg.data = self.target_joint_positions
            self.command_publisher.publish(command_msg)

def main():
    rclpy.init()
    inverse_kinematics = InverseKinematics()
    
    try:
        rclpy.spin(inverse_kinematics)
    except KeyboardInterrupt:
        print("Program terminated by user")
    finally:
        # Send zero torques
        zero_torques = Float64MultiArray()
        zero_torques.data = [0.0] * 12
        inverse_kinematics.command_publisher.publish(zero_torques)
        
        inverse_kinematics.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()