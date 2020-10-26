import rospy
import numpy as np
import time
from src.State import BehaviorState, State
from src.Command import Command
from src.Utilities import deadband, clipped_first_order_filter

from geometry_msgs.msg import Twist
from std_msgs.msg import String, Empty
from joybro.msg import JoyBro


class JoystickInterface:
    def __init__(
        self, config, udp_port=8830, udp_publisher_port = 8840,
    ):
        self.config = config
        self.previous_gait_toggle = 0
        self.previous_state = BehaviorState.REST
        self.previous_hop_toggle = 0
        self.previous_activate_toggle = 0

        self.message_rate = 50

        self.twist = Twist()
        #rospy.init_node('pupper_cmd_vel', anonymous=True)
        rospy.Subscriber("/cmd_vel", Twist, self.twist_cb)
        rospy.Subscriber("/joybro", JoyBro, self.joy_callback)

        self.activate = 0
        self.joy_state = JoyBro()
        
    def activatecb(self, data):
        self.activate = not self.activate
        print(data)

    def twist_cb(self, data):
        self.twist = data
        
    def joy_callback(self, msg):
        self.joy_state = msg
        rospy.loginfo(msg.btn3)

    def get_command(self, state, do_print=False):
        try:
            #msg = self.udp_handle.get()
            command = Command()
            
            ####### Handle discrete commands ########
            # Check if requesting a state transition to trotting, or from trotting to resting
            gait_toggle = self.joy_state.btn1
            command.trot_event = (gait_toggle == 1 and self.previous_gait_toggle == 0)

            # Check if requesting a state transition to hopping, from trotting or resting
            hop_toggle = self.joy_state.btn2
            command.hop_event = (hop_toggle == 1 and self.previous_hop_toggle == 0)            
            
            activate_toggle = self.joy_state.btn3
            command.activate_event = (activate_toggle == 1 and self.previous_activate_toggle == 0)

            # Update previous values for toggles and state
            self.previous_gait_toggle = gait_toggle
            self.previous_hop_toggle = hop_toggle
            self.previous_activate_toggle = activate_toggle

            ####### Handle continuous commands ########
            x_vel = np.clip(self.twist.linear.x, -self.config.max_x_velocity, self.config.max_x_velocity)
            y_vel = np.clip(self.twist.linear.y, -self.config.max_y_velocity, self.config.max_y_velocity) #check for "-" if needed
            command.horizontal_velocity = np.array([x_vel, y_vel])
            command.yaw_rate = np.clip(self.twist.angular.z, -self.config.max_yaw_rate, self.config.max_yaw_rate)
            #message_rate = msg["message_rate"]
            message_dt = 1.0 / self.message_rate

            
            pitch = self.joy_state.right_x/512.0 * self.config.max_pitch
            deadbanded_pitch = deadband(
                pitch, self.config.pitch_deadband
            )
            pitch_rate = clipped_first_order_filter(
                state.pitch,
                deadbanded_pitch,
                self.config.max_pitch_rate,
                self.config.pitch_time_constant,
            )
            

            #command.pitch = state.pitch + message_dt * pitch_rate
            command.pitch = 0

            command.height = state.height
            command.roll = state.roll
            """
            height_movement = msg["dpady"]
            command.height = state.height - message_dt * self.config.z_speed * height_movement
            
            roll_movement = - msg["dpadx"]
            command.roll = state.roll + message_dt * self.config.roll_speed * roll_movement
            """
            rospy.loginfo(command.horizontal_velocity)
            return command

        except rospy.is_shutdown():
            print("ros is down")
            return Command()


    def set_color(self, color):
        """
        TODO rework to do smthing
        """
        pass