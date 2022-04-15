#! /usr/bin/env python3

from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
import rospy

class RotracManager():

    def __init__(self):
        self.load_params()
        self.pub_cmd_vel_fl = rospy.Publisher('/rotrac_e2/front_left_wheel_velocity_controller/command',
                                            Float64,
                                            queue_size=1)
        self.pub_cmd_vel_rl = rospy.Publisher('/rotrac_e2/rear_left_wheel_velocity_controller/command',
                                            Float64,
                                            queue_size=1)
        self.pub_cmd_vel_fr = rospy.Publisher('/rotrac_e2/front_right_wheel_velocity_controller/command',
                                           Float64,
                                           queue_size=1)
        self.pub_cmd_vel_rr = rospy.Publisher('/rotrac_e2/rear_right_wheel_velocity_controller/command',
                                           Float64,
                                           queue_size=1)
        self.cmd_vel_sub = rospy.Subscriber('/cmd_vel', Twist, self.update_rotrac_velocity)

    def load_params(self):
        self.track_of_wheels = rospy.get_param("rotrac_manager/track_of_wheels")
        self.wheel_radius = rospy.get_param("rotrac_manager/wheel_radius")

    def update_rotrac_velocity(self, msg: Twist) -> None:
        self.calculate_wheels_veocity(msg.linear.x, msg.angular.z)
        self.publish_controller_commands()

    def calculate_wheels_veocity(self, V: float, w: float) -> None:
        self.Vl = (2*V - w*self.track_of_wheels) / (2*self.wheel_radius)                                    
        self.Vr = (2*V + w*self.track_of_wheels) / (2*self.wheel_radius)   

    def publish_controller_commands(self):
        self.pub_cmd_vel_fl.publish(self.Vl)
        self.pub_cmd_vel_rl.publish(self.Vl)
        self.pub_cmd_vel_fr.publish(self.Vr)
        self.pub_cmd_vel_rr.publish(self.Vr)    

if __name__=="__main__":
    rospy.init_node('rotrac_manager')
    try:
        rotrac_manager = RotracManager()
    except rospy.ROSInterruptException:
        pass
    rospy.spin()