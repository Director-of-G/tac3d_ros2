#!/usr/bin/python
# This node 

import rclpy
from rclpy.node import Node
import os
import numpy as np
import yaml
from ament_index_python.packages import get_package_share_directory
from std_msgs.msg import String
from geometry_msgs.msg import Point, Wrench, WrenchStamped
from tac3d_ros2.PyTac3D import Sensor as Tac3DSensor
from common_msgs.msg import ContactState

class Tac3DReaderNode(Node):
    def __init__(self):
        super().__init__('single_tac3d_reader_node')

        # declare
        self.declare_parameter('geom', 'default_model::default_collision')  # collision geometry name in SDF
        self.declare_parameter('frame', 'map')                              # frame_id in URDF
        self.declare_parameter('serial', 'AD2-0040R')                       # sensor serial number
        self.declare_parameter('index', 0)                                  # sensor index
        self.declare_parameter('port', 9988)                                # sensor UDP port

        sensor_frame = self.get_parameter('frame').get_parameter_value().string_value
        sensor_serial = self.get_parameter('serial').get_parameter_value().string_value
        sensor_index = self.get_parameter('index').get_parameter_value().integer_value
        udp_port = self.get_parameter('port').get_parameter_value().integer_value

        self.sensor_frame = sensor_frame

        # create sensor
        sensor_ = Tac3DSensor(
            recvCallback=self.sensor_state_callback,
            port=udp_port,
            maxQSize=5,
            callbackParam=sensor_index
        )

        force_topic_name = f'tac3d_contact_force_{sensor_index}'
        self.sensor_state_pub = self.create_publisher(
            WrenchStamped,
            force_topic_name,
            10
        )

        self.get_logger().info(f'Start reading from Tac3D sensor {sensor_serial} on port {udp_port}, \
                               the data will be published on {force_topic_name} in frame {sensor_frame}!')

        # timer
        # self.timer = self.create_timer(1/30, self.timer_callback)

    def sensor_state_callback(self, frame, param):
        Fr = frame.get('3D_ResultantForce')
        Mr = frame.get('3D_ResultantMoment')

        sensed_force = np.asarray(Fr.copy()).flatten()
        sensed_wrench = np.asarray(Mr.copy()).flatten()

        msg = WrenchStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.sensor_frame
        msg.wrench.force.x = sensed_force[0]
        msg.wrench.force.y = sensed_force[1]
        msg.wrench.force.z = sensed_force[2]

        self.sensor_state_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    tac3d_reader = Tac3DReaderNode()
    rclpy.spin(tac3d_reader)

    tac3d_reader.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()