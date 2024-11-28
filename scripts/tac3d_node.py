#!/usr/bin/python

import rclpy
from rclpy.node import Node
import os
import numpy as np
import yaml
from ament_index_python.packages import get_package_share_directory
from std_msgs.msg import String
from geometry_msgs.msg import Point, Wrench
from tac3d_ros2.PyTac3D import Sensor as Tac3DSensor
from common_msgs.msg import ContactState

class Tac3DReaderNode(Node):
    def __init__(self):
        super().__init__('tac3d_reader_node')

        # load config
        self.config = yaml.safe_load(
            open(os.path.join(get_package_share_directory('tac3d_ros2'), 'config', 'tac3d_config.yaml'), 'r')
        )['tac3d_leaphand']

        n_c = len(self.config)
        self.resultant_wrench_buf = np.zeros((n_c, 6))
        self.n_c = n_c

        # create sensor
        contact_geom_names = []
        sensor_list = []
        for idx, finger in enumerate(self.config):
            sensor_ = Tac3DSensor(
                recvCallback=self.sensor_state_callback_deprecated,
                port=self.config[finger]['port'],
                maxQSize=5,
                callbackParam=idx
            )
            contact_geom_names.append(self.config[finger]['geom'])
            sensor_list.append(sensor_)
        self.contact_geom_names = contact_geom_names
        self.sensor_list = sensor_list

        # publisher
        self.sensor_state_pub = self.create_publisher(
            ContactState,
            '/contact_sensor_state',
            10
        )

        # TODO(yongpeng): for debug
        self.fake_sensor_state_pub = self.create_publisher(
            String, '/fake_sensor_state', 10
        )
        self.fake_sensor_state_pub_1 = self.create_publisher(
            String, '/fake_sensor_state_1', 10
        )
        self.fake_sensor_state_pub_2 = self.create_publisher(
            String, '/fake_sensor_state_2', 10
        )
        self.fake_sensor_state_pub_3 = self.create_publisher(
            String, '/fake_sensor_state_3', 10
        )

        # timer
        self.timer = self.create_timer(1/30, self.timer_callback)

    def timer_callback(self):
        msg = ContactState()
        msg.names = self.contact_geom_names
        for i in range(self.n_c):
            point_ = Point()
            msg.points.append(point_)
            wrench_ = Wrench()
            wrench_.force.x = self.resultant_wrench_buf[i, 0]
            wrench_.force.y = self.resultant_wrench_buf[i, 1]
            wrench_.force.z = self.resultant_wrench_buf[i, 2]
            msg.wrenches.append(wrench_)
        self.sensor_state_pub.publish(msg)

    def sensor_state_callback(self, frame, param):
        index = param
        Fr = frame.get('3D_ResultantForce')
        Mr = frame.get('3D_ResultantMoment')
        self.resultant_wrench_buf[index, 0:3] = Fr.copy()
        self.resultant_wrench_buf[index, 3:6] = Mr.copy()

    def sensor_state_callback_deprecated(self, frame, param):
        # SN = frame['SN']
        # print(SN)

        # frameIndex = frame['index']
        # print(frameIndex)

        # sendTimestamp = frame['sendTimestamp']
        # recvTimestamp = frame['recvTimestamp']

        # print("send time: ", sendTimestamp)
        # print("receive time: ", recvTimestamp)

        # P = frame.get('3D_Positions')
        # D = frame.get('3D_Displacements')
        # F = frame.get('3D_Forces')
        # Fr = frame.get('3D_ResultantForce')
        # Mr = frame.get('3D_ResultantMoment')

        # print("position size: ", P.shape)
        # print("displacement size: ", D.shape)
        # print("force size: ", F.shape)
        # print("resultant force size: ", Fr.shape)
        # print("resultant moment size: ", Mr.shape)

        # msg = String()
        # msg.data = "receive one frame"
        # self.sensor_state_pub.publish(msg)

        index = param
        if index == 0:
            self.fake_sensor_state_pub.publish(String(data="sensor1 callback"))
        elif index == 1:
            self.fake_sensor_state_pub_1.publish(String(data="sensor2 callback"))
        elif index == 2:
            self.fake_sensor_state_pub_2.publish(String(data="sensor3 callback"))
        elif index == 3:
            self.fake_sensor_state_pub_3.publish(String(data="sensor4 callback"))
        else:
            raise ValueError("index out of range")

def main(args=None):
    rclpy.init(args=args)
    tac3d_reader = Tac3DReaderNode()
    rclpy.spin(tac3d_reader)

    tac3d_reader.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()