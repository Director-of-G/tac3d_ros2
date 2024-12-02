#!/usr/bin/python
# This node 

import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
import message_filters
import os
import numpy as np
import yaml
from ament_index_python.packages import get_package_share_directory
from std_msgs.msg import String
from geometry_msgs.msg import Point, Wrench, WrenchStamped, TransformStamped
from tf2_geometry_msgs import PointStamped, Vector3Stamped
from tac3d_ros2.PyTac3D import Sensor as Tac3DSensor
from common_msgs.msg import ContactState


class ContactStatePublisherNode(Node):
    def __init__(self):
        super().__init__('contact_state_publisher_node')

        # declare
        self.declare_parameter('config_url', '')                # config file path

        config_url = self.get_parameter('config_url').get_parameter_value().string_value
        sensor_config = yaml.safe_load(open(config_url, 'r'))['tac3d']

        # other parameters
        self.base_frame = 'world'
        self.sensor_config = sensor_config
        self.contact_state_update_frequency = 30

        # buffers (dict: finger -> sth.)
        self.contact_wrench_buffer = {}

        # subscribers and publishers
        subscribers = []
        ats_topics = []
        ats_slop = 0.1
        for finger, config in self.sensor_config.items():
            sensor_index = len(subscribers)
            # sub = self.create_subscription(
            #     WrenchStamped,
            #     f'tac3d_contact_force_{sensor_index}',
            #     lambda msg: self.contact_force_callback(msg, finger),
            #     10
            # )
            sensor_topic = f'tac3d_contact_force_{sensor_index}'
            sub = message_filters.Subscriber(self, WrenchStamped, sensor_topic)
            subscribers.append(sub)
            ats_topics.append(sensor_topic)
        assert len(subscribers) == 4
        ts = message_filters.ApproximateTimeSynchronizer(subscribers, 20, slop=ats_slop)
        ts.registerCallback(self.contact_force_callback)
        self.get_logger().info(f'TimeSynchronizer monitors topics {ats_topics} with slop {ats_slop}')

        self.num_sensors = len(sensor_config)
        self.subscribers = subscribers

        self.contact_state_pub = self.create_publisher(
            ContactState,
            'contact_sensor_state',
            10
        )

        # update fingertip pose
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        import time
        time.sleep(1)

        # timers
        # self.contact_timer = self.create_timer(1 / self.contact_state_update_frequency, self.update_contact_state)

    def contact_force_callback(self, msg0:WrenchStamped, msg1:WrenchStamped, msg2:WrenchStamped, msg3: WrenchStamped):
        finger0 = list(self.sensor_config.keys())[0]
        finger1 = list(self.sensor_config.keys())[1]
        finger2 = list(self.sensor_config.keys())[2]
        finger3 = list(self.sensor_config.keys())[3]

        self.contact_wrench_buffer[finger0] = msg0
        self.contact_wrench_buffer[finger1] = msg1
        self.contact_wrench_buffer[finger2] = msg2
        self.contact_wrench_buffer[finger3] = msg3

        self.update_contact_state()

    def update_contact_state(self):
        # wait until all contact forces are collected
        if len(self.contact_wrench_buffer) < self.num_sensors:
            return
                
        msg = ContactState()
        msg.header.stamp = self.get_clock().now().to_msg()

        for finger, config in self.sensor_config.items():
            try:
                wrench_stamped = self.contact_wrench_buffer[finger]
                header = wrench_stamped.header
                wrench = wrench_stamped.wrench
                wrench_tf2 = Vector3Stamped()
                # wrench_tf2.header.stamp = header.stamp
                wrench_tf2.header.frame_id = header.frame_id
                wrench_tf2.vector = wrench.force
                wrench_tf2 = self.tf_buffer.transform(wrench_tf2, self.base_frame)
                wrench.force = wrench_tf2.vector
                msg.wrenches.append(wrench)

                point = PointStamped()
                # point.header.stamp = header.stamp
                point.header.frame_id = header.frame_id
                point = self.tf_buffer.transform(point, self.base_frame)
                msg.points.append(point.point)

                msg.names.append(config['geom'])
            except Exception as e:
                self.get_logger().warn(f'Failed to get transform: {e}')
                pass

        if len(msg.names) == self.num_sensors:
            self.contact_state_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    contact_state_publisher = ContactStatePublisherNode()
    rclpy.spin(contact_state_publisher)

    contact_state_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
