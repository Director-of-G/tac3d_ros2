#!/usr/bin/python
# This node 

import threading
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
import matplotlib.animation as anim
from matplotlib import pyplot as plt
import numpy as np
import open3d as o3d
from geometry_msgs.msg import WrenchStamped
from tac3d_ros2.PyTac3D import Sensor as Tac3DSensor

class Tac3DReaderNode(Node):
    def __init__(self):
        super().__init__('single_tac3d_reader_node')

        # declare
        self.declare_parameter('geom', 'default_model::default_collision')  # collision geometry name in SDF
        self.declare_parameter('frame', 'map')                              # frame_id in URDF
        self.declare_parameter('serial', 'AD2-0040R')                       # sensor serial number
        self.declare_parameter('index', 0)                                  # sensor index
        self.declare_parameter('port', 9988)                                # sensor UDP port
        self.declare_parameter('debug', False)                              # debug and visualize

        sensor_frame = self.get_parameter('frame').get_parameter_value().string_value
        sensor_serial = self.get_parameter('serial').get_parameter_value().string_value
        sensor_index = self.get_parameter('index').get_parameter_value().integer_value
        udp_port = self.get_parameter('port').get_parameter_value().integer_value
        debug = self.get_parameter('debug').get_parameter_value().bool_value

        self.debug = debug
        self.sensor_frame = sensor_frame
        self.contact_force_thres = 5e-5     # threshold on contact force
        self.contact_disp_thres = 1e-4      # threshold on contact displacement
        self.filtered_normal = None
        self.normal_lpf_alpha = 0.75

        # initialize matplotlib
        if debug:
            fig = plt.figure()
            ax = fig.add_subplot(111, projection='3d')
            ax.set_xlim([-0.02, 0.02])
            ax.set_ylim([-0.02, 0.02])
            ax.set_zlim([-0.02, 0.02])
            points, = ax.plot([], [], [], 'o')
            self.fig, self.ax = fig, ax
            self.plt_points = points
        self.o3d_pcd = o3d.geometry.PointCloud()

        # create sensor
        force_topic_name = f'tac3d_contact_force_{sensor_index}'
        self.sensor_state_pub = self.create_publisher(
            WrenchStamped,
            force_topic_name,
            10
        )
        sensor_ = Tac3DSensor(
            recvCallback=self.sensor_state_callback,
            port=udp_port,
            maxQSize=5,
            callbackParam=sensor_index
        )

        self.get_logger().info(f'Start reading from Tac3D sensor {sensor_serial} on port {udp_port}, \
                               the data will be published on {force_topic_name} in frame {sensor_frame}!')
        self.get_logger().info(f'Debug mode is {debug}!')

        # timer
        # self.timer = self.create_timer(1/30, self.timer_callback)

    def sensor_state_callback(self, frame, param):
        Fr = frame.get('3D_ResultantForce')
        # Mr = frame.get('3D_ResultantMoment')

        P = frame.get('3D_Positions')           # unit (mm)
        P = np.asarray(P) * 0.001
        D = frame.get('3D_Displacements')           # unit (mm)
        D = np.asarray(D) * 0.001
        D_norm = np.linalg.norm(D, axis=-1, keepdims=True)
        F = frame.get('3D_Forces')
        F_norm = np.linalg.norm(np.asarray(F), axis=-1, keepdims=True)
        mask_in_contact = np.bitwise_and(
            F_norm >= self.contact_force_thres,
            D_norm >= self.contact_disp_thres
        ).flatten()
        pts_in_contact = (P+D)[mask_in_contact, :]
        if len(pts_in_contact) > 0:
            self.o3d_pcd.points = o3d.utility.Vector3dVector(pts_in_contact)
            self.o3d_pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamKNN(knn=10))
            surface_normal = np.asarray(self.o3d_pcd.normals)
            sensed_normal = np.mean(F_norm[mask_in_contact, :] * surface_normal, axis=0)
            sensed_normal = sensed_normal / (np.linalg.norm(sensed_normal) + 1e-5)
            if sensed_normal[2] < 0:
                sensed_normal = -sensed_normal
        else:
            sensed_normal = np.zeros(3,)
            self.filtered_normal = None

        if self.filtered_normal is None:
            self.filtered_normal = sensed_normal
        else:
            self.filtered_normal = self.normal_lpf_alpha * sensed_normal + (1 - self.normal_lpf_alpha) * self.filtered_normal

        sensed_force = np.asarray(Fr.copy()).flatten()
        # sensed_wrench = np.asarray(Mr.copy()).flatten()

        # TODO: we need the force applied to the environment
        sensed_force = -sensed_force

        msg = WrenchStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.sensor_frame
        msg.wrench.force.x = sensed_force[0]
        msg.wrench.force.y = sensed_force[1]
        msg.wrench.force.z = sensed_force[2]

        # use the torque field to store contact normal
        msg.wrench.torque.x = self.filtered_normal[0]
        msg.wrench.torque.y = self.filtered_normal[1]
        msg.wrench.torque.z = self.filtered_normal[2]

        self.sensor_state_pub.publish(msg)

    def plt_func(self, _):
        if self.o3d_pcd.is_empty():
            return
        points = np.asarray(self.o3d_pcd.points)
        self.plt_points.set_data(np.array([points[:, 0], points[:, 1]]))
        self.plt_points.set_3d_properties(points[:, 2], 'z')
        plt.pause(0.01)

        return self.plt_points,

    def start_animation(self):
        self.ani = anim.FuncAnimation(self.fig, self.plt_func, interval=300)
        plt.show()

def main(args=None):
    try:
        rclpy.init(args=args)
        tac3d_reader = Tac3DReaderNode()
        executor = MultiThreadedExecutor()
        executor.add_node(tac3d_reader)

        if tac3d_reader.debug:
            thread = threading.Thread(target=executor.spin, daemon=True)
            thread.start()
            tac3d_reader.start_animation()
        else:
            executor.spin()
    except:
        pass
    finally:
        executor.shutdown()
        if tac3d_reader.debug:
            thread.join()
            plt.close(tac3d_reader.fig)
        rclpy.shutdown()

if __name__ == '__main__':
    main()