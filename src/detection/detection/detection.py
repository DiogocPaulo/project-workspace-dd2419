#!/usr/bin/env python3

# Copyright 2021 Evan Flynn, Lucas Walter
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the Evan Flynn nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.


import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs_py.point_cloud2 as pc2

import ctypes
import struct

import tf2_ros

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import tf2_geometry_msgs
from tf2_geometry_msgs import do_transform_point
from geometry_msgs.msg import PointStamped


class ExamineImage(Node):

    def __init__(self):
        super().__init__('examine_image')

        self.mat = None

        self.get_logger().info(f"Init detection")

        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer,self)

        self.sub = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.image_callback,
            100)
        
        self.sub2 = self.create_subscription(
            PointCloud2,
            '/camera/camera/depth/color/points',
            self.cloud_callback,
            100)
        
        self.pub = self.create_publisher(PointCloud2,'camera/camera_depth/color/points_transformed',100)

    def image_callback(self, msg):
        sz = (msg.height, msg.width)
        if False:
            print('{encoding} {width} {height} {step} {data_size}'.format(
                encoding=msg.encoding, width=msg.width, height=msg.height,
                step=msg.step, data_size=len(msg.data)))
        if msg.step * msg.height != len(msg.data):
            print('bad step/height/data size')
            return

        if msg.encoding == 'rgb8':
            dirty = (self.mat is None or msg.width != self.mat.shape[1] or
                     msg.height != self.mat.shape[0] or len(self.mat.shape) < 2 or
                     self.mat.shape[2] != 3)
            if dirty:
                self.mat = np.zeros([msg.height, msg.width, 3], dtype=np.uint8)
            self.mat[:, :, 2] = np.array(msg.data[0::3]).reshape(sz)
            self.mat[:, :, 1] = np.array(msg.data[1::3]).reshape(sz)
            self.mat[:, :, 0] = np.array(msg.data[2::3]).reshape(sz)
        elif msg.encoding == 'mono8':
            self.mat = np.array(msg.data).reshape(sz)
        else:
            print('unsupported encoding {}'.format(msg.encoding))
            return
        if self.mat is not None:
            cv2.imshow('image', self.mat)
            cv2.waitKey(5)

        # self.get_logger().info(f"Detection message")

    def cloud_callback(self, msg: PointCloud2):
        # Convert ROS -> NumPy

        gen = pc2.read_points_numpy(msg, skip_nans=True)
        points = gen[:, :3]
        colors = np.empty(points.shape, dtype=np.uint32)

        for idx, x in enumerate(gen):
            c = x[3]
            s = struct.pack('>f', c)
            i = struct.unpack('>l', s)[0]
            pack = ctypes.c_uint32(i).value
            colors[idx, 0] = np.asarray((pack >> 16) & 255, dtype=np.uint8)
            colors[idx, 1] = np.asarray((pack >> 8) & 255, dtype=np.uint8)
            colors[idx, 2] = np.asarray(pack & 255, dtype=np.uint8)

        colors = colors.astype(np.float32) / 255

        max_dist = 0.9
        distance = np.linalg.norm(points,axis=1)
        mask = (distance < max_dist) & (points[:,2] >= 0)
        points = points[mask] 
        colors = colors[mask]

        r, g, b = colors[:,0], colors[:,1], colors[:,2]
        red_f = (r > 0.7) & (g < 0.4) & (b < 0.4)
        green_f = (r < 0.2) & (g > 0.5) & (b < 0.2)

        red_points = points[red_f]
        green_points = points[green_f]

        if len(red_points):
            self.get_logger().info(f"Red points detected")
        if len(green_points):
            self.get_logger().info(f"Green points detected")

        frame_id = msg.header.frame_id
        time_stamp = msg.header.stamp

        # tf_future = self.tfBuffer.wait_for_transform_async(
        #     target_frame = 'map',
        #     source_frame = frame_id,
        #     time = time_stamp
        # )

        # rclpy.spin_until_future_complete(self,tf_future, timeout_sec=1)

        # try:
        #     t = self.tfBuffer.lookup_transform(
        #         'map',
        #         frame_id,
        #         time_stamp)
        # except TransformException as ex:
        #     self.get_logger().info(
        #         f'Could not transform {frame_id} to map: {ex}'
        #     )

        # cloud_msg_red = PointCloud2()
        # cloud_msg_red.header.stamp = time_stamp
        # cloud_msg_red.header.frame_id = 'map'
        # cloud_msg_red.height = 1
        # cloud_msg_red.width = len(red_points)
        # cloud_msg_red.fields = [
        #     PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        #     PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        #     PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        #     PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1)
        # ]
        # cloud_msg_red.is_bigendian=False
        # cloud_msg_red.point_step = 16
        # cloud_msg_red.row_step = cloud_msg_red.point_step * len(red_points)
        # cloud_msg_red.is_dense = True


        # cloud_data = []
        # for p in red_points:
        #     point_stamped = PointStamped()
        #     point_stamped.header.frame_id = 'map'
        #     point_stamped.header.stamp = time_stamp
        #     point_stamped.point.x = p[0]
        #     point_stamped.point.y = p[1]
        #     point_stamped.point.z = p[2]
        #     transformed_point = do_transform_point(point_stamped,t)
        #     r,g,b = 255,0,0
        #     rgb = struct.unpack('I',struct.pack('BBBB',int(b),int(g),int(b),0))[0]
        #     cloud_data.append(struct.unpack('ffff',transformed_point.point.x,transformed_point.point.y,transformed_point.point.z,rgb))

        # cloud_msg_red.data = b''.join(cloud_data)

        # self.pub.publish(cloud_msg_red)



        # self.get_logger().info(f"Cloud message")





def main(args=None):
    rclpy.init(args=args)

    examine_image = ExamineImage()

    try:
        rclpy.spin(examine_image)
    except KeyboardInterrupt:
        pass

    examine_image.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
