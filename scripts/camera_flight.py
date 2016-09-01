#! /usr/bin/env python
import rospy

from sensor_msgs.msg import CameraInfo
from thread import start_new_thread
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped
from tf.transformations import quaternion_from_euler
from math import pi


class CameraFlight:
    def __init__(self):
        self.pub_cam_info_ = rospy.Publisher("~flight_camera_info", CameraInfo, queue_size=1)
        self.pub_tf_ = rospy.Publisher("/tf", TFMessage, queue_size=1)

        self.fixed_frame_ = "world"
        self.camera_frame_ = "flightCam"
        self.cam_info_msg_ = self.create_cam_info(self.camera_frame_, 640, 480, 500)

        self._current_z = 1
        self._min_z = 0.8
        self._max_z = 1.5

        self._z_step = 0.02
        self._tf_message = self.create_transform()

        start_new_thread(self.publish_loop, ())

    @staticmethod
    def create_cam_info(frame_id, w, h, f):
        msg = CameraInfo()
        msg.header.frame_id = frame_id
        msg.width = w
        msg.height = h

        # msg.P = [0]*12
        msg.P[0] = msg.P[5] = f   # 0, 5: fx, fy
        msg.P[2] = msg.width / 2  # 2, 6: cx, cy
        msg.P[6] = msg.height / 2
        msg.P[11] = 1
        return msg

    def create_transform(self):
        msg = TransformStamped()
        msg.header.frame_id = self.fixed_frame_
        msg.child_frame_id = self.camera_frame_
        #print quaternion_from_euler(pi, 0, 0)
        msg.transform.rotation.x = 1  # pi rotation around x
        return msg

    def publish_next_transform(self):
        if self._current_z >= self._max_z:
            self._z_step *= -1
            self._current_z = self._max_z
        if self._current_z <= self._min_z:
            self._z_step *= -1
            self._current_z = self._min_z
        self._current_z += self._z_step

        self._tf_message.transform.translation.z = self._current_z
        self._tf_message.header.stamp = rospy.Time.now()
        tfm = TFMessage(transforms=[self._tf_message])
        self.pub_tf_.publish(tfm)

    def publish_loop(self):
        while not rospy.is_shutdown():
            rospy.sleep(0.1)
            self.cam_info_msg_.header.stamp = rospy.Time.now()
            self.pub_cam_info_.publish(self.cam_info_msg_)
            self.publish_next_transform()

if __name__ == "__main__":
    rospy.init_node("CameraFlight")
    cf = CameraFlight()

    rospy.spin()