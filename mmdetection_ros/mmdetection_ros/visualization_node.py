
import cv2
import random
import numpy as np

import rclpy
from rclpy.node import Node
import message_filters
from cv_bridge import CvBridge

from sensor_msgs.msg import Image
from mmdetection_msgs.msg import Detections


class VisualizationNode(Node):

    def __init__(self) -> None:
        super().__init__("visualization_node")

        # params
        self.declare_parameter("alpha", 0.4)
        self.declare_parameter("thickness", 2)
        self.alpha = self.get_parameter(
            "alpha").get_parameter_value().double_value
        self.thickness = self.get_parameter(
            "thickness").get_parameter_value().integer_value

        self._class_to_color = {}
        self.cv_bridge = CvBridge()

        self._pub = self.create_publisher(Image, "result_image", 10)

        image_sub = message_filters.Subscriber(self, Image, "image_raw")
        detections_sub = message_filters.Subscriber(
            self, Detections, "detections", )

        self._synchronizer = message_filters.ApproximateTimeSynchronizer(
            (image_sub, detections_sub), 10, 0.5)
        self._synchronizer.registerCallback(self.on_detections)

    def on_detections(self, image_msg: Image, detections_msg: Detections) -> None:

        cv_image = self.cv_bridge.imgmsg_to_cv2(image_msg)

        for detection in detections_msg.detections:

            if not detection.label in self._class_to_color:
                r = random.randint(0, 255)
                g = random.randint(0, 255)
                b = random.randint(0, 255)
                self._class_to_color[detection.label] = (r, g, b)
            color = self._class_to_color[detection.label]

            # box
            cx = detection.box.center_x
            cy = detection.box.center_y
            sx = detection.box.size_x
            sy = detection.box.size_y

            min_pt = (round(cx - sx / 2.0), round(cy - sy / 2.0))
            max_pt = (round(cx + sx / 2.0), round(cy + sy / 2.0))
            cv2.rectangle(cv_image, min_pt, max_pt, color, self.thickness)

            # mask
            mask = None
            if detection.mask.height > 0 and detection.mask.width > 0:
                mask = []
                for i in range(0, detection.mask.height * detection.mask.width, detection.mask.width):
                    aux = np.array(
                        detection.mask.data[i:i + detection.mask.width])
                    mask.append(aux)

                mask = np.array(mask)
                mask = mask.astype(bool)

                cv_image[mask] = cv_image[mask] * \
                    (1 - self.alpha) + np.array(color, dtype=np.uint8) * self.alpha

                contours, _ = cv2.findContours(
                    mask.astype(np.uint8), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
                cv2.drawContours(cv_image, contours, -1,
                                 (color[0]/2, color[1]/2, color[2]/2),
                                 thickness=self.thickness,
                                 lineType=cv2.LINE_AA)

            # label
            label = '{} {:.3f}'.format(detection.label, detection.score)
            pos = (min_pt[0] + 5, min_pt[1] + 20)

            if detection.score == 0 and not mask is None:  # panoptic
                label = '{}'.format(detection.label)
                print(mask.astype(np.uint8))
                _, _, stats, centroids = cv2.connectedComponentsWithStats(
                    mask.astype(np.uint8), connectivity=8)
                largest_id = np.argmax(stats[1:, -1]) + 1
                pos = (round(centroids[largest_id][0]),
                       round(centroids[largest_id][1]))

            font = cv2.FONT_HERSHEY_SIMPLEX
            cv2.putText(cv_image, label, pos, font,
                        0.75, color, 1, cv2.LINE_AA)

        detection_image_msg = self.cv_bridge.cv2_to_imgmsg(
            cv_image, encoding=image_msg.encoding)
        detection_image_msg.header = image_msg.header

        self._pub.publish(detection_image_msg)


def main():
    rclpy.init()
    node = VisualizationNode()
    rclpy.spin(node)
    rclpy.shutdown()
