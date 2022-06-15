#!/usr/bin/env python3

import os
import rclpy
from rclpy.node import Node
from ament_index_python import get_package_share_directory
from cv_bridge import CvBridge

import torch
import mmcv
import numpy as np
from mmdet.apis import inference_detector, init_detector

from sensor_msgs.msg import Image
from std_srvs.srv import SetBool
from mmdetection_msgs.msg import (
    Detection,
    Detections
)


class MmDetectionNode(Node):

    def __init__(self) -> None:
        super().__init__("mmdetection_node")

        # params
        self.declare_parameter(
            "config", os.path.join
            (get_package_share_directory("mmdetection_bringup"),
             "config",
             "yolact/yolact_r101_1x8_coco.py"))
        self.declare_parameter(
            "weights", os.path.join
            (get_package_share_directory("mmdetection_bringup"),
             "config",
             "yolact/yolact_r101_1x8_coco_20200908-4cbe9101.pth"))
        self.declare_parameter("device", "cuda:0")
        self.declare_parameter("threshold", 0.3)
        self.declare_parameter("enable", True)

        config = self.get_parameter(
            "config").get_parameter_value().string_value
        weights = self.get_parameter(
            "weights").get_parameter_value().string_value
        device = self.get_parameter(
            "device").get_parameter_value().string_value

        self.model = init_detector(config, weights, device)
        self.threshold = self.get_parameter(
            "threshold").get_parameter_value().double_value
        self.enable = self.get_parameter(
            "enable").get_parameter_value().bool_value

        self.cv_bridge = CvBridge()

        # topcis
        self._pub = self.create_publisher(
            Detections, "detections", 10)
        self._sub = self.create_subscription(
            Image, "image_raw", self.image_cb, 10)

        # services
        self._srv = self.create_service(SetBool, "enable", self.enable_cb)

    def enable_cb(self,
                  req: SetBool.Request,
                  res: SetBool.Response
                  ) -> SetBool.Response:
        self.enable = req.data
        res.success = True
        return res

    def image_cb(self, msg: Image) -> None:

        if self.enable:

            detections_msg = Detections()
            detections_msg.header.frame_id = msg.header.frame_id
            detections_msg.header.stamp = msg.header.stamp  # self.get_clock().now().to_msg()

            cv_image = self.cv_bridge.imgmsg_to_cv2(msg)
            result = inference_detector(self.model, cv_image)

            if isinstance(result, tuple):
                bbox_result, mask_result = result
                if isinstance(mask_result, tuple):
                    mask_result = mask_result[0]  # ms rcnn
            else:
                bbox_result, mask_result = result, None
            bboxes = np.vstack(bbox_result)

            labels = [
                np.full(bbox.shape[0], i, dtype=np.int32)
                for i, bbox in enumerate(bbox_result)
            ]
            labels = np.concatenate(labels)

            masks = None
            if mask_result is not None and len(labels) > 0:  # non empty
                masks = mmcv.concat_list(mask_result)
                if isinstance(masks[0], torch.Tensor):
                    masks = torch.stack(masks, dim=0).detach().cpu().numpy()
                else:
                    masks = np.stack(masks, axis=0)

            if not labels is None:

                detections = [labels]

                if not bboxes is None:
                    detections.append(bboxes)
                if not masks is None:
                    detections.append(masks)

                for detection in zip(*detections):

                    if detection[1][4] > self.threshold:
                        d_msg = Detection()
                        d_msg.label_id = int(detection[0])
                        d_msg.label = self.model.CLASSES[detection[0]]
                        d_msg.score = float(detection[1][4])

                        if not bboxes is None:
                            d_msg.box.center_x = float((
                                detection[1][0] + detection[1][2]) / 2)
                            d_msg.box.center_y = float((
                                detection[1][1] + detection[1][3]) / 2)
                            d_msg.box.size_x = float(
                                detection[1][2] - detection[1][0])
                            d_msg.box.size_y = float(
                                detection[1][3] - detection[1][1])

                        if not masks is None:
                            d_msg.mask.height = len(detection[2])
                            d_msg.mask.width = len(detection[2][0])

                            for row in detection[2]:
                                for ele in row:
                                    d_msg.mask.data.append(ele)

                        detections_msg.detections.append(d_msg)

            self._pub.publish(detections_msg)


def main():
    rclpy.init()
    node = MmDetectionNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
