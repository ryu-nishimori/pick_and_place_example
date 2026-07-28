#!/usr/bin/env python3
# Copyright (c) 2025 TOYOTA MOTOR CORPORATION
# All rights reserved.
# Redistribution and use in source and binary forms, with or without
# modification, are permitted (subject to the limitations in the disclaimer
# below) provided that the following conditions are met:
# * Redistributions of source code must retain the above copyright notice, this
#   list of conditions and the following disclaimer.
# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
# * Neither the name of the copyright holder nor the names of its contributors may be used
#   to endorse or promote products derived from this software without specific
#   prior written permission.
# NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
# LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
# THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
# GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
# HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
# OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
# DAMAGE.
import json
from pathlib import Path
from typing import List

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from instance_segmentation_msgs.msg import InstanceSegmentation, BBox
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo, CompressedImage
from yolox_bridge.coco_mapper import COCOClassMapper

from vision_msgs.msg import Detection2DArray

def img_and_bbox_to_segment(img: Image, bbox: list[BBox]) -> Image:
    bridge = CvBridge()

    # Image -> NumPy
    cv_img = bridge.imgmsg_to_cv2(img, desired_encoding=img.encoding)

    shape = cv_img.shape[:2]
    segment_array = np.full(shape, 0, dtype=np.uint8)

    for i, box in enumerate(bbox):
        x, y, w, h = box.x, box.y, box.w, box.h
        segment_array[y:y + h, x:x + w] = i + 1

    print(f'[bridge] img_and_bbox_to_segment: ENC={img.encoding}, bbox_len={len(bbox)}')
    segment_msg = bridge.cv2_to_imgmsg(segment_array, encoding='8UC1')
    print("[bridge] img_and_bbox_to_segment: FILL!")

    segment_msg.header = img.header
    return segment_msg


class YoloxBridgeNode(Node):
    def __init__(self):
        super().__init__('yolox_bridge_node')

        # Parameters
        self.declare_parameter('output_topic', '/yolox_bridge/result')
        output_topic = self.get_parameter('output_topic').get_parameter_value().string_value

        self.declare_parameter('blacklist_path', '')
        blacklist_path = self.get_parameter('blacklist_path').get_parameter_value().string_value

        self.declare_parameter('depth_topic', '/head_rgbd_sensor/depth_registered/image_rect_raw/compressedDepth')
        depth_topic = self.get_parameter('depth_topic').get_parameter_value().string_value
        self.get_logger().info(f"[bridge] Using depth_topic: {depth_topic}")

        # blacklist
        self.blacklist: List[str] = []
        if blacklist_path:
            try:
                with open(blacklist_path, 'r') as f:
                    self.blacklist = json.load(f)
            except (FileNotFoundError, json.JSONDecodeError) as e:
                self.get_logger().error(f"[bridge] Failed to load blacklist file: {e}")

        self.coco_class_mapper = COCOClassMapper()

        # latest messages
        self.camera_info: CameraInfo | None = None
        self.rgb_image: Image | None = None
        self.depth_image: CompressedImage | None = None

        self.bridge = CvBridge()

        # Subscribers
        self.create_subscription(
            CameraInfo,
            '/head_rgbd_sensor/rgb/camera_info',
            self.camera_info_callback,
            10
        )
        self.create_subscription(
            Image,
            '/head_rgbd_sensor/rgb/image_rect_color',
            self.rgb_callback,
            10
        )
        self.create_subscription(
            CompressedImage,
            depth_topic,
            self.depth_callback,
            10
        )
        self.create_subscription(
            Detection2DArray,
            '/yolox/bounding_boxes',
            self.detection_callback,
            10
        )

        # Publishers
        self.pub = self.create_publisher(
            InstanceSegmentation,
            output_topic,
            qos_profile_sensor_data
        )
        self.tmp_pub_result_rgb = self.create_publisher(
            CompressedImage,
            '~/result_rgb',
            10
        )
        self.tmp_pub_result_depth = self.create_publisher(
            CompressedImage,
            '~/result_depth',
            10
        )
        self.tmp_pub_result_segment = self.create_publisher(
            Image,
            '~/result_segment',
            10
        )

    # ========= Callbacks =========

    def camera_info_callback(self, msg: CameraInfo):
        self.camera_info = msg

    def rgb_callback(self, msg: Image):
        self.rgb_image = msg

    def depth_callback(self, msg: CompressedImage):
        self.depth_image = msg

    def depth_to_compresseddepth2(self, depth_msg):
        try:
            depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='32FC1')

            depth_image = np.nan_to_num(
                depth_image,
                nan=10000.0,
                posinf=10000.0,
                neginf=10000.0
            )

            depth_16u = (depth_image * 1000).clip(None, 65535).astype(np.uint16)

            compressed_msg = self.bridge.cv2_to_compressed_imgmsg(depth_16u, dst_format='png')

            header_12_bytes = np.array(
                [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                dtype=np.uint8
            )
            final_data = np.concatenate((header_12_bytes, np.array(compressed_msg.data))).tolist()

            final_msg = CompressedImage()
            final_msg.header.stamp = depth_msg.header.stamp
            final_msg.header.frame_id = depth_msg.header.frame_id
            final_msg.format = "16UC1; compressedDepth"
            final_msg.data = final_data

            return final_msg
        except Exception as e:
            return None

    def depth_to_compresseddepth(self, depth_msg):
        try:
            depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='32FC1')

            depth_image = np.nan_to_num(
                depth_image,
                nan=10000.0,
                posinf=10000.0,
                neginf=10000.0
            )

            depth_16u = (depth_image * 1000).clip(None, 65535).astype(np.uint16)

            header_data = bytearray()

            height, width = depth_16u.shape
            quantization = 1000  # mm

            header_data.extend(height.to_bytes(4, byteorder='little'))
            header_data.extend(width.to_bytes(4, byteorder='little'))
            header_data.extend(quantization.to_bytes(4, byteorder='little'))

            success, png_data = cv2.imencode(
                '.png',
                depth_16u,
                [cv2.IMWRITE_PNG_COMPRESSION, 9]
            )

            if not success:
                raise Exception("Failed to encode depth image as PNG")

            compressed_msg = CompressedImage()
            compressed_msg.header = depth_msg.header
            compressed_msg.format = "16UC1;compressedDepth"
            compressed_msg.data = bytes(header_data) + png_data.tobytes()

            return compressed_msg
        except Exception as e:
            self.get_logger().error(f'[bridge] Error in depth_to_compresseddepth: {str(e)}')
            return None

    def detection_callback(self, det_msg: Detection2DArray):
        if self.camera_info is None or self.rgb_image is None or self.depth_image is None:
            self.get_logger().warn('[bridge] Awaiting camera_info, rgb_image, or depth_image...')
            return

        inst_msg = InstanceSegmentation()
        inst_msg.header = det_msg.header
        inst_msg.camera_info = self.camera_info

        bboxes: list[BBox] = []
        for det in det_msg.detections:
            if not det.results:
                continue

            hyp = det.results[0]

            try:
                class_id = hyp.hypothesis.class_id  # string
                score = float(hyp.hypothesis.score)
            except AttributeError as e:
                self.get_logger().error(f"[bridge] Unexpected hypothesis structure: {e}")
                continue

            if self.blacklist and class_id in self.blacklist:
                self.get_logger().info(f"[bridge] class_id '{class_id}' is in blacklist, skip")
                continue

            bbox2d = det.bbox
            cx = bbox2d.center.position.x
            cy = bbox2d.center.position.y
            w = bbox2d.size_x
            h = bbox2d.size_y

            x = int(cx - w / 2.0)
            y = int(cy - h / 2.0)

            b = BBox()

            try:
                coco_id = int(class_id)
            except ValueError:
                try:
                    coco_id = self.coco_class_mapper[class_id]
                except Exception as e:
                    coco_id = 0

            b.id = coco_id
            b.name = class_id
            b.score = score
            b.x = x
            b.y = y
            b.w = int(w)
            b.h = int(h)
            b.track_id = b.id

            bboxes.append(b)

        inst_msg.is_detected = len(bboxes) > 0

        if len(bboxes) == 0:
            return

        # RGB → JPEG
        try:
            cv_rgb = self.bridge.imgmsg_to_cv2(self.rgb_image, desired_encoding='bgr8')
            comp_rgb = self.bridge.cv2_to_compressed_imgmsg(cv_rgb, dst_format='jpg')
        except Exception as e:
            self.get_logger().error(f"[bridge] RGB conversion failed: {e}")
            return

        comp_rgb.header = self.rgb_image.header
        inst_msg.rgb = comp_rgb

        inst_msg.depth = self.depth_image
        inst_msg.depth.format = '16UC1; compressedDepth'

        inst_msg.bbox.clear()
        for b in bboxes:
            inst_msg.bbox.append(b)
            break

        # segment
        try:
            segment_msg = img_and_bbox_to_segment(self.rgb_image, inst_msg.bbox)
        except Exception as e:
            self.get_logger().error(f"[bridge] img_and_bbox_to_segment failed: {e}")
            return

        inst_msg.segment = segment_msg

        # publish
        self.pub.publish(inst_msg)

        # debug
        inst_msg_rgb = inst_msg.rgb
        inst_msg_rgb.format = 'jpeg'
        self.tmp_pub_result_rgb.publish(inst_msg_rgb)

        inst_msg_depth = inst_msg.depth
        inst_msg_depth.format = '16UC1; png'
        self.tmp_pub_result_depth.publish(inst_msg_depth)

        self.tmp_pub_result_segment.publish(inst_msg.segment)


def main(args=None):
    rclpy.init(args=args)
    node = YoloxBridgeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
