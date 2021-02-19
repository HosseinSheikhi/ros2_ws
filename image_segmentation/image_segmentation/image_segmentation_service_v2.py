import sys
from custom_msg_srv.srv import ImageBatch
from sensor_msgs.msg import Image
import rclpy
from rclpy.qos import qos_profile_system_default
from rclpy.node import Node
import cv2
from cv_bridge import CvBridge
from image_segmentation.predict import Predict
import numpy as np

"""
This class gets overhead images as a client and gives service to the global planner as follow:
1- global_planer_service receives a request for segmented images from global_planner
2- overhead_client sends a request for overhead images to the overhead_cameras_service and after receiving
3- feeds the overhead images to the segmentation_networks and returns the results to the global_planner 
"""

"""
TODO: send a request to overhead cameras must be based on a request from global planner not based on a timer
However, I have faced by deadlock errors when I was trying to develop this version 
"""


class ImageSegmentationService(Node):
    def __init__(self):
        super().__init__("image_segmentation_service")
        # define a client to send request for the overhead images
        self.overhead_client = self.create_client(ImageBatch, 'autonomous_robot/overhead_camera_service')
        # gives service to the global planner.
        self.global_planer_service = self.create_service(ImageBatch, 'autonomous_robot/get_segmented_images',
                                                         self.give_service_to_global_planner)

        # define stitched image publisher for UI
        self.stitched_image_publisher = self.create_publisher(Image, "autonomous_robot/image_segmentation/stitched_image", qos_profile_system_default )

        while not self.overhead_client.wait_for_service(timeout_sec=1):
            self.get_logger().info('overhead cameras service not available, trying again ...')

        self.timer = self.create_timer(2, self.send_request)
        self.waiting_for_overhead_service = False
        self.future = None

        self.overhead_images = None
        self.segmented_images = []
        self.stitched_image = None
        self.overhead_camera_num = None
        self.show_images = False
        self.predictor = Predict()

    def send_request(self):
        """
        sending a request to overhead_cameras_service for overhead_images
        :return: None
        """
        self.get_logger().info('sending request to overhead camera node ...')
        self.future = self.overhead_client.call_async(ImageBatch.Request())
        self.waiting_for_overhead_service = True

    def give_service_to_global_planner(self, request, response):
        self.get_logger().info('Incoming request from global planner')
        self.image_proc(self.overhead_images)

        self.image_stitching()
        if self.show_images:
            cv2.imshow("stitched image", self.stitched_image)
            cv2.waitKey(1)
        stitched_image_msg = CvBridge().cv2_to_imgmsg(self.stitched_image)
        response.image[0] = stitched_image_msg

        # publish for the ui
        self.stitched_image_publisher.publish(stitched_image_msg)
        return response

    def image_stitching(self):
        self.stitched_image = cv2.hconcat(self.segmented_images)

    def image_proc(self, overhead_images):
        self.get_logger().info('doing segmentation ...')

        cv_images = []
        for image in overhead_images:
            cv_images.append(CvBridge().imgmsg_to_cv2(image, desired_encoding='bgr8'))

        self.overhead_camera_num = len(cv_images)
        self.segmented_images.clear()
        predicted_images = self.predictor.predict(*cv_images)
        for i in range(self.overhead_camera_num):
            self.segmented_images.append(predicted_images[i, :, :].astype(np.float32) * 255.0)

        if self.show_images:
            for i, cv_image in enumerate(cv_images):
                cv2.imshow("image" + str(i + 1), cv_image)
            cv2.waitKey(1)

            for i, segmented_image in enumerate(self.segmented_images):
                cv2.imshow("segmented image" + str(i + 1), segmented_image)
            cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)

    image_segmentation_node = ImageSegmentationService()

    while rclpy.ok():
        rclpy.spin_once(image_segmentation_node)
        if image_segmentation_node.waiting_for_overhead_service:
            rclpy.spin_until_future_complete(image_segmentation_node, image_segmentation_node.future, timeout_sec=1)
        if image_segmentation_node.future.done():
            try:
                response = image_segmentation_node.future.result()
                image_segmentation_node.overhead_images = response.image
                image_segmentation_node.waiting_for_overhead_service = False
            except Exception as e:
                image_segmentation_node.get_logger().info("service call failed")

    image_segmentation_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
