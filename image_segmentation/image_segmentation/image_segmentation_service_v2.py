import sys
from custom_msg_srv.srv import ImageBatch
from example_interfaces.srv import AddTwoInts
import rclpy
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


class ImageSegmentationService(Node):
    def __init__(self):
        super().__init__("image_segmentation_service")
        # define a client to send request for the overhead images
        self.overhead_client = self.create_client(ImageBatch, 'get_images')
        # gives service to the global planner.
        self.global_planer_service = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

        while not self.overhead_client.wait_for_service(timeout_sec=1):
            self.get_logger().info('overhead cameras service not available, trying again ...')

        self.waiting_for_overhead_service = False
        self.future = None

        self.cv_overhead_images = None
        self.overhead_camera_num = None
        self.show_images = True
        self.predictor = Predict()

    def send_request(self):
        """
        sending a request to overhead_cameras_service for overhead_images
        :return:
        """
        self.future = self.overhead_client.call_async(ImageBatch.Request())
        self.waiting_for_overhead_service = True

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))
        self.send_request()
        return response

    def image_proc(self, overhead_images):
        cv_images = []
        for image in overhead_images:
            cv_images.append(CvBridge().imgmsg_to_cv2(image, desired_encoding='bgr8'))

        self.overhead_camera_num = len(cv_images)
        segmented_images = []
        predicted_images = self.predictor.predict(*cv_images)
        for i in range(self.overhead_camera_num):
            segmented_images.append(predicted_images[i, :, :].astype(np.float))

        if self.show_images:
            for i, cv_image in enumerate(cv_images):
                cv2.imshow("image" + str(i + 1), cv_image)
            cv2.waitKey(1)

            for i, segmented_image in enumerate(segmented_images):
                cv2.imshow("segmented image" + str(i + 1), segmented_image)
            cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)

    minimal_client = ImageSegmentationService()

    while rclpy.ok():
        rclpy.spin_once(minimal_client)
        if minimal_client.waiting_for_overhead_service:
            if minimal_client.future.done():
                try:
                    response = minimal_client.future.result()
                    minimal_client.image_proc(response.image)
                    minimal_client.get_logger().info("succeed")
                    minimal_client.waiting_for_overhead_service = False
                except Exception as e:
                    minimal_client.get_logger().info("service call failed")

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
