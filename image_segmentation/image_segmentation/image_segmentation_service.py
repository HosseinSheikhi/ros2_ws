from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.qos import qos_profile_sensor_data
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import cv2
import message_filters
from image_segmentation.predict import Predict
import numpy as np
from custom_msg_srv.srv import ImageBatch
"""
This class subscribes to the overhead images and gives service to the global planner as follow:
1- service receives a request for segmented images from global_planner
3- feeds the overhead images to the segmentation_networks and returns the results to the global_planner 
"""


class ImageSegmentationService(Node):
    def __init__(self):
        super().__init__('image_segmentation_service')

        self.overhead_camera_num = 1
        self.show_images = False
        self.image_subscriber = [
            message_filters.Subscriber(self, Image, '/overhead_cam_' + str(i + 1) + '/camera/image_raw',
                                       qos_profile=qos_profile_sensor_data)
            for i in range(self.overhead_camera_num)]

        self.service = self.create_service(ImageBatch, 'get_segmented_images', self.give_service_to_global_planner)

        syn = message_filters.ApproximateTimeSynchronizer(self.image_subscriber, 1, 0.1)
        syn.registerCallback(self.image_callback)

        self.predictor = Predict()
        self.segmented_images = []

    def give_service_to_global_planner(self, request, response):
        self.get_logger().info('Incoming request from global planner')
        for i, segmented_image in enumerate(self.segmented_images):
            response.image[i] = CvBridge().cv2_to_imgmsg(cvim=segmented_image)
        return response

    def image_callback(self, *images):
        self.get_logger().info('Images received')
        cv_images = []
        for image in images:
            cv_images.append(CvBridge().imgmsg_to_cv2(image, desired_encoding='bgr8'))

        self.segmented_images.clear()
        predicted_images = self.predictor.predict(*cv_images)
        for i in range(self.overhead_camera_num):
            self.segmented_images.append(predicted_images[i, :, :].astype(np.float32)*255.0)

        if self.show_images:
            for i, cv_image in enumerate(cv_images):
                cv2.imshow("image" + str(i + 1), cv_image)
            cv2.waitKey(1)

            for i, segmented_image in enumerate(self.segmented_images):
                cv2.imshow("segmented image" + str(i + 1), segmented_image)
            cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    minimal_service = ImageSegmentationService()

    rclpy.spin(minimal_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
