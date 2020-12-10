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

"""
This class subscribes to the overhead images and gives service to the global planner as follow:
1- service receives a request for segmented images from global_planner
3- feeds the overhead images to the segmentation_networks and returns the results to the global_planner 
"""


class ImageSegmentationService(Node):
    def __init__(self):
        super().__init__('image_segmentation_service')

        self.overhead_camera_num = 1
        self.show_images = True
        self.image_subscriber = [
            message_filters.Subscriber(self, Image, '/overhead_cam_' + str(i + 1) + '/camera/image_raw',
                                       qos_profile=qos_profile_sensor_data)
            for i in range(self.overhead_camera_num)]

        self.service = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

        syn = message_filters.ApproximateTimeSynchronizer(self.image_subscriber, 1, 0.1)
        syn.registerCallback(self.image_callback)

        self.predictor = Predict()

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info('Incoming request \n a: %d  b: %d' % (request.a, request.b))
        return response

    def image_callback(self, *images):
        self.get_logger().info('Images received')
        cv_images = []
        for image in images:
            cv_images.append(CvBridge().imgmsg_to_cv2(image, desired_encoding='bgr8'))

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
    minimal_service = ImageSegmentationService()

    rclpy.spin(minimal_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
