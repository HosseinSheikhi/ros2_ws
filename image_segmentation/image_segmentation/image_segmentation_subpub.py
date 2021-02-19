from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.qos import qos_profile_sensor_data, qos_profile_system_default
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
        # define stitched image publisher for UI

        self.overhead_camera_num = 1
        self.image_publisher = [self.create_publisher(Image,
                                                      "autonomous_robot/image_segmentation/image_"+str(i+1),
                                                      qos_profile_system_default)
                                for i in range(self.overhead_camera_num)]
        self.show_images = False
        # define message_filter::subscriber to #self.overhead_camera_num overhead cameras
        self.image_subscriber = [
            message_filters.Subscriber(self, Image, '/overhead_cam_' + str(i + 1) + '/camera/image_raw',
                                       qos_profile=qos_profile_sensor_data)
            for i in range(self.overhead_camera_num)]

        self.timer = self.create_timer(1, self.publish_segmented_images)

        # define the policy for message filtering
        syn = message_filters.ApproximateTimeSynchronizer(self.image_subscriber, 1, 0.1)
        # message_filter::subscriber will start subscribing upon being registered
        syn.registerCallback(self.image_callback)

        self.predictor = Predict()
        self.segmented_images = []
        self.stitched_image = None

    # def give_service_to_global_planner(self):
    #     self.get_logger().info('Incoming request from global planner')
    #     self.image_stitching()
    #     if self.show_images:
    #         cv2.imshow("stitched image", self.stitched_image)
    #         cv2.waitKey(1)
    #     stitched_image_msg = CvBridge().cv2_to_imgmsg(self.stitched_image)
    #     # cv2.imwrite("/home/hossein/Desktop/TW.png", self.stitched_image)
    #     # publish for the ui
    #     self.stitched_image_publisher.publish(stitched_image_msg)
    #
    def publish_segmented_images(self):
        self.get_logger().info('Publishing images ...')
        for i in range(self.overhead_camera_num):
            self.image_publisher[i].publish(CvBridge().cv2_to_imgmsg(self.segmented_images[i]))

    def image_stitching(self):
        self.stitched_image = cv2.hconcat(self.segmented_images)

    def image_callback(self, *images):
        self.get_logger().info('Images received')
        cv_images = []
        for image in images:
            cv_images.append(CvBridge().imgmsg_to_cv2(image, desired_encoding='bgr8'))

        self.segmented_images.clear()
        predicted_images = self.predictor.predict(*cv_images)
        for i in range(self.overhead_camera_num):
            self.segmented_images.append(predicted_images[i, :, :].astype(np.float32) * 255.0)  # 2 (224,224)

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
