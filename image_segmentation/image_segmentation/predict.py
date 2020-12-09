import tensorflow as tf
import os
import matplotlib.pyplot as plt
from image_segmentation.EncoderDecoder import EncoderDecoder
from PIL import Image

os.environ["CUDA_VISIBLE_DEVICES"] = "-1"
NUM_CLASS = 2
IMAGE_SIZE = 224


class Predict():
    def __init__(self):
        self.model = EncoderDecoder(NUM_CLASS, decoder_with_BN=False)
        self.model.load_weights(
            "/home/hossein/ImageSegmentation/VGG16/weights/WithoutBN/FreezedEncoder/35KTrainingImagewithAugmentation/NaiveLoss3/")

    def normalize(self, input_image: tf.Tensor) -> tf.Tensor:
        input_image = tf.image.resize(input_image, (IMAGE_SIZE, IMAGE_SIZE))
        normalized_image = tf.image.per_image_standardization(input_image)
        return normalized_image

    def predict(self, *cv_images):
        """
         receives a batch of cv2 images (batch_size = num of overhead cameras)
         and returns the segmented images in cv2 format
        :param cv_images: batch of images
        :return: batch of segmented images
        """

        tf_images = tf.convert_to_tensor(list(cv_images), dtype=tf.float32)
        normalized_images = self.normalize(tf_images)
        pred = self.model(normalized_images, training=False)
        pred_mask = tf.argmax(pred, axis=-1)
        return pred_mask.numpy()

