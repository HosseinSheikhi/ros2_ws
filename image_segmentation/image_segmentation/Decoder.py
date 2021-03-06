import tensorflow as tf
from image_segmentation.VGGBlockWithBN import VggBlockWithBN
from image_segmentation.VggBlock import VggBlock


class Decoder(tf.keras.Model):
    def __init__(self, decoder_with_BN):
        super(Decoder, self).__init__()

        self.batch_norm = decoder_with_BN
        self.conv_blk_5 = None
        self.conv_blk_4 = None
        self.conv_blk_3 = None
        self.conv_blk_2 = None
        self.conv_blk_1 = None

        self.trans_conv_blk_1 = None
        self.trans_conv_blk_2 = None
        self.trans_conv_blk_3 = None
        self.trans_conv_blk_4 = None
        self.trans_conv_blk_5 = None

    def build(self, input_shape):
        if self.batch_norm:
            self.conv_blk_5 = VggBlockWithBN(layers=3, filters=512, kernel_size=3, name="dec_conv_blk5")
            self.conv_blk_4 = VggBlockWithBN(layers=3, filters=512, kernel_size=3, name="dec_conv_blk4")
            self.conv_blk_3 = VggBlockWithBN(layers=3, filters=256, kernel_size=3, name="dec_conv_blk3")
            self.conv_blk_2 = VggBlockWithBN(layers=2, filters=128, kernel_size=3, name="dec_conv_blk2")
            self.conv_blk_1 = VggBlockWithBN(layers=2, filters=64, kernel_size=3, name="dec_conv_blk1")
        else:
            self.conv_blk_5 = VggBlock(layers=3, filters=512, kernel_size=3, name="dec_conv_blk5")
            self.conv_blk_4 = VggBlock(layers=3, filters=512, kernel_size=3, name="dec_conv_blk4")
            self.conv_blk_3 = VggBlock(layers=3, filters=256, kernel_size=3, name="dec_conv_blk3")
            self.conv_blk_2 = VggBlock(layers=2, filters=128, kernel_size=3, name="dec_conv_blk2")
            self.conv_blk_1 = VggBlock(layers=2, filters=64, kernel_size=3, name="dec_conv_blk1")
        self.trans_conv_blk_5 = tf.keras.layers.Conv2DTranspose(512, 3, strides=2, padding="same", use_bias=False)
        self.trans_conv_blk_4 = tf.keras.layers.Conv2DTranspose(512, 3, strides=2, padding="same", use_bias=False)
        self.trans_conv_blk_3 = tf.keras.layers.Conv2DTranspose(256, 3, strides=2, padding="same", use_bias=False)
        self.trans_conv_blk_2 = tf.keras.layers.Conv2DTranspose(128, 3, strides=2, padding="same", use_bias=False)
        self.trans_conv_blk_1 = tf.keras.layers.Conv2DTranspose(64, 3, strides=2, padding="same", use_bias=False)

    def call(self, inputs, blk_1_out, blk_2_out, blk_3_out, blk_4_out, blk_5_out, training=False):
        x = self.trans_conv_blk_5(inputs)
        x = tf.keras.layers.concatenate([blk_5_out, x])
        x = self.conv_blk_5(x, training)

        x = self.trans_conv_blk_4(x)
        x = tf.keras.layers.concatenate([blk_4_out, x])
        x = self.conv_blk_4(x, training)

        x = self.trans_conv_blk_3(x)
        x = tf.keras.layers.concatenate([blk_3_out, x])
        x = self.conv_blk_3(x, training)

        x = self.trans_conv_blk_2(x)
        x = tf.keras.layers.concatenate([blk_2_out, x])
        x = self.conv_blk_2(x, training)

        x = self.trans_conv_blk_1(x)
        x = tf.keras.layers.concatenate([blk_1_out, x])
        x = self.conv_blk_1(x, training)

        return x
