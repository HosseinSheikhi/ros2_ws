import tensorflow as tf
from tensorflow.keras import layers

"""
Effectively, the Layer class corresponds to what we refer to in the literature as a "layer" (as in "convolution layer" 
or "recurrent layer") or as a "block" (as in "ResNet block" or "Inception block").
Meanwhile, the Model class corresponds to what is referred to in the literature as a "model"
(as in "deep learning model") or as a "network" (as in "deep neural network").

So if you're wondering, "should I use the Layer class or the Model class?", ask yourself: will I need to call fit() on it?
Will I need to call save() on it? If so, go with Model. If not (either because your class is just a block in a bigger 
system, or because you are writing training & saving code yourself), use Layer.

For more info for subclass layer: https://www.tensorflow.org/api_docs/python/tf/keras/layers/Layer 
"""


class VggBlock(tf.keras.layers.Layer):
    def __init__(self, layers, filters, kernel_size, name, stride=1):
        super(VggBlock, self).__init__()
        self.kernel_size = kernel_size
        self.filters = filters
        self.stride = stride
        self.layers = layers
        self.layer_name = name
        self.conv_layers = None

    def build(self, input_shape):
        self.conv_layers = [layers.Conv2D(self.filters, self.kernel_size, strides=self.stride, padding="same", activation='relu',
                                          kernel_initializer='he_normal', name=self.layer_name + "_" + str(i))
                            for i in range(self.layers)]

    def call(self, inputs, training=None):
        x = inputs
        for conv in self.conv_layers:
            x = conv(x)

        return x
