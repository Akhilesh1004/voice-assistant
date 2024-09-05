import numpy as np
import random
import tensorflow as tf
import keras.backend as K
from keras.datasets import cifar10
from keras.layers import Dense, Activation, Conv2D, MaxPool2D, Flatten, Dropout
from keras.models import Sequential
from keras.utils import to_categorical
from keras import optimizers
from keras import regularizers
from keras.preprocessing import image
from sklearn.model_selection import train_test_split


def relu(x, alpha=0.0, max_value=None, threshold=0.0):

    dtype = getattr(x, "dtype", "float32")
    if alpha != 0.0:
        if max_value is None and threshold == 0:
            return tf.nn.leaky_relu(x, alpha=alpha)

        if threshold != 0:
            negative_part = tf.nn.relu(-x + threshold)
        else:
            negative_part = tf.nn.relu(-x)

    clip_max = max_value is not None

    if threshold != 0:
        # computes x for x > threshold else 0
        x = x * tf.cast(tf.greater(x, threshold), dtype=dtype)
    elif max_value == 6:
        # if no threshold, then can use nn.relu6 native TF op for performance
        x = tf.nn.relu6(x)
        clip_max = False
    else:
        x = tf.nn.relu(x)

    if clip_max:
        max_value = tf.constant(max_value, x.dtype.base_dtype)
        zero = tf.constant(0, x.dtype.base_dtype)
        x = tf.clip_by_value(x, zero, max_value)

    if alpha != 0.0:
        alpha = tf.convert_to_tensor(alpha, x.dtype.base_dtype)
        x -= alpha * negative_part
    return x

def softmax(x, axis=-1):
    if x.shape.rank > 1:
        if isinstance(axis, int):
            output = tf.nn.softmax(x, axis=axis)
        else:
            # nn.softmax does not support tuple axis.
            e = tf.exp(x - tf.reduce_max(x, axis=axis, keepdims=True))
            s = tf.reduce_sum(e, axis=axis, keepdims=True)
            output = e / s
    else:
        raise ValueError(
            f"Cannot apply softmax to a tensor that is 1D. Received input: {x}"
        )

    # Cache the logits to use for crossentropy loss.
    output._keras_logits = x
    return output

def loss_function(yHat, y):
   return  abs(K.sum((y-yHat)))

#load data
(train_images, train_labels), (test_images, test_labels) = cifar10.load_data( )

#data preprocess
image = []
label = []
for i in range(10):
  x = 0
  for j in range(train_images.shape[0]):
    if train_labels[j][0] == i:
      image.append(train_images[j])
      label.append(train_labels[j])
      x += 1
    if x == 1000:
      break
train_images = np.array(image)
train_labels = np.array(label)

print(train_images.shape)
print(train_labels.shape)
print(test_images.shape)
print(test_labels.shape)

#data enhancement
for i in range(train_images.shape[0]):
  img_ud = tf.image.flip_up_down(train_images[i])
  img_lr = tf.image.flip_left_right(train_images[i])
  img_t = tf.image.transpose(train_images[i])
  img_c = tf.image.random_contrast(train_images[i],lower=0.8,upper=3)
  img_h = tf.image.random_hue(train_images[i],max_delta=0.3)
  img_s = tf.image.random_saturation(train_images[i],lower=0,upper=4)
  img_ud_c_h = tf.image.random_hue(tf.image.random_contrast(img_ud,lower=0.8,upper=3),max_delta=0.3)
  img_lr_c_h = tf.image.random_hue(tf.image.random_contrast(img_lr,lower=0.8,upper=3),max_delta=0.3)
  img_t_c_h = tf.image.random_hue(tf.image.random_contrast(img_t,lower=0.8,upper=3),max_delta=0.3)

  x = random.randint(0, train_images.shape[0]-1)
  train_images =np.insert(train_images,x, img_ud,axis=0)
  train_labels =np.insert(train_labels,x, train_labels[i],axis=0)

  x = random.randint(0, train_images.shape[0]-1)
  train_images =np.insert(train_images,x, img_lr,axis=0)
  train_labels =np.insert(train_labels,x, train_labels[i],axis=0)

  x = random.randint(0, train_images.shape[0]-1)
  train_images =np.insert(train_images,x, img_t,axis=0)
  train_labels =np.insert(train_labels,x, train_labels[i],axis=0)

  x = random.randint(0, train_images.shape[0]-1)
  train_images =np.insert(train_images,x, img_c,axis=0)
  train_labels =np.insert(train_labels,x, train_labels[i],axis=0)

  x = random.randint(0, train_images.shape[0]-1)
  train_images =np.insert(train_images,x, img_h,axis=0)
  train_labels =np.insert(train_labels,x, train_labels[i],axis=0)

  x = random.randint(0, train_images.shape[0]-1)
  train_images =np.insert(train_images,x, img_s,axis=0)
  train_labels =np.insert(train_labels,x, train_labels[i],axis=0)

  x = random.randint(0, train_images.shape[0]-1)
  train_images =np.insert(train_images,x, img_ud_c_h,axis=0)
  train_labels =np.insert(train_labels,x, train_labels[i],axis=0)

  x = random.randint(0, train_images.shape[0]-1)
  train_images =np.insert(train_images,x, img_lr_c_h,axis=0)
  train_labels =np.insert(train_labels,x, train_labels[i],axis=0)

  x = random.randint(0, train_images.shape[0]-1)
  train_images =np.insert(train_images,x, img_t_c_h,axis=0)
  train_labels =np.insert(train_labels,x, train_labels[i],axis=0)

  train_images_ = train_images
  train_labels_ = train_labels

  print(train_images.shape)
  print(train_labels.shape)

#data preprocess
train_images = train_images.astype('float32')
test_images = test_images.astype('float32')
train_images = train_images/255
test_images = test_images/255
train_labels = to_categorical( train_labels )
test_labels = to_categorical( test_labels )
print(train_images.shape)
print(train_labels.shape)
print(test_images.shape)
print(test_labels.shape)

#build model 68%
network = Sequential( )
network.add(Conv2D(filters=32, kernel_size=3, input_shape=(32, 32, 3), activation='relu', kernel_initializer="he_uniform"))
network.add(BatchNormalization())
network.add(Dropout(0.2))
network.add(Conv2D(filters=64, kernel_size=3, input_shape=(32, 32, 3), activation='relu', kernel_initializer="he_uniform"))
network.add(BatchNormalization())
network.add(MaxPooling2D(2, 2))
network.add(Dropout(0.2))
network.add(Conv2D(filters=128, kernel_size=3, activation='relu', kernel_initializer="he_uniform"))
network.add(BatchNormalization())
network.add(MaxPooling2D(2, 2))
network.add(Dropout(0.2))
network.add(Conv2D(filters=64, kernel_size=3, activation='relu', kernel_initializer="he_uniform"))
network.add(BatchNormalization())
network.add(MaxPooling2D(2, 2))
network.add(Dropout(0.2))
network.add(Flatten())
network.add( Dense( 256 , activation='relu', kernel_initializer="he_uniform") )
network.add(Dropout(0.1))
network.add( Dense( 10 , activation='softmax', kernel_initializer="glorot_uniform") )
optimizer = optimizers.Adam()
network.compile( optimizer=optimizer, loss = "categorical_crossentropy", metrics = ['accuracy'] )
print( network.summary() )

##build model 73%
network = Sequential( )
network.add(Conv2D(filters=32, kernel_size=3, input_shape=(32, 32, 3), activation='relu', padding = 'same', kernel_initializer="he_uniform"))
network.add(Conv2D(filters=32, kernel_size=3, activation='relu', padding = 'same', kernel_initializer="he_uniform"))
network.add(BatchNormalization())
network.add(MaxPooling2D(2, 2))
network.add(Dropout(0.2))
network.add(Conv2D(filters=64, kernel_size=3, activation='relu', padding = 'same', kernel_initializer="he_uniform"))
network.add(Conv2D(filters=64, kernel_size=3, activation='relu', padding = 'same', kernel_initializer="he_uniform"))
network.add(BatchNormalization())
network.add(MaxPooling2D(2, 2))
network.add(Dropout(0.2))
network.add(Conv2D(filters=128, kernel_size=3, activation='relu', padding = 'same', kernel_initializer="he_uniform"))
network.add(Conv2D(filters=128, kernel_size=3, activation='relu', padding = 'same', kernel_initializer="he_uniform"))
network.add(BatchNormalization())
network.add(MaxPooling2D(2, 2))
network.add(Dropout(0.2))
network.add(Flatten())
network.add( Dense( 256 , activation='relu', kernel_initializer="he_uniform") )
network.add(BatchNormalization())
network.add(Dropout(0.1))
network.add( Dense( 128 , activation='relu', kernel_initializer="he_uniform") )
network.add(Dropout(0.1))
network.add( Dense( 10 , activation='softmax', kernel_initializer="glorot_uniform") )
optimizer = optimizers.Adam(lr = 0.001)
network.compile( optimizer=optimizer, loss = "categorical_crossentropy", metrics = ['accuracy'] )
print( network.summary() )

#training
network.fit( train_images, train_labels, epochs = 30, batch_size = 200, validation_data=(test_images, test_labels))
test_loss, test_acc = network.evaluate( test_images, test_labels )
print( "Test Accuracy: %f, Test Loss: %f,"%(test_acc, test_loss) )
