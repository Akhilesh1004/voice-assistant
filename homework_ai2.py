import numpy as np
import tensorflow as tf



def l_reg(weight_matrix):
   #return 0.01 * tf.math.reduce_sum(tf.math.abs(weight_matrix))
   return 0.01 * tf.math.reduce_sum(tf.math.square(weight_matrix))
def preporocess(x,y):
    x = tf.cast(x,dtype=tf.float32) / 255
    x = tf.reshape(x,(-1,28 *28))
    x = tf.squeeze(x,axis=0)

    y = tf.cast(y,dtype=tf.int32)
    y = tf.one_hot(y,depth=10)
    return x,y

def main():

    mnist = tf.keras.datasets.mnist
    (train_x, train_y), (test_x, test_y) = mnist.load_data()
    print(test_x.shape)



    db = tf.data.Dataset.from_tensor_slices((train_x, train_y))  # 將x,y分成一一對應的


   #使用 tf.data.Dataset 的好處：
   #1直接對資料（tensor類型）進行預處理，支援batch和多執行緒的方式處理
   #2.提供了 .shuffle（打散）, .map（預處理） 功能

    #db = db.map(preporocess)  # 執行預處理函數
    #db = db.batch(2000)
    #print(db)


    db_test = tf.data.Dataset.from_tensor_slices((test_x, test_y))
    db = db.map(preporocess)  # 執行預處理函數
    db = db.shuffle(1000).batch(600)
    db_test = db_test.map(preporocess)
    db_test = db_test.shuffle(1000).batch(250)

    iter_num = 2000
    lr = 0.01

    model = tf.keras.Sequential([
        tf.keras.layers.Dense(1024, activation='relu'),
        tf.keras.layers.Dense(1024, activation='relu', kernel_regularizer=tf.keras.regularizers.L2(0.01)),
        tf.keras.layers.Dense(512, activation='relu'),
        tf.keras.layers.Dense(512, activation='relu'),
        tf.keras.layers.Dense(256, activation='relu'),
        tf.keras.layers.Dense(256, activation='relu', kernel_regularizer=tf.keras.regularizers.L2(0.01)),
        tf.keras.layers.Dense(128, activation='relu'),
        tf.keras.layers.Dense(128, activation='relu'),
        tf.keras.layers.Dense(64, activation='relu'),
        tf.keras.layers.Dense(64, activation='relu'),
        tf.keras.layers.Dense(32, activation='relu', kernel_regularizer=tf.keras.regularizers.L2(0.01)),
        tf.keras.layers.Dense(32, activation='relu'),
        tf.keras.layers.Dense(16, activation='relu'),
        tf.keras.layers.Dense(16, activation='relu'),
        tf.keras.layers.Dense(10)
    ])

    #optimizer = tf.keras.optimizers.SGD(learning_rate=lr)
    callback = tf.keras.callbacks.EarlyStopping(monitor='loss', patience=3)
    optimizer = tf.keras.optimizers.Adam(learning_rate=lr)
    model.compile(optimizer= optimizer,loss=tf.losses.CategoricalCrossentropy(from_logits=True),metrics=['accuracy'])
    model.fit(db ,epochs=30, callbacks=[callback],validation_data=db_test,validation_freq=3)
    model.evaluate(db_test)
    model.summary()

if __name__ == '__main__':
    main()
