# Copyright 2019 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
from __future__ import annotations

import sys

import cv2 as cv
import cv_bridge
import message_filters
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from vision_msgs.msg import Detection2DArray
from std_msgs.msg import String
import cv2
import numpy as np
import selectivesearch
import os
import tensorflow as tf

PATCH_SIZE = 5
NUM_CLASSES = 43
NUM_EPOCH = 10
IMG_WIDTH = 32
IMG_HEIGHT = 32
IMG_CHANNELS = 3
IMG_SHAPE = (IMG_HEIGHT, IMG_WIDTH, IMG_CHANNELS)
BATCH_SIZE = 128


def params():
    # weights and biases
    params = {}

    params['w_conv1'] = tf.get_variable(
        'w_conv1',
        shape=[PATCH_SIZE, PATCH_SIZE, 3, 32],
        initializer=tf.contrib.layers.xavier_initializer())
    params['b_conv1'] = tf.Variable(tf.constant(0.1, shape=[32]))

    params['w_conv2'] = tf.get_variable(
        'w_conv2',
        shape=[PATCH_SIZE, PATCH_SIZE, 32, 64],
        initializer=tf.contrib.layers.xavier_initializer())
    params['b_conv2'] = tf.Variable(tf.constant(0.1, shape=[64]))

    params['w_conv3'] = tf.get_variable(
        'w_conv3',
        shape=[PATCH_SIZE, PATCH_SIZE, 64, 128],
        initializer=tf.contrib.layers.xavier_initializer())
    params['b_conv3'] = tf.Variable(tf.constant(0.1, shape=[128]))

    params['w_fc1'] = tf.get_variable(
        'w_fc1',
        shape=[4 * 4 * 128, 2048],
        initializer=tf.contrib.layers.xavier_initializer())
    params['b_fc1'] = tf.Variable(tf.constant(0.1, shape=[2048]))

    params['w_fc2'] = tf.get_variable(
        'w_fc2',
        shape=[2048, NUM_CLASSES],
        initializer=tf.contrib.layers.xavier_initializer())
    params['b_fc2'] = tf.Variable(tf.constant(0.1, shape=[NUM_CLASSES]))

    return params


def cnn(data, model_params, keep_prob):
    # First layer
    h_conv1 = tf.nn.relu(
        tf.nn.conv2d(
            data, model_params['w_conv1'], [1, 1, 1, 1], padding='SAME') +
        model_params['b_conv1'])
    h_pool1 = tf.nn.max_pool(
        h_conv1, ksize=[1, 2, 2, 1], strides=[1, 2, 2, 1], padding='SAME')

    # Second layer
    h_conv2 = tf.nn.relu(
        tf.nn.conv2d(
            h_pool1, model_params['w_conv2'], [1, 1, 1, 1], padding='SAME') +
        model_params['b_conv2'])
    h_pool2 = tf.nn.max_pool(
        h_conv2, ksize=[1, 2, 2, 1], strides=[1, 2, 2, 1], padding='SAME')

    # Third layer
    h_conv3 = tf.nn.relu(
        tf.nn.conv2d(
            h_pool2, model_params['w_conv3'], [1, 1, 1, 1], padding='SAME') +
        model_params['b_conv3'])
    h_pool3 = tf.nn.max_pool(
        h_conv3, ksize=[1, 2, 2, 1], strides=[1, 2, 2, 1], padding='SAME')

    # Fully connected layer
    conv_layer_flat = tf.reshape(h_pool3, [-1, 4 * 4 * 128])
    h_fc1 = tf.nn.relu(
        tf.matmul(conv_layer_flat, model_params['w_fc1']) +
        model_params['b_fc1'])
    h_fc1 = tf.nn.dropout(h_fc1, keep_prob)

    # Output layer
    out = tf.matmul(h_fc1, model_params['w_fc2']) + model_params['b_fc2']

    return out


class TrafficSignRecognizer:
    def __init__(self, mode, model_dir):
        assert mode in {'train', 'inference'}
        self.mode = mode
        self.model_dir = model_dir
        self.recognizer_model = self.build(mode)

    def build(self, mode):
        assert mode in {'train', 'inference'}
        input_image = tf.keras.layers.Input(
            shape=[IMG_HEIGHT, IMG_WIDTH, IMG_CHANNELS])
        x = tf.keras.layers.Conv2D(
            32, PATCH_SIZE, padding='same', activation='relu')(input_image)
        x = tf.keras.layers.MaxPool2D()(x)
        x = tf.keras.layers.Conv2D(
            64, PATCH_SIZE, padding='same', activation='relu')(x)
        x = tf.keras.layers.MaxPool2D()(x)
        x = tf.keras.layers.Conv2D(
            128, PATCH_SIZE, padding='same', activation='relu')(x)
        x = tf.keras.layers.MaxPool2D()(x)
        x = tf.keras.layers.Flatten()(x)
        x = tf.keras.layers.Dense(4*4*128, activation='relu')(x)
        if mode == 'train':
            x = tf.keras.layers.Dropout(0.5)(x, training=True)
        outputs = tf.keras.layers.Dense(NUM_CLASSES, activation='softmax')(x)
        return tf.keras.Model(inputs=input_image, outputs=outputs)

    def compile(self, learning_rate):
        optimizer = tf.keras.optimizers.Adam(learning_rate=learning_rate)
        self.recognizer_model.compile(
            optimizer=optimizer, loss='categorical_crossentropy', metrics=['accuracy'])

    def train(self, train_dataset, train_labels, learning_rate):
        assert self.mode == 'train', 'Create model in train mode'

        # Callbacks
        callbacks = [
            tf.keras.callbacks.TensorBoard(log_dir=os.path.join(
                self.model_dir, 'log_dir'), histogram_freq=0, write_graph=True),
            tf.keras.callbacks.ModelCheckpoint(
                os.path.join(self.model_dir, 'ckpt'), verbose=0, save_weights_only=True),
        ]

        # Compile
        self.compile(learning_rate)

        # Do training
        self.recognizer_model.fit(
            train_dataset, train_labels, callbacks=callbacks, epochs=NUM_EPOCH, validation_split=0.2)


class Image2Code(Node):

    def __init__(self):
        super().__init__('signal_recognize')

        self._bridge = cv_bridge.CvBridge()
        
        self._command_pub = self.create_publisher(String, "/output_command",10)
        self._image_sub = self.create_subscription(CompressedImage, "/oakd/rgb/preview/image_raw/compressed", self.on_imageread, 10)
        
    def loop(self):
        self._image_sub.callback()

    def get_object_proposals(img, scale=500, sigma=0.9, min_size=10):
    # Selective search
        img_lbl, regions = selectivesearch.selective_search(
            img, scale=scale, sigma=sigma, min_size=min_size)

        candidates = set()
        for r in regions:
            # excluding same rectangle (with different segments)
            if r['rect'] in candidates:
                continue
            # excluding regions smaller than 500 pixels
            x, y, w, h = r['rect']
            if r['size'] < 2000 or w > 0.95 * img.shape[1] or h > 0.95 * img.shape[0]:
                continue
            # excluding the zero-width or zero-height box
            if r['rect'][2] == 0 or r['rect'][3] == 0:
                continue
            # distorted rects
            if w / h > 5 or h / w > 5:
                continue
            candidates.add(r['rect'])

        return candidates

    def setup_graph():
        graph_params = {}
        graph_params['graph'] = tf.Graph()
        with graph_params['graph'].as_default():
            model_params = params()
            graph_params['target_image'] = tf.placeholder(
                tf.float32,
                shape=(1, IMG_HEIGHT, IMG_WIDTH, IMG_CHANNELS))
            logits = cnn(
                graph_params['target_image'], model_params, keep_prob=1.0)
            graph_params['pred'] = tf.nn.softmax(logits)
            graph_params['saver'] = tf.train.Saver()
        return graph_params

    def traffic_sign_recognition(sess, img, obj_proposal, graph_params):
    # recognition results
        recog_results = {}
        recog_results['obj_proposal'] = obj_proposal

        # Resize image
        if img.shape != IMG_SHAPE:
            img = cv2.resize(img, (IMG_WIDTH, IMG_HEIGHT))

        # Pre-processing(Hist equalization)
        img = cv2.cvtColor(img, cv2.COLOR_BGR2YCrCb)
        split_img = cv2.split(img)
        split_img[0] = cv2.equalizeHist(split_img[0])
        eq_img = cv2.merge(split_img)
        eq_img = cv2.cvtColor(eq_img, cv2.COLOR_YCrCb2BGR)
        # Scaling in [0, 1]
        eq_img = (eq_img / 255.).astype(np.float32)
        eq_img = np.expand_dims(eq_img, axis=0)

        # Traffic sign recognition
        pred = sess.run(
            [graph_params['pred']],
            feed_dict={graph_params['target_image']: eq_img})
        recog_results['pred_class'] = np.argmax(pred)
        recog_results['pred_prob'] = np.max(pred)

        return recog_results
    
    def update_idx(results):
        probs = np.array([r['pred_prob'] for r in results])
        idx = np.argsort(probs)[::-1]
        return idx
    
    def iou_xywh(box1, box2):
        """

        Arguments:
        box1 -- rectangles of object proposals with coordinates (x, y, w, h)
        box2 -- rectangle of ground truth with coordinates (x1, y1, w, h)
        """
        xi1 = max(box1[0], box2[0])
        yi1 = max(box1[1], box2[1])
        xi2 = min(box1[0] + box1[2], box2[0] + box2[2])
        yi2 = min(box1[1] + box1[3], box2[1] + box2[3])
        inter_area = (yi2 - yi1) * (xi2 - xi1)

        # Calculate the union area by using formula: union(A, B) = A + B - inter_area
        box1_area = box1[2] * box1[3]
        box2_area = box2[2] * box2[3]
        union_area = box1_area + box2_area - inter_area

        # Compute the IoU
        iou = inter_area / union_area

        return iou

    
    def nms(self,recog_results, pred_prob_th=0.99, iou_th=0.5):
    # nms results
        nms_results = []

        # Discard all results with prob <= pred_prob_th
        pred_probs = np.array([r['pred_prob'] for r in recog_results])
        cand_idx = np.where(pred_probs > pred_prob_th)[0]
        cand_results = np.array(recog_results)[cand_idx]
        if len(cand_results) == 0:
            return nms_results

        # Sort in descending order
        cand_nms_idx = self.update_idx(cand_results)

        #
        # [Non-max suppression]
        #

        # Pick the result with the largest prob as a prediction
        pred = cand_results[cand_nms_idx[0]]
        nms_results.append(pred)
        if len(cand_results) == 1:
            return nms_results
        cand_results = cand_results[cand_nms_idx[1:]]
        cand_nms_idx = self.update_idx(cand_results)

        # Discard any remaining results with IoU >= iou_th
        while len(cand_results) > 0:
            del_idx = []
            del_seq_idx = []
            for seq_i, i in enumerate(cand_nms_idx):
                if self.iou_xywh(cand_results[i]['obj_proposal'],
                            pred['obj_proposal']) >= iou_th:
                    del_idx.append(i)
                    del_seq_idx.append(seq_i)
            # Delete non-max results
            cand_results = np.delete(cand_results, del_idx)
            if len(cand_results) == 0:
                break
            cand_nms_idx = self.update_idx(cand_results)
            # For next iteration
            pred, cand_results = cand_results[cand_nms_idx[0]], cand_results[
                cand_nms_idx[1:]]
            if len(cand_results) == 0:
                break
            cand_nms_idx = self.update_idx(cand_results)
            nms_results.append(pred)

        return nms_results

    def on_imageread(self, msg):
        image_msg = msg
        cv_image = self._bridge.compressed_imgmsg_to_cv2(image_msg)
        object_proposals = self.get_object_proposals(cv_image)

        # Setup computation graph
        graph_params = self.setup_graph()

        # Model initialize
        sess = tf.Session(graph=graph_params['graph'])
        tf.global_variables_initializer()
        if os.path.exists('models'):
            save_path = os.path.join('models', 'deep_traffic_sign_model')
            graph_params['saver'].restore(sess, save_path)
            print('Model restored')
        else:
            print('Initialized')

        # traffic sign recognition
        results = []
        for obj_proposal in object_proposals:
            x, y, w, h = obj_proposal
            crop_image = cv_image[y:y + h, x:x + w]
            results.append(
                self.traffic_sign_recognition(sess, crop_image, obj_proposal,
                                        graph_params))
        """
        del_idx = []
        for i, result in enumerate(results):
            if result['pred_class'] == common.CLASS_NAME[-1]:
                del_idx.append(i)
        results = np.delete(results, del_idx)
        """
        # Non-max suppression
        nms_results = self.nms(results, pred_prob_th=0.999999, iou_th=0.4)

        out = String()
        out.data = results[0]['pred_class']
        self.get_logger().info(results[0]['pred_class'])
        self._command_pub.publish(out)


def main():
    
    rclpy.init()
    decoder = Image2Code()
    rclpy.spin(decoder)
    decoder.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()