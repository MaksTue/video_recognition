import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import std_msgs
from std_msgs.msg import Int32
from cv_bridge import CvBridge
from keras.models import load_model
import pickle

class classification_node(Node):
    def __init__(self):
        super().__init__('classification_node')
        self.subscription = self.create_subscription(
            Image, '/detection_result', self.image_callback, 10
        )
        self.publisher = self.create_publisher(Int32, '/classification_result', 10)
        self.publisher_icon = self.create_publisher(Image, '/classification_image', 10)
        self.model = load_model('/model/model-3x3.h5')
        with open('/model/mean_image_rgb.pickle', 'rb') as f:
            self.mean_image_rgb = pickle.load(f, encoding='latin1')
        
        
        self.labels = { 0:'Speed limit (20km/h)',
            1:'Speed limit (30km/h)',
            2:'Speed limit (50km/h)',
            3:'Speed limit (60km/h)',
            4:'Speed limit (70km/h)',
            5:'Speed limit (80km/h)',
            6:'End of speed limit (80km/h)',
            7:'Speed limit (100km/h)',
            8:'Speed limit (120km/h)',
            9:'No passing',
            10:'No passing veh over 3.5 tons',
            11:'Right-of-way at intersection',
            12:'Priority road',
            13:'Yield',
            14:'Stop',
            15:'No vehicles',
            16:'Veh > 3.5 tons prohibited',
            17:'No entry',
            18:'General caution',
            19:'Dangerous curve left',
            20:'Dangerous curve right',
            21:'Double curve',
            22:'Bumpy road',
            23:'Slippery road',
            24:'Road narrows on the right',
            25:'Road work',
            26:'Traffic signals',
            27:'Pedestrians',
            28:'Children crossing',
            29:'Bicycles crossing',
            30:'Beware of ice/snow',
            31:'Wild animals crossing',
            32:'End speed + passing limits',
            33:'Turn right ahead',
            34:'Turn left ahead',
            35:'Ahead only',
            36:'Go straight or right',
            37:'Go straight or left',
            38:'Keep right',
            39:'Keep left',
            40:'Roundabout mandatory',
            41:'End of no passing',
            42:'End no passing veh > 3.5 tons' }

        self.bridge = CvBridge()

    def image_callback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
               
        # Предобработка изображения
        blob_ts = cv2.dnn.blobFromImage(image, 1 / 255.0, size=(32, 32), swapRB=True, crop=False)
        blob_ts[0] = blob_ts[0, :, :, :] - self.mean_image_rgb['mean_image_rgb']
        blob_ts = blob_ts.transpose(0, 2, 3, 1)

        # Классификация изображения
        scores = self.model.predict(blob_ts)
        prediction = np.argmax(scores)
        sign_label = self.labels[prediction]

        print(sign_label, "was detected")
        int_msg = std_msgs.msg.Int32()
        int_msg.data = int(prediction)
        self.publisher.publish(int_msg)

        image_path = "/traffic_sign/{}.png".format(prediction)
        image = cv2.imread(image_path)
        sign_image = self.bridge.cv2_to_imgmsg(image, encoding="bgr8")
        self.publisher_icon.publish(sign_image)


def main(args=None):
    rclpy.init(args=args)
    node = classification_node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
