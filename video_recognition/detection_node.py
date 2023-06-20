import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import time

class detection_node(Node):
    def __init__(self):
        super().__init__("detection_node")
        self.subscription = self.create_subscription(
            Image, "/camera/image_raw", self.image_callback, 5
        )
        self.publisher = self.create_publisher(Image, "/detection_result", 10)
        self.publisher_img = self.create_publisher(Image, "/detection_image", 10)
        self.network = cv2.dnn.readNetFromDarknet(
            '/model/yolov3_ts_test.cfg',
            '/model/yolov3.weights'
        )
        self.bridge = CvBridge()

    def image_callback(self, msg):
        start_time = time.time()
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        h, w = image.shape[:2]

        blob = cv2.dnn.blobFromImage(image, 1 / 255.0, (416, 416), swapRB=True, crop=False)
        self.network.setInput(blob)

        layers_names_output = ['yolo_82', 'yolo_94', 'yolo_106']
        output_from_network = self.network.forward(layers_names_output)

        probability_minimum = 0.2
        threshold = 0.2

        boxes = []
        confidences = []
        class_ids = []

        for output in output_from_network:
            for detection in output:
                scores = detection[5:]
                class_id = np.argmax(scores)
                confidence = scores[class_id]
                if confidence > probability_minimum:
                    box = detection[0:4] * np.array([w, h, w, h])

                    (box_center_x, box_center_y, box_width, box_height) = box.astype("int")
                    x_min = int(box_center_x - (box_width / 2))
                    y_min = int(box_center_y - (box_height / 2))

                    boxes.append([x_min, y_min, int(box_width), int(box_height)])
                    confidences.append(float(confidence))
                    class_ids.append(class_id)

        indices = cv2.dnn.NMSBoxes(boxes, confidences, probability_minimum, threshold)
        end_time = time.time()
        processing_time = end_time - start_time
        print("Time to frame:", processing_time)  
        if len(indices) > 0:
            for i in indices.flatten():
                (x, y) = (boxes[i][0], boxes[i][1])
                (width, height) = (boxes[i][2], boxes[i][3])

                cut_image = image[y_min:y_min + int(box_height), x_min:x_min + int(box_width), :]

                if cut_image.shape[:1] == (0,) or cut_image.shape[1:2] == (0,):
                    pass
                else:
                    cv2.rectangle(image, (x, y), (x + width, y + height), (0, 255, 0), 2)       
                    detection_image = self.bridge.cv2_to_imgmsg(image, encoding="bgr8")
                    self.publisher_img.publish(detection_image)
                    
                    area = cut_image.shape[0]*cut_image.shape[1]
                    if area > 5000:
                        detection_msg = self.bridge.cv2_to_imgmsg(cut_image, encoding="bgr8")
                        self.publisher.publish(detection_msg)
          







def main(args=None):
    rclpy.init(args=args)

    node = detection_node()
    
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
