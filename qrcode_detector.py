import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image # Image is the message type
from geometry_msgs.msg import Twist
import numpy as np
from std_msgs.msg import Bool

import cv2 # OpenCV library
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images


class QRCodeDetector(Node):

    def __init__(self):
        super().__init__('qrcode_detector')
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.listener_callback,
            qos_profile_sensor_data)
        self.subscription  # prevent unused variable warning
        self.command_publisher = self.create_publisher(Twist, "robot2/cmd_vel",10)
        self.msg_publisher = self.create_publisher(Bool, "qr_detected",10)
        # Used to convert between ROS and OpenCV images
        self.br = CvBridge()
        print("init finished")

    def send_vel(self):
        cmd=Twist()
        cmd.linear.x=0.0
        cmd.angular.z=0.5
        print("command sent!")
        self.command_publisher.publish(cmd)
        
    def send_stop(self):
        cmd=Twist()
        cmd.linear.x=0.0
        cmd.angular.z=0.0
        print("stop sent!")
        self.command_publisher.publish(cmd)
    
    def send_true(self):
        msg=Bool()
        msg.data=True
        print("msg sent")
        self.msg_publisher.publish(msg)
    
    def send_false(self):
        msg=Bool()
        msg.data=False
        print("msg sent")
        self.msg_publisher.publish(msg)
    
    def listener_callback(self, data):
        
        # Display the message on the console
        #self.get_logger().info('Receiving image')
        
        # Convert ROS Image message to OpenCV image
        frame = self.br.imgmsg_to_cv2(data, "bgr8")
        
        # initialize the cv2 QRCode detector
        detector = cv2.QRCodeDetector()

        # detect and decode
        data, bbox, _ = detector.detectAndDecode(frame)
	# self.send_vel()
        #if there is a QR code
        #if bbox is not None:
        #print("reading data")
        if data:
            print("[+] QR Code detected, data:", data)
            # display the image with lines
            bbox = bbox[0]
            #self.send_vel()
            self.send_true()
            for i in range(len(bbox)):
                pt1 = [int(val) for val in bbox[i]]
                pt2 = [int(val) for val in bbox[(i + 1) % len(bbox)]]
                cv2.line(frame, pt1, pt2, color=(255, 0, 0), thickness=3)

                bbox = np.int0(bbox)
                # Obtener las coordenadas del punto de origen del texto
                x = bbox[0][0]
                y = bbox[1][1] - 10

                # Agregar texto encima del cuadrado
                font = cv2.FONT_HERSHEY_SIMPLEX
                text = "QR Code: " + data
                org = (x, y)
                cv2.putText(frame, text, org, font, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
        else:
            print("No QR")
            #self.send_stop()
            #self.send_false()
        # display the result
        #cv2.imshow("Camera", frame)
        #cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    print("creating object")
    qrcode_detector = QRCodeDetector()
    rclpy.spin(qrcode_detector)
    qrcode_detector.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
