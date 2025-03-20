#!/usr/bin/env python3
#
#   detectaruco.py
#
#   Detect the ArUco marker with OpenCV.
#
#   Node:           /detectaruco
#   Subscribers:    /usb_cam2/image_raw          Source image
#   Publishers:     /detectaruco/image_raw      Debug image
#
import cv2
import numpy as np

# ROS Imports
import rclpy
import cv_bridge

from rclpy.node         import Node
from sensor_msgs.msg    import Image

from geometry_msgs.msg  import Point
from geometry_msgs.msg  import Pose

from custom_msgs.msg import Object

from datetime import datetime, timedelta
#
#  Detector Node Class
#
class DetectorNode(Node):
    # Pick some colors, assuming RGB8 encoding.
    red    = (255,   0,   0)
    green  = (  0, 255,   0)
    blue   = (  0,   0, 255)
    yellow = (255, 255,   0)
    white  = (255, 255, 255)

    # Initialization.
    def __init__(self, name):
        # Initialize the node, naming it as specified
        super().__init__(name)

        # Create a publisher for the processed (debugging) image.
        # Store up to three images, just in case.
        self.pub = self.create_publisher(Image, name+'/image_raw', 3)

        # Set up the OpenCV bridge.
        self.bridge = cv_bridge.CvBridge()

        # Set up the ArUco detector.  Use a dictionary with the
        # appropriate tags.  DICT_6x6_1000 has 1000 options (way too
        # many).  DICT_6x6_250 has 250 options.  DICT_6x6_50 is
        # probably enough for our projects...
        self.dict   = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
        self.params = cv2.aruco.DetectorParameters_create()

        # Finally, subscribe to the incoming image topic.  Using a
        # queue size of one means only the most recent message is
        # stored for the next subscriber callback.
        self.sub = self.create_subscription(
            Image, '/image_raw', self.process, 1)
        
        self.objspub = self.create_publisher(Object, '/targets', 10)
        
        self.start = datetime.now()
        self.min_pos = np.inf
        self.max_pos = -np.inf
        self.max_time = None
        self.min_time = None
        
        self.curr_start = False
        self.curr_start_time = None
        self.curr_min_pos = np.inf
        self.curr_max_pos = -np.inf
        self.curr_min_time = None
        self.curr_max_time = None
        
        self.print = False
        
        self.M = None
        
    def predict_pos(self, t):
        hold = 2
        diff = np.abs((self.min_time - self.max_time).total_seconds())
        slope = (self.max_pos - self.min_pos)/(diff - hold)
        total_period = 2 * diff
        phase = (t - self.max_time).total_seconds() % total_period
        if phase <= hold:
            return self.max_pos
        elif phase <= diff:
            return self.max_pos - slope * (phase - hold)
        elif phase <= diff + hold:
            return self.min_pos
        else:
            return self.min_pos + slope * (phase - diff - hold)
        

    # Shutdown
    def shutdown(self):
        # No particular cleanup, just shut down the node.
        self.destroy_node()


    # Process the image (detect the aruco).
    def process(self, msg):
        # Confirm the encoding and report.
        assert(msg.encoding == "rgb8")
        # self.get_logger().info(
        #     "Image %dx%d, bytes/pixel %d, encoding %s" %
        #     (msg.width, msg.height, msg.step/msg.width, msg.encoding))
 
        # Convert into OpenCV image, using RGB 8-bit (pass-through).
        frame = self.bridge.imgmsg_to_cv2(msg, "passthrough")

        # Detect.
        (boxes, ids, rejected) = cv2.aruco.detectMarkers(
            frame, self.dict, parameters=self.params)
       
        if ids is None:
            return None

        # Determine the center of the marker pixel coordinates.
        flag = False
        count = 0
        uvMarkers = np.zeros((4,2), dtype='float32')
        for i in range(len(ids)):
            if ids[i] >= 4 and ids[i] <= 7:
                uvMarkers[ids[i] - 4,:] = np.mean(boxes[i], axis=1)
                count += 1
            elif ids[i] == 0:
                flag = True
                u, v = np.mean(boxes[i], axis=1)[0]
            
        if not flag:
            return None
        
        imp = False    
        if count != 4:
            if self.M is None:
                return None
            imp = True
            
        x0 = 0.0155
        z0 = 0.5605
        # Calculate the matching World coordinates of the 4 Aruco markers.
        DX = 0.22
        DZ = 0.1755
        xzMarkers = np.float32([[x0+dx, z0+dz] for (dx, dz) in
                                [(-DX, DZ), (DX, DZ), (-DX, -DZ), (DX, -DZ)]])

        if not imp:
            # Create the perspective transform.
            M = cv2.getPerspectiveTransform(uvMarkers, xzMarkers)
            self.M = M

        # Map the object in question.
        uvObj = np.float32([u, v])
        xzObj = cv2.perspectiveTransform(uvObj.reshape(1,1,2), self.M).reshape(2)
        
        s = "(%7.4f, %7.4f)" % (xzObj[0], xzObj[1])
        cv2.putText(frame, s, (int(u)-80, int(v)-8), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (255, 0, 0), 2, cv2.LINE_AA)
                    
        # Convert the frame back into a ROS image and republish.
        self.pub.publish(self.bridge.cv2_to_imgmsg(frame, "rgb8"))
        
        if ((datetime.now() - self.start).total_seconds() < 30):
            if float(xzObj[0]) > self.max_pos:
                self.max_pos = float(xzObj[0])
                self.max_time = datetime.now()
            if float(xzObj[0]) < self.min_pos:
                self.min_pos = float(xzObj[0])
                self.min_time = datetime.now()
        else:
            if not self.print:
                self.get_logger().info("Start running robot...")
                self.print = True
            if self.curr_start == False:
               self.curr_start = True
               self.curr_start_time = datetime.now()
               self.curr_min_pos = np.inf
               self.curr_max_pos = -np.inf
               self.curr_min_time = None
               self.curr_max_time = None
            
            if self.curr_start:     
                if float(xzObj[0]) > self.curr_max_pos:
                    self.curr_max_pos = float(xzObj[0])
                    self.curr_max_time = datetime.now()
                if float(xzObj[0]) < self.curr_min_pos:
                    self.curr_min_pos = float(xzObj[0])
                    self.curr_min_time = datetime.now()
                
            if self.curr_start and (datetime.now() - self.curr_start_time).total_seconds() >= 30:
                self.get_logger().info("Updating...")
                self.max_pos = self.curr_max_pos
                self.max_time = self.curr_max_time
                self.min_pos = self.curr_min_pos
                self.min_time = self.curr_min_time
                self.curr_start = False
            
            obj = Object()
            obj.type = 2
            pose = Pose()
        
            #pose.position.x = float(xzObj[0])
            
            #self.get_logger().info("%r" % self.predict_pos(datetime.now()))
            pose.position.x = self.predict_pos(datetime.now() + timedelta(seconds=12))
            pose.position.y = 1.473
            pose.position.z = float(xzObj[1])
            obj.pose = pose
            self.objspub.publish(obj)
        
        return xzObj


#

#   Main Code
#
def main(args=None):
    # Initialize ROS.
    rclpy.init(args=args)

    # Instantiate the detector node.
    node = DetectorNode('backboarddetector')

    # Spin the node until interrupted.
    rclpy.spin(node)

    # Shutdown the node and ROS.
    node.shutdown()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
