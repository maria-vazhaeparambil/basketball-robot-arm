#!/usr/bin/env python3
#
#   balldetector.py
#
#   Detect the tennis balls with OpenCV.
#
#   Node:           /balldetector
#   Subscribers:    /usb_cam/image_raw          Source image
#   Publishers:     /balldetector/binary        Intermediate binary image
#                   /balldetector/image_raw     Debug (marked up) image
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
from custom_msgs.msg import ObjectArray
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
    
    diameter = 0.135

    # Initialization.
    def __init__(self, name):
        # Initialize the node, naming it as specified
        super().__init__(name)

        # Thresholds in Hmin/max, Smin/max, Vmin/max
        self.hsvlimits = np.array([[144, 172], [95, 255], [75, 255]])

        # Create a publisher for the processed (debugging) images.
        # Store up to three images, just in case.
        self.pubrgb = self.create_publisher(Image, name+'/image_raw', 3)
        self.pubbin = self.create_publisher(Image, name+'/binary',    3)

        # Set up the OpenCV bridge.
        self.bridge = cv_bridge.CvBridge()

        # Finally, subscribe to the incoming image topic.  Using a
        # queue size of one means only the most recent message is
        # stored for the next subscriber callback.
        self.sub = self.create_subscription(
            Image, '/image_raw', self.process, 1)

        # Report.
        self.get_logger().info("Ball detector running...")
        
        self.objspub = self.create_publisher(ObjectArray, '/objects', 10)
        
        self.stagnant = {}
        self.threshold = 10
        
        self.M = None

    # Shutdown
    def shutdown(self):
        # No particular cleanup, just shut down the node.
        self.destroy_node()

    # Pixel Conversion
    def pixelToWorld(self, image, u, v, x0, y0, annotateImage=True):
        '''
        Convert the (u,v) pixel position into (x,y) world coordinates
        Inputs:
          image: The image as seen by the camera
          u:     The horizontal (column) pixel coordinate
          v:     The vertical (row) pixel coordinate
          x0:    The x world coordinate in the center of the marker paper
          y0:    The y world coordinate in the center of the marker paper
          annotateImage: Annotate the image with the marker information

        Outputs:
          point: The (x,y) world coordinates matching (u,v), or None

        Return None for the point if not all the Aruco markers are detected
        '''
        
        # Detect the Aruco markers (using the 4X4 dictionary).
        markerCorners, markerIds, _ = cv2.aruco.detectMarkers(
            image, cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50))
        if annotateImage:
            cv2.aruco.drawDetectedMarkers(image, markerCorners, markerIds)
        
        imp = True
        
        # Abort if not all markers are detected.
        if (markerIds is None or len(markerIds) != 4 or
            set(markerIds.flatten()) != set([0,1,2,3])):
            if self.M is None:
                return None
            else:
                imp = False

        if imp:
            # Determine the center of the marker pixel coordinates.
            uvMarkers = np.zeros((4,2), dtype='float32')
            for i in range(4):
                uvMarkers[markerIds[i],:] = np.mean(markerCorners[i], axis=1)

            # Calculate the matching World coordinates of the 4 Aruco markers.
            DX = 0.2325
            DY = 0.5
            xyMarkers = np.float32([[x0+dx, y0+dy] for (dx, dy) in
                                   [(-DX, DY), (DX, DY), (-DX, -DY), (DX, -DY)]])


            # Create the perspective transform.
            M = cv2.getPerspectiveTransform(uvMarkers, xyMarkers)
            self.M = M

        # Map the object in question.
        uvObj = np.float32([u, v])
        xyObj = cv2.perspectiveTransform(uvObj.reshape(1,1,2), self.M).reshape(2)

        # Mark the detected coordinates.
        if annotateImage:
            # cv2.circle(image, (u, v), 5, (0, 0, 0), -1)
            s = "(%7.4f, %7.4f)" % (xyObj[0], xyObj[1])
            cv2.putText(image, s, (u-80, v-8), cv2.FONT_HERSHEY_SIMPLEX,
                        0.5, (255, 0, 0), 2, cv2.LINE_AA)

        return xyObj
        
    def stationary(self, lst):
        
        ret = ObjectArray()
        n = {}
        for (x, y) in lst:
            k1 = (x, y)
            flag = False
            for k2 in self.stagnant.keys():
                if np.linalg.norm(np.array(k1) - np.array(k2)) < 0.01:
                    if self.stagnant[k2] + 1 > self.threshold:
                        obj = Object()
                        obj_type = 0
                        pose = Pose()
                        pose.position.x = float(x)
                        pose.position.y = float(y)
                        pose.position.z = self.diameter
                        obj.type = obj_type
                        obj.pose = pose
                        ret.objects.append(obj)
                    else:
                        n[k1] = self.stagnant[k2] + 1
                    flag = True
            if not flag:
                n[k1] = 1
        self.stagnant = n
        self.objspub.publish(ret)


    # Process the image (detect the ball).
    def process(self, msg):
        # Confirm the encoding and report.
        assert(msg.encoding == "rgb8")
        # self.get_logger().info(
        #     "Image %dx%d, bytes/pixel %d, encoding %s" %
        #     (msg.width, msg.height, msg.step/msg.width, msg.encoding))

        # Convert into OpenCV image, using RGB 8-bit (pass-through).
        frame = self.bridge.imgmsg_to_cv2(msg, "passthrough")

        # Convert to HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)
        # hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)  # Cheat: swap red/blue

        # Grab the image shape, determine the center pixel.
        (H, W, D) = frame.shape
        uc = W//2
        vc = H//2
        
        x0 = -0.0315
        y0 = 0.62

        # Help to determine the HSV range...
        if True:
            # Draw the center lines.  Note the row is the first dimension.
            frame = cv2.line(frame, (uc,0), (uc,H-1), self.white, 1)
            frame = cv2.line(frame, (0,vc), (W-1,vc), self.white, 1)

            # Report the center HSV values.  Note the row comes first.
            #self.get_logger().info(
            #    "HSV = (%3d, %3d, %3d)" % tuple(hsv[vc, uc]))

        
        # Threshold in Hmin/max, Smin/max, Vmin/max
        binary = cv2.inRange(hsv, self.hsvlimits[:,0], self.hsvlimits[:,1])

        # Erode and Dilate. Definitely adjust the iterations!
        iter = 4
        binary = cv2.erode( binary, None, iterations=iter)
        binary = cv2.dilate(binary, None, iterations=2*iter)
        binary = cv2.erode( binary, None, iterations=iter)
        
        lst = []
        
        #circles = cv2.HoughCircles(binary, cv2.HOUGH_GRADIENT, 2, 20, param1=50, param2=30, minRadius=0, maxRadius=0)
        #if circles is not None:   
        #    circles = np.uint16(np.around(circles))
        #    for circ in circles[0, :]:
        #        cv2.circle(frame, (circ[0], circ[1]), circ[2], self.yellow, 2)
        #        cv2.circle(frame, (circ[0], circ[1]), 5,       self.red,   -1)
        #        center = self.pixelToWorld(frame, circ[0], circ[1], x0, y0, True)
        #        if center is not None:
        #            lst.append((center[0], center[1]))
        
        
        # Find contours in the mask and initialize the current
        # (x, y) center of the ball
        (contours, hierarchy) = cv2.findContours(
            binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
        # Draw all contours on the original image for debugging.
        cv2.drawContours(frame, contours, -1, self.blue, 2)

        # Only proceed if at least one contour was found.  You may
        # also want to loop over the contours...

        for cnt in contours:
            contour_area = cv2.contourArea(cnt)
            rect = cv2.minAreaRect(cnt)
            ((x, y), (w, h), angle) = rect
            
            if h < w:
                temp = w
                w = h
                h = temp
                angle -= 90
            angle *= np.pi/180
            ratio = w/h
            
            #if round(contour_area, -3) == 6000:
            #    oval = cv2.SimpleBlobDetector(cnt)
            #	#center = self.pixelToWorld(frame, ur, vr, x0, y0, True)          
            if ratio > 0.6:
                # Find the enclosing circle (convert to pixel values)
                ((ur, vr), radius) = cv2.minEnclosingCircle(cnt)
                ur     = int(ur)
                vr     = int(vr)
                radius = int(radius)

                # Draw the circle (yellow) and centroid (red) on the
                # original image.
                cv2.circle(frame, (ur, vr), int(radius), self.yellow,  2)
                cv2.circle(frame, (ur, vr), 5,           self.red,    -1)
                    
                center = self.pixelToWorld(frame, ur, vr, x0, y0, True)
                if center is not None:
                    lst.append((center[0], center[1]))
            else:
                box = cv2.boxPoints(rect)
                box = np.int0(box)
                cv2.drawContours(frame, [box], 0, self.green, 2)
                x_mid = np.uint16(x + h/4 * np.sin(angle))
                y_mid = np.uint16(y - h/4 * np.cos(angle))
                cv2.circle(frame, (x_mid, y_mid), 5, self.red,   -1)
                center = self.pixelToWorld(frame, x_mid, y_mid, x0, y0, True)
                lst.append((center[0], center[1]))
        self.stationary(lst)

        # Convert the frame back into a ROS image and republish.
        self.pubrgb.publish(self.bridge.cv2_to_imgmsg(frame, "rgb8"))

        # Also publish the binary (black/white) image.
        self.pubbin.publish(self.bridge.cv2_to_imgmsg(binary))


#
#   Main Code
#
def main(args=None):
    # Initialize ROS.
    rclpy.init(args=args)

    # Instantiate the detector node.
    node = DetectorNode('balldetector')

    # Spin the node until interrupted.
    rclpy.spin(node)

    # Shutdown the node and ROS.
    node.shutdown()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
