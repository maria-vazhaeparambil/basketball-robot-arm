#!/usr/bin/env python3
#
#   touchtable.py
#
#   Demonstration node to interact with the HEBIs.
#
import numpy as np
import rclpy

from rclpy.node         import Node
from sensor_msgs.msg    import JointState

from threedof.TrajectoryUtils    import spline, goto5
from threedof.KinematicChain     import KinematicChain
from threedof.TransformHelpers   import ep
from enum import Enum
import math

from geometry_msgs.msg  import Point
from geometry_msgs.msg  import Pose

#
#   Definitions
#
RATE = 100.0            # Hertz


class Modes(Enum):
   WAITING = 0
   TOUCHINGDISC = 1
   TAPPINGSTRIP = 2
   
#
#   DEMO Node Class
#
class DemoNode(Node):
    
    # Initialization.
    def __init__(self, name):
        # Initialize the node, naming it as specified
        super().__init__(name)

        self.traj_time = 5
        self.wait_time = 0.5
       
        self.A = -0.14
        self.B = -1.6
 
        self.chain = KinematicChain(self, 'world', 'tip', ['base', 'shoulder', 'elbow'])
        
        self.waiting = [-np.pi/2, -np.pi/2, np.pi/2]
        (self.waitp, _, _, _) = self.chain.fkin(self.waiting)
        
        # Create a temporary subscriber to grab the initial position.
        self.position0 = self.grabfbk()
        self.get_logger().info("Initial positions: %r" % self.position0)

        self.lam = 1
        self.curr_qd = self.position0.copy()
        self.pose_qd = None
        self.touch_qd = None
        
        # Create a message and publisher to send the joint commands.
        self.cmdmsg = JointState()
        self.cmdpub = self.create_publisher(JointState, '/joint_commands', 10)

        # Wait for a connection to happen.  This isn't necessary, but
        # means we don't start until the rest of the system is ready.
        self.get_logger().info("Waiting for a /joint_commands subscriber...")
        while(not self.count_subscribers('/joint_commands')):
            pass

        # Create a subscriber to continually receive joint state messages.
        self.actpos = self.position0.copy()
        self.fbksub = self.create_subscription(
            JointState, '/joint_states', self.recvfbk, 10)

        # Create a timer to keep calculating/sending commands.
        rate           = RATE
        self.starttime = self.get_clock().now()
        self.timer     = self.create_timer(1/rate, self.update)
        self.get_logger().info("Sending commands with dt of %f seconds (%fHz)" %
                               (self.timer.timer_period_ns * 1e-9, rate))
        
        # Create a subscriber to receive point messages.
        self.pointsub = self.create_subscription(
            Point, '/point', self.recvpoint, 10)
        self.posesub = self.create_subscription(
            Pose, '/pose', self.recvpose, 10)

        # Report.
        self.get_logger().info("Running %s" % name)
        
        self.status = Modes.WAITING
        
        self.amount_to_move = 0.03

        self.inputx = None
        self.inputy = None
        self.inputz = None
        self.qx = None
        self.qy = None
        self.qz = None
        self.qw = None
        self.point_sent = None
        self.contact = False
        self.contact_amount = 0.05
        
    # Shutdown
    def shutdown(self):
        # No particular cleanup, just shut down the node.
        self.destroy_node()


    # Grab a single feedback - DO NOT CALL THIS REPEATEDLY!
    def grabfbk(self):
        # Create a temporary handler to grab the position.
        def cb(fbkmsg):
            self.grabpos   = list(fbkmsg.position)
            self.grabready = True

        # Temporarily subscribe to get just one message.
        sub = self.create_subscription(JointState, '/joint_states', cb, 1)
        self.grabready = False
        while not self.grabready:
            rclpy.spin_once(self)
        self.destroy_subscription(sub)

        # Return the values.
        return self.grabpos

    # Send a command.
    def sendcmd(self, pos, vel, eff = []):
        # Build up the message and publish.
        self.cmdmsg.header.stamp = self.get_clock().now().to_msg()
        self.cmdmsg.name         = ['base', 'shoulder', 'elbow']
        self.cmdmsg.position     = pos
        self.cmdmsg.velocity     = vel
        self.cmdmsg.effort       = eff
        self.cmdpub.publish(self.cmdmsg)

    # Receive a point message - called by incoming messages.
    def recvpoint(self, pointmsg):
        # Extract the data.
        x = pointmsg.x
        y = pointmsg.y
        z = pointmsg.z
        
        now = self.get_clock().now()
        t = (now - self.starttime).nanoseconds * 1e-9
        # Report.
        if self.status == Modes.WAITING and t > self.traj_time * 2 + self.wait_time:
            self.get_logger().info("Running disc %r, %r, %r" % (x,y,z))
            self.inputx = x
            self.inputy = y
            self.inputz = z
            self.point_sent = t
            self.status = Modes.TOUCHINGDISC
        #else:
           #self.get_logger().info("Already running point %r, %r, %r" % (self.inputx,self.inputy,self.inputz))

    # Receive a point message - called by incoming messages.
    def recvpose(self, posemsg):
        # Extract the data.
        px = posemsg.position.x
        py = posemsg.position.y
        pz = posemsg.position.z

        qx = posemsg.orientation.x
        qy = posemsg.orientation.y
        qz = posemsg.orientation.z
        qw = posemsg.orientation.w
        
        theta = 2 * math.atan2(qz, qw)
        
        now = self.get_clock().now()
        t = (now - self.starttime).nanoseconds * 1e-9
        # Report.
        if self.status == Modes.WAITING and t > self.traj_time * 2 + self.wait_time:
            self.get_logger().info("Running strip %r, %r, %r at angle %r" % (px,py,pz,theta))
            self.inputx = px
            self.inputy = py
            self.inputz = pz
            self.qx = qx
            self.qy = qy
            self.qz = qz
            self.qw = qw
            self.point_sent = t
            self.status = Modes.TAPPINGSTRIP
        #else:
           #self.get_logger().info("Already running point %r, %r, %r" % (self.inputx,self.inputy,self.inputz))
        
    ######################################################################
    # Handlers
    # Receive feedback - called repeatedly by incoming messages.
    def recvfbk(self, fbkmsg):
        # Save the actual position.
        self.actpos = fbkmsg.position
        
    def smooth_move(self, t, T, start, end):
        (xp, xv) = spline(t, T, start[0], end[0], 0, 0)
        (yp, yv) = spline(t, T, start[1], end[1], 0, 0)
        (zp, zv) = spline(t, T, start[2], end[2], 0, 0)
        return [xp, yp, zp], [xv, yv, zv]

    def ikin(self, t, shift, total_time, startp, finalp):
        dt = 1/RATE
        
        pd, vd = goto5(t - self.point_sent - shift, total_time, startp, finalp)
        
        qdlast = self.curr_qd
        pdlast = pd
        
        (p, _, Jv, _) = self.chain.fkin(qdlast)
        vr = vd + self.lam * ep(pdlast, p)
        J = Jv
        xrdot = vr
        gamma = 0.1
        J_inv = np.linalg.inv(J.T @ J + (gamma ** 2) * np.identity(3)) @ J.T
        qddot = J_inv @ xrdot
        qd = qdlast + dt * qddot
        self.curr_qd = qd
        
        if np.linalg.norm(np.array(qdlast) - self.actpos) > self.contact_amount:
            self.contact = True
            now = self.get_clock().now()
            self.point_sent = (now - self.starttime).nanoseconds * 1e-9
        
        tau   = self.gravity(self.actpos)
        return qd, qddot, tau
    
    def newton_raphson(self, startp, finalp, theta_guess):
        theta = theta_guess
        qdlast = self.waiting
        gamma = 0.1
        (p, _, J, _) = self.chain.fkin(qdlast)
        J_inv = np.linalg.inv(J.T @ J + (gamma ** 2) * np.identity(3)) @ J.T
        iternum = 0
        while np.linalg.norm(finalp - p) > 0.001:
            theta = theta + J_inv @ (finalp - p)
            (p, _, J, _) = self.chain.fkin(theta)
            J_inv = np.linalg.inv(J.T @ J + (gamma ** 2) * np.identity(3)) @ J.T
            iternum += 1
            if iternum > 100:
                self.get_logger().info("Newton-Raphson failed to converge.")
                raise Exception()
        return theta
        
    # Timer (100Hz) update.
    def update(self):
        
        # Grab the current time.
        now = self.get_clock().now()
        t   = (now - self.starttime).nanoseconds * 1e-9
         
        traj_time = self.traj_time
        wait_time = self.wait_time
        tap_time = 2
        
        T1 = traj_time
        start = self.position0
        up = [0.0, -np.pi/2, 0.0]
        
        T2 = traj_time * 2
        waiting = self.waiting
        
        T3 = traj_time * 2 + wait_time
        
        #if True:
        #    qd = []
        #    qddot = []
        #    tau   = self.gravity(self.actpos)
        #    self.sendcmd(qd, qddot, tau) 
        if t < T1:
            # Compute the trajectory.
            qd, qddot = self.smooth_move(t, T1, start, up)
            tau   = self.gravity(self.actpos)
            self.sendcmd(qd, qddot, tau)
        elif t < T2:
            qd, qddot = self.smooth_move(t - T1, T2 - T1, up, waiting)
            tau   = self.gravity(self.actpos)
            self.curr_qd = waiting
            self.sendcmd(qd, qddot, tau)
        elif t < T3:
            qd = list(self.curr_qd)
            qddot = [0.0, 0.0, 0.0]
            tau   = self.gravity(self.actpos)
            self.sendcmd(qd, qddot, tau) 
        elif t > T3:
            if self.status == Modes.TOUCHINGDISC and self.contact == False:
                if t - self.point_sent < traj_time:
                    startp = self.waitp
                    finalp = np.array([self.inputx, self.inputy, self.inputz])
                    qd, qddot, tau = self.ikin(t, 0.0, traj_time, startp, finalp)
                    self.sendcmd(list(qd), list(qddot), tau)
                elif t - self.point_sent < traj_time + wait_time:
                    qd = list(self.curr_qd)
                    qddot = [0.0, 0.0, 0.0]
                    tau   = self.gravity(self.actpos)
                    self.sendcmd(qd, qddot, tau)
                elif t - self.point_sent < traj_time * 2 + wait_time:
                    qd, qddot = self.smooth_move(t - self.point_sent - traj_time - wait_time, traj_time, self.curr_qd, waiting)
                    tau   = self.gravity(self.actpos)
                    self.sendcmd(qd, qddot, tau)
                elif t - self.point_sent < traj_time * 2 + wait_time * 2:
                    qd = waiting
                    qddot = [0.0, 0.0, 0.0]
                    tau   = self.gravity(self.actpos)
                    self.curr_qd = waiting
                    self.sendcmd(qd, qddot, tau)
                else:
                    self.status = Modes.WAITING
                    self.contact = False
            elif self.status == Modes.TAPPINGSTRIP and self.contact == False:
                x, y = self.inputx, self.inputy
                qz, qw = self.qz, self.qw

                # Calculate the 2D orientation angle (theta) from the quaternion
                theta = 2 * math.atan2(qz, qw)

                # Perpendicular direction vector
                perp_dx = np.sin(theta)
                perp_dy = np.cos(theta)

                # Calculate midway points
                W_half = self.amount_to_move
                P1 = np.array([x + W_half * perp_dx, y + W_half * perp_dy, 0.0])
                P2 = np.array([x - W_half * perp_dx, y - W_half * perp_dy, 0.0])

                if t - self.point_sent < traj_time:
                    startp = self.waitp
                    finalp = np.array([self.inputx, self.inputy, self.inputz])
                    qd, qddot, tau = self.ikin(t, 0.0, traj_time, startp, finalp)
                    self.sendcmd(list(qd), list(qddot), tau)
                elif t - self.point_sent < traj_time + wait_time:
                    qd = list(self.curr_qd)
                    qddot = [0.0, 0.0, 0.0]
                    tau   = self.gravity(self.actpos)
                    self.pose_qd = qd
                    self.sendcmd(qd, qddot, tau)
                elif t - self.point_sent < traj_time + wait_time + tap_time:
                    startp = np.array([self.inputx, self.inputy, self.inputz])
                    finalp = P1
                    qd, qddot, tau = self.ikin(t, traj_time + wait_time, tap_time, startp, finalp)
                    self.sendcmd(list(qd), list(qddot), tau)
                elif t - self.point_sent < traj_time + wait_time * 2 + tap_time:
                    qd = list(self.curr_qd)
                    qddot = [0.0, 0.0, 0.0]
                    tau   = self.gravity(self.actpos)
                    self.touch_qd = qd
                    self.sendcmd(qd, qddot, tau)
                elif t - self.point_sent < traj_time + wait_time * 2 + tap_time * 2:
                    qd, qddot = self.smooth_move(t - self.point_sent - traj_time - wait_time * 2 - tap_time, tap_time, self.touch_qd, self.pose_qd)
                    tau   = self.gravity(self.actpos)
                    self.curr_qd = qd
                    self.sendcmd(qd, qddot, tau)
                elif t - self.point_sent < traj_time + wait_time * 3 + tap_time * 2:
                    qd = list(self.curr_qd)
                    qddot = [0.0, 0.0, 0.0]
                    tau   = self.gravity(self.actpos)
                    self.sendcmd(qd, qddot, tau)
                elif t - self.point_sent < traj_time + wait_time * 3 + tap_time * 3:
                    startp = np.array([self.inputx, self.inputy, self.inputz])
                    finalp = P2
                    qd, qddot, tau = self.ikin(t, traj_time + wait_time * 3 + tap_time * 2, tap_time, startp, finalp)
                    self.sendcmd(list(qd), list(qddot), tau)
                elif t - self.point_sent < traj_time + wait_time * 4 + tap_time * 3:
                    qd = list(self.curr_qd)
                    qddot = [0.0, 0.0, 0.0]
                    tau   = self.gravity(self.actpos)
                    self.touch_qd = qd
                    self.sendcmd(qd, qddot, tau)
                elif t - self.point_sent < traj_time + wait_time * 4 + tap_time * 4:
                    qd, qddot = self.smooth_move(t - self.point_sent - traj_time - wait_time * 4 - tap_time * 3, tap_time, self.touch_qd, self.pose_qd)
                    tau   = self.gravity(self.actpos)
                    self.curr_qd = qd
                    self.sendcmd(qd, qddot, tau)
                elif t - self.point_sent < traj_time + wait_time * 5 + tap_time * 4:
                    qd = list(self.curr_qd)
                    qddot = [0.0, 0.0, 0.0]
                    tau   = self.gravity(self.actpos)
                    self.sendcmd(qd, qddot, tau)
                elif t - self.point_sent < traj_time * 2 + wait_time * 5 + tap_time * 4:
                    qd, qddot = self.smooth_move(t - self.point_sent - traj_time - wait_time * 5 - tap_time * 4, traj_time, self.pose_qd, waiting)
                    tau   = self.gravity(self.actpos)
                    self.sendcmd(qd, qddot, tau)
                elif t - self.point_sent < traj_time * 2 + wait_time * 6 + tap_time * 4:
                    qd = waiting
                    qddot = [0.0, 0.0, 0.0]
                    tau   = self.gravity(self.actpos)
                    self.curr_qd = waiting
                    self.sendcmd(qd, qddot, tau)
                else:
                    self.status = Modes.WAITING
                    self.contact = False
               
            elif self.status != Modes.WAITING:
                if t - self.point_sent < wait_time:
                    qd = list(self.curr_qd)
                    qddot = [0.0, 0.0, 0.0]
                    tau   = self.gravity(self.actpos)
                    self.sendcmd(qd, qddot, tau)
                elif t - self.point_sent < traj_time + wait_time:
                    qd, qddot = self.smooth_move(t - self.point_sent - wait_time, traj_time, self.curr_qd, waiting)
                    tau   = self.gravity(self.actpos)
                    self.sendcmd(qd, qddot, tau)
                elif t - self.point_sent < traj_time + wait_time * 2:
                    qd = waiting
                    qddot = [0.0, 0.0, 0.0]
                    tau   = self.gravity(self.actpos)
                    self.curr_qd = waiting
                    self.sendcmd(qd, qddot, tau)
                else:
                    self.status = Modes.WAITING
                    self.contact = False
            else:
                qd = self.curr_qd
                qddot = [0.0, 0.0, 0.0]
                tau = self.gravity(self.actpos)
                self.sendcmd(qd, qddot, tau)
              
                    
    def gravity(self, pos):
        tau_shoulder = self.A * math.sin(pos[1]) + self.B * math.cos(pos[1])
        return [0.0, tau_shoulder, 0.0]
                    
#
#   Main Code
#
def main(args=None):
    # Initialize ROS.
    rclpy.init(args=args)

    # Instantiate the DEMO node.
    node = DemoNode('demo')

    # Spin the node until interrupted.
    rclpy.spin(node)

    # Shutdown the node and ROS.
    node.shutdown()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
