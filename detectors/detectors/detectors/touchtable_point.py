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
from threedof.TransformHelpers     import *
from enum import Enum
import math

from geometry_msgs.msg          import Point

#
#   Definitions
#
RATE = 100.0            # Hertz


class Modes(Enum):
   WAITING = 0
   TOUCHINGTABLE = 1
   
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

        
        # Create a temporary subscriber to grab the initial position.
        self.position0 = self.grabfbk()
        print(self.position0)
        self.get_logger().info("Initial positions: %r" % self.position0)

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
                               
        self.chain = KinematicChain(self, 'world', 'tip', ['base', 'shoulder', 'elbow'])
        self.lam = 5
        self.curr_qd = self.position0.copy()
        
        # Create a subscriber to receive point messages.
        self.pointsub = self.create_subscription(
            Point, '/point', self.recvpoint, 10)

        # Report.
        self.get_logger().info("Running %s" % name)
        
        self.status = Modes.WAITING
        
        self.inputx = None
        self.inputy = None
        self.inputz = None
        self.point_sent = None
        self.contact = False
        
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
        if self.status == Modes.WAITING and t > self.traj_time + self.wait_time * 2:
            self.get_logger().info("Running point %r, %r, %r" % (x,y,z))
            self.inputx = x
            self.inputy = y
            self.inputz = z
            self.point_sent = t
            self.status = Modes.TOUCHINGTABLE
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

    # Timer (100Hz) update.
    def update(self):
        
        # Grab the current time.
        now = self.get_clock().now()
        t   = (now - self.starttime).nanoseconds * 1e-9
         
        traj_time = self.traj_time
        wait_time = self.wait_time
        
        T1 = traj_time
        start = self.position0
        up = [0.0, -np.pi/2, 0.0]
        
        T2 = traj_time * 2
        waiting = [-np.pi/2, -np.pi/2, np.pi/2]
        
        T3 = traj_time * 2+ wait_time
        
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
            
        if t > T3:
        
            if self.status == Modes.TOUCHINGTABLE and self.contact == False:
                if t - self.point_sent < traj_time:
                    dt = 1/RATE
                    (startp, _, _, _) = self.chain.fkin(waiting)
                    finalp = np.array([self.inputx, self.inputy, self.inputz])
                    
                    xd, xv = goto5(t - self.point_sent, traj_time, startp[0], finalp[0])
                    yd, yv = goto5(t - self.point_sent, traj_time, startp[1], finalp[1])
                    zd, zv = goto5(t - self.point_sent, traj_time, startp[2], finalp[2])
                    
                    pd = [xd, yd, zd]
                    vd = [xv, yv, zv]
                    
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
                    
                    actual = self.chain.fkin(self.actpos)[0]
                    command = np.array(pd)
                    if np.linalg.norm(actual - command) > 0.02:
                        self.contact = True
                        now = self.get_clock().now()
                        self.point_sent = (now - self.starttime).nanoseconds * 1e-9
                    
                    tau   = self.gravity(self.actpos)
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
            elif self.status == Modes.TOUCHINGTABLE:
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
