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

from threedof.TrajectoryUtils    import spline
from threedof.KinematicChain     import KinematicChain
from threedof.TransformHelpers     import *


#
#   Definitions
#
RATE = 100.0            # Hertz


#
#   DEMO Node Class
#
class DemoNode(Node):
    # Initialization.
    def __init__(self, name):
        # Initialize the node, naming it as specified
        super().__init__(name)

        # Create a temporary subscriber to grab the initial position.
        self.position0 = self.grabfbk()
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
        self.lam = 20
        self.curr_qd = None

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

        T1 = 5
        start = self.position0
        up = [0.0, -np.pi/2, 0.0]
        
        T2 = 7.5
        waiting = [-np.pi/2, -np.pi/2, np.pi/2]
        
        T3 = 10
        T4 = 15
        T5 = 17.5
        T6 = 22.5
        
        if t < T1:
        
            # Compute the trajectory.
            qd, qddot = self.smooth_move(t, T1, start, up)
            tau   = [0.0, 0.0, 0.0]

            # To test the gravity compensation:
            #qd    = []
            #qddot = []
            #tau   = [0.0, 0.0, 0.0].0

            # Send.
            self.sendcmd(qd, qddot, tau)
        elif t < T2:
            qd, qddot = self.smooth_move(t - T1, T2 - T1, up, waiting)
            tau   = [0.0, 0.0, 0.0]
            self.curr_qd = waiting
            self.sendcmd(qd, qddot, tau)
        elif t < T3:
            qd = list(self.curr_qd)
            qddot = [0.0, 0.0, 0.0]
            tau   = [0.0, 0.0, 0.0]
            self.sendcmd(qd, qddot, tau)
        elif t < T4:
            
            dt = 1/RATE
            (startp, _, _, _) = self.chain.fkin(waiting)
            finalp = np.array([-0.5161, 0.16, 0.0])
            
            xd, xv = spline(t - T3, T4 - T3, startp[0], finalp[0], 0, 0)
            yd, yv = spline(t - T3, T4 - T3, startp[1], finalp[1], 0, 0)
            zd, zv = spline(t - T3, T4 - T3, startp[2], finalp[2], 0, 0)
           
            pd = [xd, yd, zd]
            vd = [xv, yv, zv]
            
            qdlast = self.curr_qd
            pdlast = pd
            
            (p, _, Jv, _) = self.chain.fkin(qdlast)

            vr = vd + self.lam * ep(pdlast, p)
            J = Jv
            xrdot = vr
            qddot = np.linalg.inv(J) @ xrdot
            
            qd = qdlast + dt * qddot
            self.curr_qd = qd

            tau   = [0.0, 0.0, 0.0]
            self.sendcmd(list(qd), list(qddot), tau)
        elif t < T5:
            qd = list(self.curr_qd)
            qddot = [0.0, 0.0, 0.0]
            tau   = [0.0, 0.0, 0.0]
            self.sendcmd(qd, qddot, tau)
        elif t < T6:
            qd, qddot = self.smooth_move(t - T5, T6 - T5, self.curr_qd, waiting)
            tau   = [0.0, 0.0, 0.0]
            self.sendcmd(qd, qddot, tau)
            
            


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
