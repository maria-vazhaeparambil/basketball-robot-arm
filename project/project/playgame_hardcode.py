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

from project.TrajectoryUtils    import spline, goto5
from project.KinematicChain     import KinematicChain
from project.TransformHelpers   import ep
from enum import Enum
import math

from custom_msgs.msg  import ObjectArray
from custom_msgs.msg  import Object

#
#   Definitions
#
RATE = 100.0            # Hertz


#class Modes(Enum):
#   APPEND = 0
#   HALT = 1

class Spline():
    # Initialization connecting last command to next segment
    def __init__(self, tcmd, pcmd, vcmd, segment):
        # Save the initial time and duration.
        self.t0 = tcmd
        self.T = segment.Tmove
        # Pre-compute the parameters.
        p0 = np.array(pcmd)
        v0 = np.array(vcmd)
        pf = np.array(segment.pf)
        vf = np.array(segment.vf)
        T = self.T
        self.a = p0
        self.b = v0
        self.c = np.zeros_like(p0)
        self.d = + 10*(pf-p0)/T**3 - 6*v0/T**2 - 4*vf/T**2
        self.e = - 15*(pf-p0)/T**4 + 8*v0/T**3 + 7*vf/T**3
        self.f = + 6*(pf-p0)/T**5 - 3*v0/T**4 - 3*vf/T**4
        # Evaluation at any time (Shortening self to s).
    def evaluate(s, t):
        # Get the time relative to the start time.
        t = t - s.t0
        # Compute the current commands.
        p = s.a + s.b*t + s.c*t**2 + s.d*t**3 + s.e*t**4 + s.f*t**5
        v = s.b + 2*s.c*t + 3*s.d*t**2 + 4*s.e*t**3 + 5*s.f*t**4
        # Return as a list.
        return (p.tolist(),v.tolist())
    
class Segment:
    def __init__(self, pf, vf, Tmove):
        """
        Initializes a segment with final position, final velocity, and duration.
        
        :param pf: Final position as a list or numpy array
        :param vf: Final velocity as a list or numpy array
        :param Tmove: Duration of movement
        """
        self.pf = np.array(pf)
        self.vf = np.array(vf)
        self.Tmove = Tmove

#
#   DEMO Node Class
#
class DemoNode(Node):
    
    # Initialization.
    def __init__(self, name):
        # Initialize the node, naming it as specified
        super().__init__(name)

        self.traj_time = 5
        self.grip_time = 2

        self.waiting = [0, np.pi/2, np.pi/2, 0, 0.0]
        self.guess = [-np.pi/4, -1, np.pi/2]
        self.segments = [Segment(self.waiting, [0, 0, 0, 0, 0], self.traj_time)]
        self.segments.append(Segment([0, np.pi/3, np.pi/2, -np.pi/3, 0.0], [0, 0, 0, 0, 0], self.traj_time))
        # use scope to find last shift
        pick_up = -0.6
        self.segments.append(Segment([0, np.pi/3, np.pi/2, -np.pi/3, pick_up], [0, 0, 0, 0, 0], self.grip_time))
        self.segments.append(Segment([0, np.pi/2, np.pi/2, 0, pick_up], [0, 0, 0, 0, 0], self.traj_time))
        self.segments.append(Segment([0, np.pi/2, -np.pi/3, 0, pick_up], [0, 0, 0, 0, 0], self.traj_time))
        self.segments.append(Segment([0, np.pi/3, np.pi/4, 0, -0.30], [0, 0, 0, 0, 0], 0.5))
        self.spline = None
        self.abort = False
        
        self.amount_to_move = 0.03
        self.contact_amount = 0.015

        self.A = -0.14
        self.B = -1.6
 
        self.chain = KinematicChain(self, 'world', 'tip', ['base', 'shoulder', 'elbow', 'wrist'])
        
        (p, _, J, _) = self.chain.fkin([0, np.pi/2, 0, 0])
        print(p)
        
        # Create a temporary subscriber to grab the initial position.
        self.position0 = self.grabfbk()
        self.get_logger().info("Initial positions: %r" % self.position0)
        
        self.pcmd = self.position0
        self.vcmd = [0.0, 0.0, 0.0, 0.0, 0.0]

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
        
        # Create a subscriber to receive object messages.
        self.objssub = self.create_subscription(
            ObjectArray, '/objects', self.recvobjs, 10)

        # Report.
        self.get_logger().info("Running %s" % name)
        
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
        self.cmdmsg.name         = ['base', 'shoulder', 'elbow', 'wrist', 'spool']
        self.cmdmsg.position     = pos
        self.cmdmsg.velocity     = vel
        self.cmdmsg.effort       = eff
        self.cmdpub.publish(self.cmdmsg)

    def newton_raphson(self, finalp, theta_guess):
        theta_guess = np.array(theta_guess)
        theta = theta_guess
        gamma = 0.1
        alpha = 0.1
        (p, _, J, _) = self.chain.fkin(theta)
        J_inv = np.linalg.inv(J.T @ J + (gamma ** 2) * np.identity(3)) @ J.T
        iternum = 0
        while np.linalg.norm(finalp - p) > 0.001:
            delta_theta = J_inv @ (finalp - p)
            theta = theta + delta_theta 
            (p, _, J, _) = self.chain.fkin(theta)
            J_inv = np.linalg.inv(J.T @ J + (gamma ** 2) * np.identity(3)) @ J.T
            iternum += 1
            if iternum > 100:
                self.get_logger().info("Newton-Raphson failed to converge.")
                return None
        return theta
    
    # Receive a point message - called by incoming messages.
    def recvobjs(self, objsmsg):
        # Extract the data.
        objs = objsmsg.objects
        
        # Report.
        if len(self.segments) == 0:
            for obj in objs:
                self.get_logger().info("Adding object of type %r to queue." % (obj.type))
                if obj.type == 0:
                    pos = [obj.pose.position.x, obj.pose.position.y, obj.pose.position.z]
                    q = self.newton_raphson(pos, self.guess)
                    q.extend([0.0, 0.0])
                    if q is not None:
                        self.segments.append(Segment(q, [0, 0, 0, 0, 0], self.traj_time))
                        self.segments.append(Segment(self.waiting, [0, 0, 0, 0, 0], self.traj_time))
                else:
                    x, y, z = obj.pose.position.x, obj.pose.position.y,  obj.pose.position.z
                    qz, qw = obj.pose.orientation.z, obj.pose.orientation.w

                    # Calculate the 2D orientation angle (theta) from the quaternion
                    theta = 2 * math.atan2(qz, qw)

                    # Perpendicular direction vector
                    perp_dx = np.sin(theta)
                    perp_dy = np.cos(theta)

                    # Calculate midway pointsse
                    W_half = self.amount_to_move
                    P1 = np.array([x + W_half * perp_dx, y + W_half * perp_dy, 0.0])
                    P2 = np.array([x - W_half * perp_dx, y - W_half * perp_dy, 0.0])

                    q_center = self.newton_raphson([x, y, z], self.guess)

                    if q_center is not None:
                        q_p1 = self.newton_raphson(P1, q_center)
                        q_p2 = self.newton_raphson(P2, q_center)
                        q_center.extend([0.0, 0.0])
                        q_p2.extend([0.0, 0.0])
                        self.segments.append(Segment(q_center, [0, 0, 0, 0, 0], self.traj_time))
                        self.segments.append(Segment(q_p1, [0, 0, 0, 0, 0], self.grip_time))
                        self.segments.append(Segment(q_center, [0, 0, 0, 0, 0], self.grip_time))
                        self.segments.append(Segment(q_p2, [0, 0, 0, 0, 0], self.grip_time))
                        self.segments.append(Segment(q_center, [0, 0, 0, 0, 0], self.grip_time))
                        self.segments.append(Segment(self.waiting, [0, 0, 0, 0, 0], self.traj_time))
        
    ######################################################################
    # Handlers
    # Receive feedback - called repeatedly by incoming messages.
    def recvfbk(self, fbkmsg):
        # Save the actual position.
        self.actpos = fbkmsg.position
        
    # Timer (100Hz) update.
    def update(self):
        
        # Grab the current time.
        now = self.get_clock().now()
        t   = (now - self.starttime).nanoseconds * 1e-9
        
        if False:
            tau = self.gravity(self.actpos)
            self.sendcmd([], [], tau)
        else:    
            #if np.linalg.norm(np.array(self.pcmd) - self.actpos) > self.contact_amount:
                #self.get_logger().info("Pcmd: %r" % self.pcmd)
                #self.get_logger().info("Actpos: %r" % self.actpos)
            #    self.segments = [Segment(self.waiting, [0, 0, 0, 0, 0], self.traj_time)]
            
            # Cancel the current spline if it has run past time or an abort has
            # been requested. If there is no spline, just clear the abort flag.
            if self.spline and (t - self.spline.t0 > self.spline.T) or self.abort:
                self.spline = None
                self.abort = False
            # Without a current spline but a waiting segment, pop and start it.
            if not self.spline and len(self.segments) > 0:
                seg = self.segments.pop(0)
                self.spline = Spline(t, self.pcmd, self.vcmd, seg)
            # If there is a spline, compute it. Else just hold.
            if self.spline:
                (self.pcmd, self.vcmd) = self.spline.evaluate(t)
            else:
                (self.pcmd, self.vcmd) = (self.pcmd, [0.0 for _ in self.vcmd])
            tau = self.gravity(self.actpos)
            self.sendcmd(self.pcmd, self.vcmd, tau)
              
                    
    def gravity(self, pos):
        tau_shoulder = self.A * math.sin(pos[1]) + self.B * math.cos(pos[1])
        return [0.0, tau_shoulder, 0.0, 0.0, 0.0]
                    
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
