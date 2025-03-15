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
    def __init__(self, tcmd, pcmd, vcmd, segment, throw):
        # Save the initial time and duration.
        self.throw = throw
        self.t0 = tcmd
        self.T = segment.Tmove
        # Pre-compute the parameters.
        p0 = np.array(pcmd)
        v0 = np.array(vcmd)
        pf = np.array(segment.pf)
        vf = np.array(segment.vf)
        T = self.T
        
        p1 = pf
        if self.throw == 1:
            T *= 0.05
            vf = (pf - p0)/self.T * 1/0.95
            p1 = p0 + vf/2 * T
        self.p1 = p1 
        self.vf = vf
        
        self.a = p0
        self.b = v0
        self.c = np.zeros_like(p0)
        self.d = + 10*(p1-p0)/T**3 - 6*v0/T**2 - 4*vf/T**2
        self.e = - 15*(p1-p0)/T**4 + 8*v0/T**3 + 7*vf/T**3
        self.f = + 6*(p1-p0)/T**5 - 3*v0/T**4 - 3*vf/T**4
        
        p2 = self.p1 + self.T * 0.9 * vf
        self.a2 = p2
        self.b2 = vf
        self.c2 = np.zeros_like(p0)
        self.d2 = + 10*(pf-p2)/T**3 - 6*vf/T**2
        self.e2 = - 15*(pf-p2)/T**4 + 8*vf/T**3
        self.f2 = + 6*(pf-p2)/T**5 - 3*vf/T**4
        
        # Evaluation at any time (Shortening self to s).
    def evaluate(s, t):
        # Get the time relative to the start time.
        t = t - s.t0
        
        if s.throw != 1:
            # Compute the current commands.
            p = s.a + s.b*t + s.c*t**2 + s.d*t**3 + s.e*t**4 + s.f*t**5
            v = s.b + 2*s.c*t + 3*s.d*t**2 + 4*s.e*t**3 + 5*s.f*t**4
        else:
            if t < s.T * 0.05:
                p = s.a + s.b*t + s.c*t**2 + s.d*t**3 + s.e*t**4 + s.f*t**5
                v = s.b + 2*s.c*t + 3*s.d*t**2 + 4*s.e*t**3 + 5*s.f*t**4
            elif t < s.T * 0.95:
                p = s.p1 + (t - s.T * 0.05) * s.vf
                v = s.vf
            else:
                t2 = t - 0.95 * s.T
                p = s.a2 + s.b2*t2 + s.c2*t2**2 + s.d2*t2**3 + s.e2*t2**4 + s.f2*t2**5
                v = s.b2 + 2*s.c2*t2 + 3*s.d2*t2**2 + 4*s.e2*t2**3 + 5*s.f2*t2**4
        # Return as a list.
        return (p.tolist(),v.tolist())
            
    
class Segment:
    def __init__(self, pf, vf, Tmove, throw):
        """
        Initializes a segment with final position, final velocity, and duration.
        
        :param pf: Final position as a list or numpy array
        :param vf: Final velocity as a list or numpy array
        :param Tmove: Duration of movement
        """
        self.pf = np.array(pf)
        self.vf = np.array(vf)
        self.Tmove = Tmove
        self.throw = throw

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
        self.actpos = self.position0.copy()
        
        self.traj_time = 5
        self.grip_time = 2

        self.waiting = [0, np.pi/2, np.pi/2, 0, 0]
        self.guess = [0, np.pi/3, np.pi/4, -7 * np.pi/12]
        self.segments = [Segment(self.waiting, [0, 0, 0, 0, 0], self.traj_time, 0)]
        
        self.backboard = [0.0255, 0, 0]
        
        self.val = 0
        
        # use scope to find last shift
        self.pick_up = -0.6
        
        self.spline = None
        self.abort = False
        
        self.amount_to_move = 0.03
        self.contact_amount = 0.015

        self.A = 1.5
        self.B = 7.1
        self.C = -0.10
        self.D = -1.85
 
        self.chain = KinematicChain(self, 'world', 'tip', ['base', 'shoulder', 'elbow', 'wrist'])
        
        (p, _, J, _) = self.chain.fkin([0, np.pi/2, 0, 0])
         
        self.load = list(self.newton_raphson_angle([0.01906, 0.0, 0.35], self.guess, False))
        self.load.append(self.pick_up)
        self.release = list(self.newton_raphson_angle([0.01906, 0.49, 0.53], self.guess, False))
        self.release.append(self.pick_up)
        
        self.count = 0
        self.ending = False
        self.threshold = 10
        
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
            
        self.targsub = self.create_subscription(
            Object, '/targets', self.recvtarg, 10)

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
        (p, _, J, _) = self.chain.fkin(theta)
        J_inv = np.linalg.inv(J.T @ J + (gamma ** 2) * np.identity(len(J[0]))) @ J.T
        iternum = 0
        while np.linalg.norm(finalp - p) > 0.001:
            delta_theta = J_inv @ (finalp - p)
            theta = theta + delta_theta 
            (p, _, J, _) = self.chain.fkin(theta)
            J_inv = np.linalg.inv(J.T @ J + (gamma ** 2) * np.identity(len(J[0]))) @ J.T
            iternum += 1
            if iternum > 100:
                self.get_logger().info("Newton-Raphson failed to converge.")
                return None
        return theta
    
    def newton_raphson_angle(self, finalp, theta_guess, down):
        theta_guess = np.array(theta_guess)
        theta = theta_guess
        gamma = 0.1
        (p, _, J, _) = self.chain.fkin(theta)
        J_inv = np.linalg.inv(J.T @ J + (gamma ** 2) * np.identity(len(J[0]))) @ J.T
        iternum = 0
        while np.linalg.norm(finalp - p) > 0.001:
            delta_theta = J_inv @ (finalp - p)
            theta = theta + delta_theta 
            if down:
                theta[3] = -np.pi/2 - theta[1] + theta[2]
            else:
                theta[3] = 0
            (p, _, J, _) = self.chain.fkin(theta)
            J_inv = np.linalg.inv(J.T @ J + (gamma ** 2) * np.identity(len(J[0]))) @ J.T
            iternum += 1
            if iternum > 100:
                self.get_logger().info("Newton-Raphson failed to converge.")
                return None
        return theta
    
    # Receive a message - called by incoming messages.
    def recvobjs(self, objsmsg):
        # Extract the data.
        objs = objsmsg.objects
        
        # Report.
        if not self.spline and len(self.segments) == 0:
            for obj in objs:
                self.get_logger().info("Adding object of type %r to queue: (%r, %r)." % (obj.type, obj.pose.position.x, obj.pose.position.y))
                if obj.type == 0 and obj.pose.position.y < 0.6:
                    pos = [obj.pose.position.x, obj.pose.position.y, obj.pose.position.z]
                    if obj.pose.position.x > 0.18:
                        pos[0] = 0.18
                    elif obj.pose.position.x < -0.204:
                        pos[0] = -0.204
                    if obj.pose.position.y < 0.327:
                        pos[1] = 0.327
                    q = self.newton_raphson_angle(pos, self.guess, True)
                    if q is not None:
                        q = list(q)
                        q.append(0)
                        self.segments.append(Segment(q, [0, 0, 0, 0, 0], self.traj_time, 0))
                        q[-1] = self.pick_up
                        self.segments.append(Segment(q, [0, 0, 0, 0, 0], self.grip_time, 0))
                        
                        # throw 1
                        #up = [0, np.pi/2, np.pi/2, 0, -0.6]
                        #self.segments.append(Segment(up, [0, 0, 0, 0, 0], self.traj_time, 2))
                        #self.segments.append(Segment(self.load, [0, 0, 0, 0, 0], self.traj_time, 2))
                        #time = 1
                        #self.segments.append(Segment(self.release,[0, 0, 0, 0, 0] , time, 1)) 
                        
                        # 0.0196
                        base_angle = np.arctan((self.backboard[0] - 0.0255)/1.4169)
                        self.get_logger().info("backboard: %r" % (self.backboard[0]))
                        self.get_logger().info("base angle: %r" % (base_angle))
  
                        
                        self.segments.append(Segment([-base_angle, np.pi/2, -np.pi/3, 0, self.pick_up], [0, 0, 0, 0, 0], self.traj_time, 2))
                        self.segments.append(Segment([-base_angle, np.pi/3, np.pi/4, 0, -0.33], [0, 0, 0, 0, 0], 0.5, 3))          
                        
    # Receive a message - called by incoming messages.
    def recvtarg(self, objsmsg):
    
        # Extract the data.
        obj = objsmsg
        if not self.spline and len(self.segments) == 0:
            if obj.type == 1:
                self.backboard = [obj.pose.position.x, obj.pose.position.y, obj.pose.position.z]
            
        
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
        
        if self.count == self.threshold and not self.spline and self.ending == False:
            end = [0, np.pi/5, np.pi/2 + np.pi/5, np.pi/2, 0]
            self.spline = Spline(t, self.pcmd, self.vcmd, Segment(end, [0, 0, 0, 0, 0], self.traj_time, 0), 0)
            self.ending = True
        elif self.count == self.threshold and not self.spline and self.ending:
            exit()
        if False:
            tau = self.gravity(self.actpos)
            self.sendcmd([], [], tau)
        else:    
            #if np.linalg.norm(np.array(self.pcmd[:3]) - self.actpos[:3]) > self.contact_amount:
            #    #self.get_logger().info("Pcmd: %r" % self.pcmd)
            #    #self.get_logger().info("Actpos: %r" % self.actpos)
            #    self.segments = [Segment(self.waiting, [0, 0, 0, 0, 0], self.traj_time, 0)]
            
            # Cancel the current spline if it has run past time or an abort has
            # been requested. If there is no spline, just clear the abort flag.
            if self.spline and (t - self.spline.t0 > self.spline.T) or self.abort:
                self.spline = None
                self.abort = False
            # Without a current spline but a waiting segment, pop and start it.
            if not self.spline and len(self.segments) > 0:
                seg = self.segments.pop(0)
                if seg.throw == 2 and np.abs(self.pcmd[4] - self.actpos[4]) < 0.15:
                    self.segments = [Segment(self.waiting, [0, 0, 0, 0, 0], self.traj_time, 0)]
                else:
                    if seg.throw == 3:
                        self.count += 1
                    self.spline = Spline(t, self.pcmd, self.vcmd, seg, seg.throw)                 
            #elif not self.spline and len(self.segments) == 0:
            #    self.segments = [Segment(self.waiting, [0, 0, 0, 0, 0], self.traj_time, 0)]
            # If there is a spline, compute it. Else just hold.
            if self.spline:
                if self.spline.throw == 2 and np.abs(self.pcmd[4] - self.actpos[4]) < 0.15:
                    self.spline = Spline(t, self.pcmd, self.vcmd, Segment(self.waiting, [0, 0, 0, 0, 0], self.traj_time, 0), 0)
                    self.segments = []
                else:
                    (self.pcmd, self.vcmd) = self.spline.evaluate(t)
                    #if self.spline.throw == 1 and t - self.spline.t0 > self.spline.T*8/10:
                    #    self.pcmd[-1] = 0.0
        
            else:
                (self.pcmd, self.vcmd) = (self.pcmd, [0.0 for _ in self.vcmd])
            tau = self.gravity(self.actpos)
            self.sendcmd(self.pcmd, self.vcmd, tau)
              
                    
    def gravity(self, pos):
        if self.val < 1:
            self.val += 0.01
        tau_shoulder = self.val * (self.A * math.sin(pos[1]) + self.B * math.cos(pos[1]))
        tau_elbow = self.val * (self.C * math.sin(pos[1] - pos[2]) + self.D * math.cos(pos[1] - pos[2]))
        return [0.0, tau_shoulder, tau_elbow, 0.0, 0.0]
                    
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
