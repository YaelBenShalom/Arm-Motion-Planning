#!/usr/bin/env python3

"""
 The trajectory node publishes a custom velocity message into a geometry_msgs/Twist
 and publish to the turtlebot cmd_vel in order to draw the figure eight

PUBLISHERS:
  cmd_vel (geometry_msgs/Twist) - the linear and angular velocity of the turtlebot
  path (nav_msgs/Path) - the path (x,y) of the turtlebot

SERVICES:   
  pause (Empty) - stop the turtle's motion, in a way that the trajectory can be
                    resumed
  resume (Empty) - resume the turtle's motion along the trajectory

PARAMETERS:
  width - The width of the figure eight
  height - The height of the figure eight
  period - The amount of time it takes to complete the figure eight
  ~pub_freq - The frequency at which to publish the messages (a private parameter)
"""

import rospy
import geometry_msgs.msg
from geometry_msgs.msg import Twist, Pose, Point, PoseStamped, Quaternion, Vector3
from nav_msgs.msg import Path
from std_srvs.srv import Empty, EmptyResponse
import sympy
from sympy.abc import t
from sympy import symbols, Eq, Function, cos, sin, atan2, pi
import tf
import tf2_ros
from homework2.calculations import FigureEight


class Trajectory():
    """ Publish a geometry_msgs/Twist of the calculated linear and angular
    velocity to the tertlebot's cmd_vel at a fixed rate 
    """
    def __init__(self):
        self.W = rospy.get_param("parameters/width")    # initializing the width of the figure eight
        self.H = rospy.get_param("parameters/height")   # initializing the height of the figure eight
        self.T = rospy.get_param("parameters/period")   # initializing the amount of time it takes to complete the figure eight
        self.R = rospy.get_param("~pub_freq")           # initializing the frequency at which to publish the messages
        self.pub_turtle1_vel = rospy.Publisher("turtle1/cmd_vel", Twist, queue_size = 10) # TODO replace turtle1 with turtlebot
        self.pub_vel = rospy.Publisher("/cmd_vel", Twist, queue_size = 10)
        self.path_pub = rospy.Publisher("path", Path, queue_size = 10)
        self.pause = rospy.Service("pause", Empty, self.pause_turtle_callback)
        self.resume = rospy.Service("resume", Empty, self.resume_turtle_callback)
        self.figure_eight = FigureEight(self.T, self.H, self.W)
        self.rate = rospy.Rate(self.R)
        self._t = 0
        self.t_paused = rospy.get_time()
        self.delta_t_paused = 0
        self.move_turtlebot = True # TODO False
        self.init_t = rospy.get_time()
        self.round = 1
        self.path = Path()
        self.path.poses = []
        self.x = None
        self.y = None

        self.br = tf2_ros.StaticTransformBroadcaster() # static transform between world to odom frame

        static_transformStamped = geometry_msgs.msg.TransformStamped()
        static_transformStamped.header.stamp = rospy.Time.now()
        static_transformStamped.header.frame_id = "world"
        static_transformStamped.child_frame_id = "odom"

        static_transformStamped.transform.translation.x = 0
        static_transformStamped.transform.translation.y = 0
        static_transformStamped.transform.translation.z = 0
        quat = tf.transformations.quaternion_from_euler(0, 0, 1.08) # the angle of the turtle at (0,0,0) - from calculation
        static_transformStamped.transform.rotation.x = quat[0]
        static_transformStamped.transform.rotation.y = quat[1]
        static_transformStamped.transform.rotation.z = quat[2]
        static_transformStamped.transform.rotation.w = quat[3]
        self.br.sendTransform(static_transformStamped)

    def pause_turtle_callback(self, message):
        """ Callback function for pause service (type - Empty).
        The service call the turtlebot_twist function, publish the twist message
        (to store the turtlebot velocity) and pause the turtlebot movement.
        Args: None
        Return: EmptyResponse
        """
        rospy.logdebug(f"Pause Message")
        twist = self.turtlebot_twist()
        self.pub_turtle1_vel.publish(twist)
        self.pub_vel.publish(twist)
        self.move_turtlebot = False
        self.t_paused = rospy.get_time()
        return EmptyResponse()


    def resume_turtle_callback(self, message): 
        """ Callback function for resume service (type - Empty).
        The service resume the turtlebot movement with the stored velocity
        values.
        Args: None
        Return: EmptyResponse
        """
        rospy.logdebug(f"Resume Message")
        self.move_turtlebot = True
        self.delta_t_paused += rospy.get_time() - self.t_paused
        return EmptyResponse()


    def turtlebot_twist(self):
        """ Create a twist suitable for the turtlebot's cmd_vel
        Args: None
        Returns:
            Twist - a 2D twist object corresponding to linear/angular velocity
        """
        self.x, self.y, v, omega = self.figure_eight.get_velocity(self._t)
        return Twist(linear = Vector3(x = v, y = 0, z = 0),
                    angular = Vector3(x = 0, y = 0, z = omega))


    def move(self):
        """ Move the turtle by publish a twist messages to the cmd_vel
        """
        self.path.header.stamp = rospy.Time.now()
        self.path.header.frame_id = "world"
        self.path.poses.append(PoseStamped(header = self.path.header, 
                                        pose = Pose(
                                            position = Point(self.x, self.y, 0), 
                                            orientation = Quaternion(0, 0, 0, 1))))
        self.path_pub.publish(self.path)
        rospy.logdebug(f"Posting: {self._t, self.figure_eight.get_velocity(self._t)}")
        self._t = rospy.get_time() - self.init_t - self.delta_t_paused
        twist = self.turtlebot_twist()
        self.pub_turtle1_vel.publish(twist)
        self.pub_vel.publish(twist)
        rospy.logdebug(f"t = {self._t}")
        if int(self._t/self.round) == self.T:
            self.path.poses = []
            self.round += 1


    def run(self):
        while not rospy.is_shutdown():
            rospy.logdebug(f"Run Message")
            if self.move_turtlebot:
                self.move()
            else:
             rospy.logdebug(f"In pause mode")
            self.rate.sleep()


def main():
    """ The main() function. """
    rospy.init_node('trajectory', log_level = rospy.DEBUG)
    traj = Trajectory()
    traj.run()
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass