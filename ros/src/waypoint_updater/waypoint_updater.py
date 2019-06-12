#!/usr/bin/env python
import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from scipy.spatial import KDTree
from std_msgs.msg import Int32
from styx_msgs.msg import Lane, Waypoint

import math

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number
MAX_DECEL = .5

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')
        
        # TODO: Add other member variables you need below
        self.base_lane = None
        self.stopline_wp_idx = -1
        self.pose = None
        self.base_waypoints = None
        self.waypoints_2d = None
        self.waypoints_tree = None


        #Topic:     current_pose
        #Message:     PoseStamped
        #Callback:  pose_cb
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)

        #Topic:     base_waypoints
        #Message:     Lane
        #Callback:  waypoints_cb
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        self.loop()

        #[BlockingCall] until a shutdown request is received by the node
        #rospy.spin()

        
    def loop(self):
        #it could be 30Hz
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            if self.pose and self.base_waypoints:
            #if self.pose and self.base_lane:
                #Get closest waypoint
                closest_waypoint_idx = self.get_closest_waypoint_idx()
                #Publish closest way point index
                self.publish_waypoints(closest_waypoint_idx)
                
            rate.sleep()

    def get_closest_waypoint_idx(self):
        #Get Coordinates of our car
        x = self.pose.pose.position.x
        y = self.pose.pose.position.y
        
        #Query on waypoints tree giving back closest point index in our KB Tree
        closest_idx = self.waypoints_tree.query([x, y], 1)[1]
        
        # Check if closest is ahead or behind vehicle
        closest_coord = self.waypoints_2d[closest_idx]
        prev_coord = self.waypoints_2d[closest_idx-1]
        
        #Equation for hyperplane through closest_coords
        #2D coordinate of the WayPoint
        cl_vect = np.array(closest_coord)
        #2D coordinate of the car's previous waypoint
        prev_vect = np.array(prev_coord)
        #2D coordinate of the car's current waypoint
        pos_vect = np.array([x, y])
        
        #Dot Prodcut to see positive(way point is behind the car) or negative(way point is in front of the car)
        val = np.dot(cl_vect-prev_vect, pos_vect-cl_vect)
        
        if val > 0:
            closest_idx = (closest_idx + 1) % len(self.waypoints_2d)
        return closest_idx
    
    def publish_waypoints(self, closest_idx):
        #Create a new lane message

        final_lane = self.generate_lane()
        self.final_waypoints_pub.publish(final_lane)
        
        
    def generate_lane(self):
        lane = Lane()
        closest_idx = self.get_closest_waypoint_idx()
        farthest_idx = closest_idx + LOOKAHEAD_WPS
        #base_waypoints = self.base_lane.waypoints[closest_idx:farthest_idx]
        base_waypoints = self.base_waypoints.waypoints[closest_idx:farthest_idx]

        #We didn't find any traffic lights
        if self.stopline_wp_idx == -1 or (self.stopline_wp_idx >= farthest_idx):
            #publish waypoints directly
            lane.waypoints = base_waypoints
        else:
            lane.waypoints = self.decelerate_waypoints(base_waypoints, closest_idx)

        return lane

    def decelerate_waypoints(self, waypoints, closest_idx):
        #Creating new waypoints messages 
        temp = []
        for i, wp in enumerate(waypoints):
            #Create new waypoint message
            p = Waypoint()
            #Set the pose to the base waypoint pose
            p.pose = wp.pose
            #Two waypoints back from line so front of the car stops at line
            stop_idx = max(self.stopline_wp_idx - closest_idx - 2, 0)
            #calculate the distance to stop at (zero if it's after the stop index)
            dist = self.distance(waypoints, i, stop_idx)

            #Truncate at 0 if the velociy gots too small
            vel = math.sqrt(2 * MAX_DECEL * dist)
            if vel < 1:
                vel = 0
            
            #Truncate at current velocity if the velociy gots larger (speed limit)
            p.twist.twist.linear.x = min(vel, wp.twist.twist.linear.x)

            #Add to the newly created list
            temp.append(p)

            return temp
            

    def pose_cb(self, msg):
        # TODO: Implement
        self.pose = msg
        pass

    def waypoints_cb(self, waypoints):
        # TODO: Implement
        self.waypoints = waypoints
        self.base_waypoints = waypoints
        if not self.waypoints_2d:
            self.waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in waypoints.waypoints]
            self.waypoints_tree = KDTree(self.waypoints_2d)
            
        pass

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        self.stopline_wp_idx = msg.data

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    #gets the linear velocity (x-direction) for a single waypoint.
    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    #sets the linear velocity (x-direction) for a single waypoint in a list of waypoints
    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    #Computes the distance between two waypoints in a list along the piecewise linear arc connecting all waypoints between the two
    #helpful in determining the velocities for a sequence of waypoints leading up to a red light 
    #the velocities should gradually decrease to zero starting some distance from the light
    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
