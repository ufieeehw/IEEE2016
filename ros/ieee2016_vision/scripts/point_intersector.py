#!/usr/bin/env python
import rospy
import tf
from camera_manager import Camera

import numpy as np

class IntersectPoint():
    '''
    Given a point in the camera frame and Shia's current position estimate where that point is along the wall.
    (We are assuming that the blocks will be flush against the wall, or have some offset from the wall.)
    '''
    def __init__(self):
        # Map used for estimating distances
        self.map = np.array([0, 0, 0, .784, 0, .784, .015, .784, .015, .784, .015, 1.158, 0, 1.158, .015, 1.158, 0, 1.158, 0, 2.153, .464, .784, .479, .784, .479, .784, .479, 1.158, .464, .784, .464, 1.158, .464, 1.158, .479, 1.158, 0, 0, .549, 0, .549, 0, .549, .317, .549, .317, .569, .317, .569, 0, .569, .317, .569, 0, .809, 0, .809, 0, .809, .317, .809, .317, .829, .317, .829, 0, .829, .317, .829, 0, 2.458, 0, 0, 2.153, 2.458, 2.153, 2.458, 0, 2.458, .907, 2.161, .907, 2.458, .907, 2.161, .907, 2.161, 1.178, 2.161, 1.178, 2.458, 1.178, 2.458, 1.178, 2.458, 1.181, 2.161, 1.181, 2.458, 1.181, 2.161, 1.181, 2.161, 1.452, 2.161, 1.452, 2.458, 1.452, 2.458, 1.452, 2.458, 1.482, 2.161, 1.482, 2.458, 1.482, 2.161, 1.482, 2.161, 1.753, 2.161, 1.753, 2.458, 1.753, 2.458, 1.753, 2.458, 1.783, 2.161, 1.783, 2.458, 1.783, 2.161, 1.783, 2.161, 2.054, 2.161, 2.054, 2.458, 2.054, 2.458, 2.054, 2.458, 2.153]).astype(np.float32)
        #rospy.Subscriber('/robot/pf_pose_est', PoseStamped, self.got_pose, queue_size=10)

    def intersect_point(self, camera, point):
        # Make a ray and remove components we don't need
        ray = camera.make_3d_ray(point)[:3]
        ray[1] = 0
        unit_ray = ray / np.linalg.norm(ray)

        # Calculate alpha between [0,0,1] and the unit_ray in the camera frame
        forward_ray = np.array([0,0,1])
        alpha = np.arccos(np.dot(unit_ray,forward_ray))
        signed_alpha = -1 * np.sign(unit_ray[0]) * alpha

        # Find map frame position of the camera
        cam_tf = camera.get_tf("map")
        theta = cam_tf[2] + signed_alpha
        point = cam_tf[:2]
        print ray
        
    def simulate_scan(self, point, theta):
        '''
        The works similarly to the particle filter raytracer. We just need to calculate theta for our point
        and the position of the camera. Theta is comprised of the robots yaw, and the angle the point makes
        with the camera in the frame.
        '''
        # Make sure the point is a numpy array
        point = np.array(point)
        ray_direction = np.array([np.cos(theta), np.sin(theta)])

        intersections = []
        # Go through each wall and test for intersections, then pick the closest intersection
        for w in range(len(self.map)/4):
            intersection_dist = self.find_intersection(point, ray_direction, np.array([self.map[4*w],self.map[4*w+1]]),
                                                                            np.array([self.map[4*w+2],self.map[4*w+3]]))
            if intersection_dist is not None:
                intersections.append(intersection_dist)

        #All intersection points found, now return the closest
        return min(intersections)
    
    def find_intersection(self, ray_origin, ray_direction, point1, point2):
        # Ray-Line Segment Intersection Test in 2D
        # http://bit.ly/1CoxdrG
        v1 = ray_origin - point1
        v2 = point2 - point1
        v3 = np.array([-ray_direction[1], ray_direction[0]])

        v2_dot_v3 = np.dot(v2, v3)
        if v2_dot_v3 == 0:
            return None

        t1 = np.cross(v2, v1) / v2_dot_v3
        t2 = np.dot(v1, v3) / v2_dot_v3
            
        if t1 >= 0.0 and t2 >= 0.0 and t2 <= 1.0:
            return t1
        return None


if __name__ == "__main__":
    rospy.init_node("point_intersector")
    c = Camera("cam_1")
    c.activate()
    i = IntersectPoint()
    i.intersect_point(c,(1920/2,108))
    c.deactivate()
