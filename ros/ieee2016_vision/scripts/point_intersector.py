#!/usr/bin/env python
import rospy
from camera_manager import Camera
import tf

from ieee2016_msgs.srv import RequestMap

import numpy as np


class PointIntersector():
    '''
    Given a point in the camera frame and Shia's current position estimate where that point is along the wall.
    (We are assuming that the blocks will be flush against the wall, or have some offset from the wall).
    '''
    def __init__(self):
        # Map used for estimating distances
        map_request = rospy.ServiceProxy('/robot/request_map',RequestMap)
        self.map = np.array(map_request().map)

    def intersect_point(self, camera, point, time = None, offset = 0):
        # Make a ray and remove components we don't need
        ray = camera.make_3d_vector(point)
        raw_ray = np.copy(ray)
        ray[1] = 0
        unit_ray = ray / np.linalg.norm(ray)

        # Calculate alpha angle between [0,0,1] and the unit_ray in the camera frame
        forward_ray = np.array([0, 0, 1])
        alpha = np.arccos(np.dot(unit_ray, forward_ray))
        signed_alpha = -1 * np.sign(unit_ray[0]) * alpha

        # Find map frame position of the camera
        cam_tf = camera.get_tf(target_frame = "map", time = time)
        theta = cam_tf[2] + signed_alpha
        point = cam_tf[:2]

        self.offset = offset
        dist = self.simulate_scan(point, theta)

        return camera.make_3d_point(raw_ray, dist, output_frame = "map", time = time)

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
        # Sketchy as fuck way of detecting half blocks - move all the walls upwards.
        offset_map = (self.map.reshape(len(self.map)/5,5) + np.array([0,self.offset,0,self.offset,0])).flatten()
        for w in range(len(offset_map) / 5):
            intersection_dist = self.find_intersection(point, ray_direction, 
                                                       np.array([offset_map[5 * w],     offset_map[5 * w + 1]]),
                                                       np.array([offset_map[5 * w + 2] ,offset_map[5 * w + 3]]))
            if intersection_dist is not None:
                intersections.append(intersection_dist)

        # All intersection points found, now return the closest
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
    c = Camera(1)
    c.activate()
    i = PointIntersector()
    i.intersect_point(c, (1920 / 2, 108))
    c.deactivate()
    # print np.array([0.00464233,-0.30620446,1.0]) * c.proj_mat
