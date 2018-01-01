#!/usr/bin/env python
import rospy
import sys
from ar_track_alvar_msgs.msg import AlvarMarkers, AlvarMarker
from geometry_msgs.msg import Pose
import copy
import tf2_ros
import tf
from math import degrees, radians
from pybrain.optimization import CMAES
import numpy as np
from pyquaternion import Quaternion
import xml.etree.cElementTree as ET

class BundleGenerator:
    def __init__(self):
        print "Initializing BundleGenerator"
        self.master_id = None
        self.master_pose = None
        self.marker_buffer = {}
        self.optimized_marker_poses = {}
        self.point_locations = {}

        self.ar_sub = rospy.Subscriber("ar_pose_marker", AlvarMarkers, self.callback)
        self.tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(self.tfBuffer)
        self.optimize_id = -1
        self.marker_size = 4.75/100
        np.set_printoptions(suppress=True)

        rospy.sleep(3)
        print "Finished Init BundleGenerator"

    def add_marker_to_file(self,root, tag_id):
        marker = ET.SubElement(root, "marker",index=str(tag_id),status=str(1))

        for point in self.point_locations[tag_id]:
            corner = ET.SubElement(marker,"corner", x = str(point[0]*100), y=str(point[1]*100), z=str(point[2]*100))
            # corner.x, corner.y, corner.z = point

    def create_bundle_file(self):
        root = ET.Element("multimarker",markers=str(len(self.point_locations)))

        self.add_marker_to_file(root,self.master_id)

        for marker_id in sorted(self.point_locations.keys()):
            if marker_id != self.master_id:
                self.add_marker_to_file(root,marker_id)

        tree = ET.ElementTree(root)
        tree.write("test.xml")
       
    def points_from_pose(self,pose):
        p_bl = np.array(pose)
        p_bl[0] -= self.marker_size/2
        p_bl[1] -= self.marker_size/2
        p_br = np.array(p_bl)
        p_br[0] += self.marker_size
        p_tr = np.array(p_br)
        p_tr[1] += self.marker_size
        p_tl = np.array(p_tr)
        p_tl[0] -= self.marker_size
        return p_bl,p_br,p_tr,p_tl

    def generate_points(self):
        for marker_id in sorted(self.optimized_marker_poses.keys()):
            pose = self.optimized_marker_poses[marker_id]
            rot = tf.transformations.euler_from_quaternion(pose[3:7])
            matrix = tf.transformations.compose_matrix(translate=pose[0:3], angles=rot)
            points = self.points_from_pose(pose)
            res = []
            for point in points:
                dot = np.array([point[0],point[1],point[2],1]).T.dot(matrix)
                res.append(dot[:3])
            self.point_locations[marker_id] = res

    def objF(self, x):
        sum = 0
        for pose in self.marker_buffer[self.optimize_id]:
            sum += np.sum(np.square(np.subtract(x[0:4],pose[0:4])))
            x_quat = Quaternion(x[6],x[3],x[4],x[5])
            pose_quat = Quaternion(pose[6],pose[3],pose[4],pose[5])
            dist = Quaternion.absolute_distance(x_quat,pose_quat)/3.14
            sum += dist*dist 
        return sum

    def optimize_pose(self, marker_id, save_transform=True):
        x0 = np.zeros(7)
        self.optimize_id = marker_id
        l = CMAES(self.objF, x0)
        l.minimize = True
        l.maxEvaluations = 1000
        pose = l.learn()
        print marker_id, pose #DEBUG
        #TODO transform relative to master pose
        if save_transform:
            self.optimized_marker_poses[marker_id] = compute_transform(self.master_pose, pose[0])
        return pose[0]

    def learn_pose(self):
        self.master_id = find_max_count(self.marker_buffer)
        self.master_pose = self.optimize_pose(self.master_id, save_transform=False)
        print "master tag is {0}".format(self.master_id)
        print "computer transform test {0}".format(compute_transform(self.master_pose,self.master_pose))
        for marker_id in sorted(self.marker_buffer.keys()):
            self.optimize_pose(marker_id)
        print self.marker_buffer
        print self.optimized_marker_poses #DEBUG
        self.generate_points()
        self.create_bundle_file()

    def stop_record(self):
        self.ar_sub.unregister()

    def callback(self, msg):
        raw_markers = msg.markers
        if len(raw_markers) < 1:
            return
        print "callback" #DEBUG
        for marker in raw_markers:
            if marker.id in self.marker_buffer:
                self.marker_buffer[marker.id].append(standardize_pose(marker.pose.pose))
            else:
                self.marker_buffer[marker.id] = [standardize_pose(marker.pose.pose)]

def compute_transform(p1, p2):
    res = np.zeros(7)
    res[0:3] = p2[0:3]-p1[0:3]
    p1[6] = -p1[6]
    q2, q1 = Quaternion(p2[6],*p2[3:6]), Quaternion(p1[6],*p1[3:6])
    res[3:7] = (q2/q1).elements
    res[3], res[6] = res[6], res[3]
    return res

def find_max_count(dict):
    max_count, max_id = -1,-1
    for key,value in dict.iteritems():
        if(len(value)>max_count):
            max_id = key
            max_count = len(value)
    return max_id

def standardize_pose(pose):
    trans = pose.position
    orient = pose.orientation
    angle = tf.transformations.euler_from_quaternion([
    orient.x, orient.y, orient.z, orient.w])
    euler_pose = [trans.x,trans.y,trans.z, degrees(angle[0]), degrees(angle[1]),degrees(angle[2])]
    euler_pose = simplify_pose(euler_pose)
    quat = tf.transformations.quaternion_from_euler(radians(euler_pose[3]), radians(euler_pose[4]), radians(euler_pose[5]))
    return np.concatenate((np.array(euler_pose[0:3]), np.array([orient.x,orient.y,orient.z,orient.w])))

def simplify_pose(pose):
    # for i in range(0,3):
    #     if(pose[i]<0):
    #         pose[i] = -pose[i]
    #         pose[i+3] = pose[i+3] + 180
    for i in range(3,6):
        while (pose[i]<0): pose[i]+=360
    return pose

def main():
    gen = BundleGenerator()
    raw_input('Press Enter To Exit')
    gen.stop_record()
    gen.learn_pose()

if __name__ == '__main__':
    # First initialize moveit_commander and rospy.
    # moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node(
        'ar_track_alvar_bundle_generator', anonymous=True)
    main()
