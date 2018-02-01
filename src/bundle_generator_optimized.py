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
from itertools import combinations_with_replacement

class BundleGenerator:
    def __init__(self):
        print "Initializing BundleGenerator"

        self.master_id = 0  # TODO make ros param
        self.marker_size = 4.75 / 100  # TODO make ros param
        self.optimize_id1 = -1
        self.optimize_id2 = -1

        self.raw_frame_buffer = []
        self.marker_buffer = {}
        self.optimized_marker_poses = {}
        self.point_locations = {}

        self.ar_sub = rospy.Subscriber("ar_pose_marker", AlvarMarkers, self.callback)

        np.set_printoptions(suppress=True)
        self.tfBuffer = tf2_ros.Buffer()  # DEBUG
        listener = tf2_ros.TransformListener(self.tfBuffer)

        rospy.sleep(1)
        print "Finished Init BundleGenerator"

    
    def add_marker_to_file(self, root, tag_id):
        """
        Add points for marker in counterclockwise order to xml tree
        :param root: xml tree root
        :param tag_id: tag id
        """
        marker = ET.SubElement(root, "marker", index=str(tag_id), status=str(1))

        for point in self.point_locations[tag_id]:
            ET.SubElement(marker, "corner", x=str(point[0]*100), y=str(point[1]*100), z=str(point[2]*100))

    def create_bundle_file(self):
        """
        Create xml tree and add each master, and each marker to tree
        """
        root = ET.Element("multimarker", markers=str(len(self.point_locations)))

        self.add_marker_to_file(root, self.master_id)

        for marker_id in sorted(self.point_locations.keys()):
            if marker_id != self.master_id:
                self.add_marker_to_file(root, marker_id)

        tree = ET.ElementTree(root)
        tree.write("test.xml")  # TODO change save location

    def points_from_pose(self,pose):
        """
        Calculate coordinates for corners of marker
        :param pose: marker center coordinate
        """
        p_bl = np.array(pose)
        p_bl[0] -= self.marker_size/2
        p_bl[1] -= self.marker_size/2
        p_br = np.array(p_bl)
        p_br[0] += self.marker_size
        p_tr = np.array(p_br)
        p_tr[1] += self.marker_size
        p_tl = np.array(p_tr)
        p_tl[0] -= self.marker_size
        return p_bl, p_br, p_tr, p_tl

    def generate_points(self):
        """
        Compute final coordinates for each marker based on optimized transforms
        """
        for marker_id in sorted(self.optimized_marker_poses.keys()):
            pose = self.optimized_marker_poses[marker_id]
            rot = tf.transformations.euler_from_quaternion(pose[3:7])
            matrix = tf.transformations.compose_matrix(translate=pose[0:3], angles=rot)
            points = self.points_from_pose(pose)
            res = []
            for point in points:
                dot = np.array([point[0], point[1], point[2], 1]).T.dot(matrix)
                dot[2] = -dot[2]
                res.append(dot[:3])
            self.point_locations[marker_id] = res

    def objF(self, x):
        """
        Blackbox optimization function, sums error relative to all transforms in marker buffer for both translation
        and rotation
        :param x:
        """
        sum = 0
        for pose in self.marker_buffer[self.optimize_id1][self.optimize_id2]:
            sum += np.sum(np.square(np.subtract(x[0:4],pose[0:4])))
            x_quat = Quaternion(x[6], x[3], x[4], x[5])
            pose_quat = Quaternion(pose[6], pose[3], pose[4], pose[5])
            dist = Quaternion.absolute_distance(x_quat, pose_quat)/3.14
            sum += dist*dist
        return sum

    def optimize_pose(self, from_marker_id,to_marker_id, save_transform=True):
        """
        Find optimized transform for marker relative to master tag
        :param marker_id: id for marker
        :param save_transform: bool, update optimized marker pose dict or not
        :return optimized pose
        """
        x0 = np.zeros(7)
        self.optimize_id1 = from_marker_id
        self.optimize_id2 = to_marker_id
        l = CMAES(self.objF, x0)
        l.minimize = True
        l.maxEvaluations = 1000
        pose = l.learn()
        print "optimized transfrom from {0} to {1} is {2}".format(from_marker_id,to_marker_id, pose)
        return pose[0]

    def transform_and_save_frames(self, frames, ref_tag_id=None, save=True):
        """
        Process list of frames, transform each pose relative to reference tag if given
        If no reference tag is given, finds a reference tag with optimized transform to master for each frame
        Returns list of skipped frames that didn't have reference tag
        :param frames: list of frames to process
        :param ref_tag_id: id for reference tag
        :param save: bool, save optimized pose in optimized marker pose dict
        :return: skipped frames
        """
        for frame in frames:
            for marker_id_1, marker_id_2 in combinations_with_replacement(frame.keys(),2):
                pose1, pose2 = frame[marker_id_1], frame[marker_id_2]
                if marker_id_1 in self.marker_buffer:
                    if marker_id_2 in self.marker_buffer[marker_id_1]:
                        self.marker_buffer[marker_id_1][marker_id_2].append(compute_transform(pose1, pose2))
                    else:
                        self.marker_buffer[marker_id_1][marker_id_2] = [compute_transform(pose1, pose2)]
                else:
                    self.marker_buffer[marker_id_1] = {marker_id_2:[compute_transform(pose1, pose2)]}

        finished_list = [self.master_id]
        for marker_id in self.marker_buffer[self.master_id]:  #Optimize all with relation to master
            self.optimized_marker_poses[marker_id] = self.optimize_pose(self.master_id, marker_id)
            finished_list.append(marker_id)

        self.marker_buffer.pop(self.master_id)
        prev_len = len(self.marker_buffer.keys())
        while len(self.marker_buffer.keys())>0 and prev_len != len(self.marker_buffer.keys()):
            prev_len = len(self.marker_buffer.keys())
            for marker_id_1, transforms in self.marker_buffer.iteritems():
                if marker_id_1 not in finished_list:
                    break
                for marker_id_2 in transforms:
                    if marker_id_2 not in finished_list:
                        trans = combine_transform(self.optimized_marker_poses[marker_id_1], self.optimized_marker_poses[marker_id_2])
                        self.optimized_marker_poses[marker_id_2] = trans
                        finished_list.append(marker_id_2)
                self.marker_buffer.pop(marker_id_1)

                


    def learn_pose(self):
        """
        Entry function post recording frames, processes all frames that have master tag, then processing remaining
        frames using other tags for reference
        """
        prev_len = -1  # Keep track of skipped frames to avoid infinite loop
        self.transform_and_save_frames(self.raw_frame_buffer, self.master_id)

        

        self.generate_points()
        self.create_bundle_file()

    def stop_record(self):
        """
        Unregister subscriber to stop callbacks
        """
        self.ar_sub.unregister()

    def callback(self, msg):
        """
        If more than 2 tags in message, save frame in raw frame buffer
        :param msg: subscriber message
        """
        if rospy.is_shutdown():
            self.stop_record()
            sys.exit(1)
            
        raw_markers = msg.markers
        if len(raw_markers) < 2:
            return

        # print msg.markers  # DEBUG

        print "callback"  # DEBUG

        markers = {}
        for marker in raw_markers:
            markers[marker.id] = (standardize_pose(marker.pose.pose))
        self.raw_frame_buffer.append(markers)

def combine_transform(p1, p2):
    """
    Utility method for combining two transforms, adds translational component, and multiplies quaternions
    :param t1: transform 1
    :param t2: transform 2
    :return: resulting transform
    """
    r1 = tf.transformations.euler_from_quaternion(p1[3:7])
    r2 = tf.transformations.euler_from_quaternion(p2[3:7])
    m1 = tf.transformations.compose_matrix(translate=p1[0:3], angles=r1)
    m2 = tf.transformations.compose_matrix(translate=p2[0:3], angles=r2)
    res = m1.dot(m2)
    scale, shear, angles, trans, persp = tf.transformations.decompose_matrix(res)
    rot_quat = tf.transformations.quaternion_from_euler(*angles)
    return trans.tolist() + rot_quat.tolist()


def compute_transform(p1, p2):
    """
    Utility method for computing transform between two poses, subtracts translation, and divides quaternions
    :param p1: pose 1
    :param p2: pose 2
    :return: resulting transform
    """
    r1 = tf.transformations.euler_from_quaternion(p1[3:7])
    r2 = tf.transformations.euler_from_quaternion(p2[3:7])
    m1 = tf.transformations.compose_matrix(translate=p1[0:3], angles=r1)
    m2 = tf.transformations.compose_matrix(translate=p2[0:3], angles=r2)
    res = np.linalg.inv(m1).dot(m2)
    scale, shear, angles, trans, persp = tf.transformations.decompose_matrix(res)
    rot_quat = tf.transformations.quaternion_from_euler(*angles)
    return trans.tolist()+rot_quat.tolist()


def find_max_count(data):
    """
    Utility function for finding element in dictionary with most items
    :param data: dict
    """
    max_count, max_id = -1, -1
    for key, value in data.iteritems():
        if len(value) > max_count:
            max_id = key
            max_count = len(value)
    return max_id


def standardize_pose(pose):
    """
    Convert geometry msgs pose to numpy array and standardize pose format
    :param pose: geom msgs pose
    :return: numpy array
    """
    trans = pose.position
    orient = pose.orientation
    return np.array([trans.x, trans.y, trans.z, orient.x, orient.y, orient.z, orient.w])

def main():
    gen = BundleGenerator()
    raw_input('Press Enter To Generate Bundle')
    gen.stop_record()
    gen.learn_pose()

if __name__ == '__main__':
    rospy.init_node(
        'ar_track_alvar_bundle_generator', anonymous=True)
    main()
