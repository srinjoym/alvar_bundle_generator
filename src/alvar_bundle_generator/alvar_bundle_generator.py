#!/usr/bin/env python
import rospy
import sys
import copy
import tf2_ros
import tf
import numpy as np
import xml.etree.cElementTree as ET
import transform_utilities as tu

from math import degrees, radians
from pybrain.optimization import CMAES
from pyquaternion import Quaternion
from ar_track_alvar_msgs.msg import AlvarMarkers, AlvarMarker
from geometry_msgs.msg import Pose

class BundleGenerator:
    def __init__(self, marker_size, master_tag_id):
        print "Initializing BundleGenerator"

        self.master_id = master_tag_id
        self.marker_size = marker_size
        self.optimize_id = -1
        self.frame_count = 0

        self.raw_frame_buffer = []
        self.marker_buffer = {}
        self.optimized_marker_poses = {}
        self.point_locations = {}

        self.ar_sub = rospy.Subscriber("ar_pose_marker", AlvarMarkers, self.callback)

        np.set_printoptions(suppress=True)
        self.tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(self.tfBuffer)

        rospy.sleep(3)
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
        tree.write("output.xml")  # TODO change save location

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
        for pose in self.marker_buffer[self.optimize_id]:
            sum += np.sum(np.square(np.subtract(x[0:4],pose[0:4])))
            x_quat = Quaternion(x[6], x[3], x[4], x[5])
            pose_quat = Quaternion(pose[6], pose[3], pose[4], pose[5])
            dist = Quaternion.absolute_distance(x_quat, pose_quat)/3.14
            sum += dist*dist
        return sum

    def optimize_pose(self, marker_id, save_transform=True):
        """
        Find optimized transform for marker relative to master tag
        :param marker_id: id for marker
        :param save_transform: bool, update optimized marker pose dict or not
        :return optimized pose
        """
        x0 = np.zeros(7)
        self.optimize_id = marker_id
        l = CMAES(self.objF, x0)
        l.minimize = True
        l.maxEvaluations = 1000
        pose = l.learn()
        print marker_id, pose  # DEBUG
        if save_transform:
            self.optimized_marker_poses[marker_id] = pose[0]
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
        skipped_frames = []
        for frame in frames:
            if ref_tag_id is None:  # Find tag that has transform to master
                for marker_id in frame.keys():
                    if marker_id in self.optimized_marker_poses:
                        ref_tag_id = marker_id

            if ref_tag_id is not None and ref_tag_id in frame:
                ref_tag_pose = frame[ref_tag_id]  # Pose of reference tag relative to camera
                if ref_tag_id in self.optimized_marker_poses:
                    ref_tag_transform = self.optimized_marker_poses[ref_tag_id]  # Transform from master to reference
                else:
                    ref_tag_transform = [0, 0, 0, 0, 0, 0, 1]  # If reference tag pose not found, use master transform

                for marker_id, marker_pose in frame.iteritems():  # Compute transform for each marker and save
                    marker_transform = tu.compute_transform(ref_tag_pose, marker_pose)
                    total_transform = tu.combine_transform(ref_tag_transform, marker_transform)
                    if marker_id in self.marker_buffer:
                        self.marker_buffer[marker_id].append(total_transform)
                    else:
                        self.marker_buffer[marker_id] = [total_transform]

            else:  # Skip frame because no reference tag in frame
                skipped_frames.append(frame)
        if save:  # Save positions in optimized marker pose dict
            for marker_id in self.marker_buffer.keys():
                self.optimize_pose(marker_id)

        return skipped_frames

    def learn_pose(self):
        """
        Entry function post recording frames, processes all frames that have master tag, then processing remaining
        frames using other tags for reference
        """
        prev_len = -1  # Keep track of skipped frames to avoid infinite loop
        skipped_frames = self.transform_and_save_frames(self.raw_frame_buffer, self.master_id)

        print "Frames Left To Process \n{0}".format(len(skipped_frames))

        while len(skipped_frames) > 0 and len(skipped_frames) != prev_len:
            prev_len = len(skipped_frames)
            skipped_frames = self.transform_and_save_frames(skipped_frames)

            print "Frames Left To Process \n{0}".format(len(skipped_frames))

        self.generate_points()
        self.create_bundle_file()

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

        if self.frame_count >= 25:
            print "Reached Frame Limit, Generating Bundle File..."
            self.stop_record()
            self.learn_pose()
            return

        self.frame_count += 1
        print "Recorded Frame " + str(self.frame_count)

        markers = {}
        for marker in raw_markers:
            markers[marker.id] = (tu.standardize_pose(marker.pose.pose))  # convert to numpy array
        self.raw_frame_buffer.append(markers)

    def stop_record(self):
        """
        Unregister subscriber to stop callbacks
        """
        self.ar_sub.unregister()

def main():
    gen = BundleGenerator(sys.argv[1], sys.argv[2])
    raw_input('Press Enter To Generate Bundle')
    gen.stop_record()
    gen.learn_pose()
