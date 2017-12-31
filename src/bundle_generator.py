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
        print "in init"
        self.master_id = None
        self.marker_locations = {}
        self.bundle_locations = {}
        self.point_locations = {}
        self.angles = {}
        # rospy.Subscriber(image_topic,Image, self.process_img, queue_size=1)
        self.ar_sub = rospy.Subscriber("ar_pose_marker", AlvarMarkers, self.callback)
        self.tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(self.tfBuffer)
        self.optimize_id = -1
        self.marker_size = 4.75/100
        # rospy.sleep(5)
        np.set_printoptions(suppress=True)
        print "finished init"

    def learn_pose(self):
        for marker_id in sorted(self.marker_locations.keys()):
            x0 = np.zeros(7)
            self.optimize_id = marker_id

            l = CMAES(self.objF, x0)
            l.minimize = True
            l.maxEvaluations = 1000

            pose = l.learn()
            print pose

            self.bundle_locations[marker_id] = pose[0]

        self.generate_points()
        self.create_bundle_file()

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
        for marker_id in sorted(self.bundle_locations.keys()):
            # if(marker_id==self.master_id):
            pose = self.bundle_locations[marker_id]
            # print "quaternion {0}".format(pose)

            rot = tf.transformations.euler_from_quaternion(pose[3:7])
            matrix = tf.transformations.compose_matrix(translate=pose[0:3], angles=rot)
            # print "transformation matrix \n {0}".format(matrix.shape)

            points = self.points_from_pose(pose)

            res = []
            for point in points:
                # print "origin_point \n {0}".format(point)
                dot = np.array([point[0],point[1],point[2],1]).T.dot(matrix)
                res.append(dot[:3])

            self.point_locations[marker_id] = res

        print self.point_locations

    def objF(self, x):
        sum = 0
        # print "begin optimize"

        for pose in self.marker_locations[self.optimize_id]:
            sum += np.sum(np.square(np.subtract(x[0:4],pose[0:4])))
            x_quat = Quaternion(x[6],x[3],x[4],x[5])
            pose_quat = Quaternion(pose[6],pose[3],pose[4],pose[5])
            dist = Quaternion.absolute_distance(x_quat,pose_quat)/3.14
            sum += dist*dist

        # print "marker {0} sum {1}".format(self.optimize_id, sum)
        # print "end optimize"
        return sum

    def stop_record(self):
        self.ar_sub.unregister()

    def callback(self, msg):
        raw_markers = msg.markers
        if len(raw_markers) < 1:
            return

        if not self.master_id:
            self.master_id = raw_markers[0].id
            #Debugs
            # self.master_id = 0

        master_name = 'ar_marker_{0}'.format(self.master_id)

        for marker in raw_markers:
            marker_name = "ar_marker_{0}".format(marker.id)
            try:
                time = rospy.Time()
                trans = self.tfBuffer.lookup_transform(
                    master_name, marker_name, time)
                print "trans from {0} to {1} is {2}".format(master_name,marker_name,trans)
                if marker.id in self.marker_locations:
                    self.marker_locations[marker.id].append(self.tf_to_array(trans.transform,marker.id))
                else:
                    self.marker_locations[marker.id] = [self.tf_to_array(trans.transform,marker.id)]
            except Exception, e:
                print "No Transform {0}".format(e)
                return

    def tf_to_array(self,pose, id_):
        angle = tf.transformations.euler_from_quaternion([
        pose.rotation.x, pose.rotation.y, pose.rotation.z, pose.rotation.w])
        euler_pose = []
        euler_pose.append(pose.translation.x)
        euler_pose.append(pose.translation.y)
        euler_pose.append(pose.translation.z)
        euler_pose.append(degrees(angle[0]))
        euler_pose.append(degrees(angle[1]))
        euler_pose.append(degrees(angle[2]))
        if id_ in self.angles:
            self.angles[id_].append(euler_pose[3:6])
        else:
            self.angles[id_] = [euler_pose[3:6]]
        euler_pose = simplify_pose(euler_pose)
        quat = tf.transformations.quaternion_from_euler(radians(euler_pose[3]), radians(euler_pose[4]), radians(euler_pose[5]))
        return np.concatenate((np.array(euler_pose[0:3]), quat))

def simplify_pose(pose):
    for i in range(0,3):
        if(pose[i]<0):
            pose[i] = -pose[i]
            pose[i+3] = pose[i+3] + 180
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
