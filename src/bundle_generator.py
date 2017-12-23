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

class BundleGenerator:
    def __init__(self):
        print "in init"
        self.master_id = None
        self.marker_locations = {}
        self.angles = {}
        # rospy.Subscriber(image_topic,Image, self.process_img, queue_size=1)
        self.ar_sub = rospy.Subscriber("ar_pose_marker", AlvarMarkers, self.callback)
        self.tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(self.tfBuffer)
        # rospy.sleep(5)
        np.set_printoptions(suppress=True)
        print "finished init"

    def learn_pose(self):
        print "testing"
        x0 = np.zeros((len(self.marker_locations),7))
        print x0
        print "x0 row {0}".format(x0[0])
        l = CMAES(self.objF, x0)
        l.minimize = True
        l.maxEvaluations = 2000
        print l.learn()

        print "keys"
        for marker_id in sorted(self.marker_locations.keys()):
            print "Marker_id {0}".format(marker_id)
            for data in self.marker_locations[marker_id]:
                print data

        # print "angles"
        # for marker_id in sorted(self.angles.keys()):
        #     if marker_id == 9:
        #         for data in self.angles[marker_id]:
        #             print data


    def objF(self, x):
        x.shape = (len(self.marker_locations),7)
        sum = 0
        i = 0

        for marker_id in sorted(self.marker_locations.keys()):
            for pose in self.marker_locations[marker_id]:
                sum += np.sum(np.square(np.subtract(x[i][0:4],pose[0:4])))
                x_quat = Quaternion(x[i][6],x[i][3],x[i][4],x[i][5])
                pose_quat = Quaternion(pose[6],pose[3],pose[4],pose[5])
                dist = Quaternion.distance(x_quat,pose_quat)
                print "distance is {0}".format(dist*dist)
                sum += dist*dist
            i+=1
        return sum

    def stop_record(self):
        self.ar_sub.unregister()

    def callback(self, msg):
        raw_markers = msg.markers
        if len(raw_markers) < 1:
            return

        if not self.master_id:
            self.master_id = raw_markers[0].id

        master_name = 'ar_marker_{0}'.format(self.master_id)

        for marker in raw_markers:
            marker_name = "ar_marker_{0}".format(marker.id)
            try:
                trans = self.tfBuffer.lookup_transform(
                    master_name, marker_name, rospy.Time())
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
        quat = tf.transformations.quaternion_from_euler(euler_pose[3], euler_pose[4], euler_pose[5])
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
