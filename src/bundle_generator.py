#!/usr/bin/env python
import rospy
import sys
from ar_track_alvar_msgs.msg import AlvarMarkers, AlvarMarker
from geometry_msgs.msg import Pose
import copy
import tf2_ros
import tf
import math

class BundleGenerator:
  def __init__(self):
    print "in init"
    self.master_id = None
    self.marker_locations = {}
    #rospy.Subscriber(image_topic,Image, self.process_img, queue_size=1)
    rospy.Subscriber("ar_pose_marker",AlvarMarkers,self.callback)
    self.tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(self.tfBuffer)

    rospy.sleep(5)

    print "finished init"

  
  def callback(self, msg):
    # print msg
    raw_markers = msg.markers
    adj_markers = []

    if(len(raw_markers)>0):

      if not self.master_id:
        master = raw_markers[0]
        self.master_id = master.id

      for marker in raw_markers:

        try: 
          trans = self.tfBuffer.lookup_transform('ar_marker_{0}'.format(marker.id), 'ar_marker_{0}'.format(self.master_id),rospy.Time())
          print "transform {0}".format(trans)
        except:
          print "no transform"
          return

        rot = trans.transform.rotation

        euler = tf.transformations.euler_from_quaternion((rot.x,rot.y,rot.z,rot.w))

        print "trans pos {0}".format(trans.transform.translation)
        print "euler angle: {0}".format(list(map(lambda x: math.degrees(x), euler)))
        



  def update_marker_location(self,adj_markers):

    for marker in adj_markers:
      marker_pose = marker.pose.pose
      if marker.id not in self.marker_locations:
        self.marker_locations[marker.id] = {"pose": marker_pose,"count":1}
      else:
        current_pose = self.marker_locations[marker.id]["pose"]
        new_pose = Pose()
        count = self.marker_locations[marker.id]["count"]

        new_pose.position.x = ((current_pose.position.x*count)+marker_pose.position.x)/(count+1)
        new_pose.position.y = ((current_pose.position.y*count)+marker_pose.position.y)/(count+1)
        new_pose.position.z = ((current_pose.position.z*count)+marker_pose.position.z)/(count+1)
        self.marker_locations[marker.id]["count"] += 1

        current_pose = new_pose

        # print self.marker_locations


def main():
  BundleGenerator()

if __name__ == '__main__':
  ## First initialize moveit_commander and rospy.
  #moveit_commander.roscpp_initialize(sys.argv)
  rospy.init_node('ar_track_alvar_bundle_generator', anonymous=True)
  main()
  rospy.spin()
