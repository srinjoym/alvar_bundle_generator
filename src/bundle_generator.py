#!/usr/bin/env python
import rospy
import sys
from ar_track_alvar_msgs.msg import AlvarMarkers, AlvarMarker
import copy

class BundleGenerator:
  def __init__(self):
    print "in init"
    self.master_id = None
    self.marker_locations = {}
    #rospy.Subscriber(image_topic,Image, self.process_img, queue_size=1)
    rospy.Subscriber("ar_pose_marker",AlvarMarkers,self.callback)

    rospy.sleep(5)

    print "finished init"

  
  def callback(self, msg):
    # print msg
    raw_markers = msg.markers
    adj_markers = []

    if(len(raw_markers)>0):

      if self.master_id:
        master_list = filter(lambda d: d.id==self.master_id, raw_markers)
        if len(master_list) < 1:
          print "no master in frame, skipping"
          return
        else:
          master = master_list[0]
      else:
        master = raw_markers[0]
        self.master_id = master.id

      master_pose = master.pose.pose
      # print "master {0}".format(master)

      for marker in raw_markers:
        # print "************** RAW VALUES **********************"
        # print "marker {0} pose {1}".format(marker.id, marker.pose.pose.position)

        # print master_pose

        new_marker = copy.deepcopy(marker)
        new_marker_pose = new_marker.pose.pose

        new_marker_pose.position.x -= master_pose.position.x
        new_marker_pose.position.y -= master_pose.position.y
        new_marker_pose.position.z -= master_pose.position.z

        new_marker_pose.orientation.x -= master_pose.orientation.x
        new_marker_pose.orientation.y -= master_pose.orientation.y
        new_marker_pose.orientation.z -= master_pose.orientation.z
        new_marker_pose.orientation.w -= master_pose.orientation.w
        # print " "
        # print "************** NEW VALUES **********************"
        # print "new marker {0} pose {1}".format(new_marker.id, new_marker.pose.pose.position)
        # print " "
        adj_markers.append(new_marker)

    # print "Adjusted markers {0}".format(adj_markers)
    self.update_marker_location(adj_markers)


  def update_marker_location(self,adj_markers):

    for marker in adj_markers:
      marker_pose = marker.pose.pose
      if marker.id not in self.marker_locations:
        self.marker_locations[marker.id] = {"pose": marker_pose,"count":1}
      else:
        current_pose = self.marker_locations[marker.id]["pose"]
        count = self.marker_locations[marker.id]["count"]

        current_pose.position.x = ((current_pose.position.x*count)+marker_pose.position.x)/(count+1)
        current_pose.position.y = ((current_pose.position.y*count)+marker_pose.position.y)/(count+1)
        current_pose.position.z = ((current_pose.position.z*count)+marker_pose.position.z)/(count+1)
        count += 1
        print self.marker_locations[marker.id]


def main():
  BundleGenerator()

if __name__ == '__main__':
  ## First initialize moveit_commander and rospy.
  #moveit_commander.roscpp_initialize(sys.argv)
  rospy.init_node('ar_track_alvar_bundle_generator', anonymous=True)
  main()
  rospy.spin()
