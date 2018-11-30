#!/usr/bin/env python

import rospy
import sys
from alvar_bundle_generator import main

def print_usage():
  print "usage: alvar_bundle_generator marker_size master_tag_id"
  print " marker_size: length of one side of tag (in cm)"
  print " master_tag_id: ID of tag that should be master in bundle file"

if __name__ == '__main__':
    if len(sys.argv) < 3:
      print_usage()
    else:
      rospy.init_node(
          'alvar_bundle_generator', anonymous=True)
      main()
