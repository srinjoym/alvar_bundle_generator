#!/usr/bin/env python

import rospy
from alvar_bundle_generator import main

if __name__ == '__main__':
    rospy.init_node(
        'alvar_bundle_generator', anonymous=True)
    main()
