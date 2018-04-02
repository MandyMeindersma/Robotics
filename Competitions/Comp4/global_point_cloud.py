#!/usr/bin/env python

import rospy
from std_srvs.srv import Empty

rospy.init_node('global')

rospy.wait_for_service('global_localization')
global_localization = rospy.ServiceProxy('global_localization', Empty)
global_localization()

rospy.spin()