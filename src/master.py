#!/usr/bin/env python

import rospy
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
import roslaunch

from final_project.srv import sweep
# from std_srvs.srv import SetBool


if __name__ == '__main__':
    rospy.init_node("master_node")

    sweep = rospy.ServiceProxy("/initial_sweep", sweep)
    rospy.wait_for_service("/initial_sweep", 3.0)
    sweep_return = sweep(True)
    print "RETURNED VALUE FROM SWEEP NODE"
    print sweep_return
    # package = 'final_project'
    # executable = 'bottle_search.py'
    # node = roslaunch.core.Node(package, executable)
    #
    # launch = roslaunch.scriptapi.ROSLaunch()
    # launch.start()
    #
    # process = launch.launch(node)
    # print process.is_alive()
    # # process.stop()
    # rospy.spin()

    # main()
