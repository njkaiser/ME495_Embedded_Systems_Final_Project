#!/usr/bin/env python
import rospy
import baxter_interface


def wave():
    limb.move_to_joint_positions(angles)
    for _move in range(10):
        limb.move_to_joint_positions(wave_1)
        limb.move_to_joint_positions(wave_2)


if __name__ == '__main__':
    rospy.init_node('hello_baxter', anonymous=False)
    # rate = rospy.Rate(20) # in hz

    limb = baxter_interface.Limb('right')
    angles = limb.joint_angles()
    angles['right_s0']=0.0
    angles['right_s1']=0.0
    angles['right_e0']=0.0
    angles['right_e1']=0.0
    angles['right_w0']=0.0
    angles['right_w1']=0.0
    angles['right_w2']=0.0
    wave_1 = {'right_s0': -0.459, 'right_s1': -0.202, 'right_e0': 1.807, 'right_e1': 1.714, 'right_w0': -0.906, 'right_w1': -1.545, 'right_w2': -0.276}
    wave_2 = {'right_s0': -0.395, 'right_s1': -0.202, 'right_e0': 1.831, 'right_e1': 1.981, 'right_w0': -1.979, 'right_w1': -1.100, 'right_w2': -0.448}

    # while not rospy.is_shutdown():
    # try:
    print "WAVING 10 TIMES"
    wave()
    print "DONE"
    quit()
    # except rospy.ROSInterruptException:
        # pass
        # rate.sleep()

### END OF SCRIPT
