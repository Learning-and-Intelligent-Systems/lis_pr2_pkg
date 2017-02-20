#!/usr/bin/env python
#Author: Ariel Anders
from lis_pr2_pkg.uber_controller import Uber
import rospy
import os




run_dance = lambda x:  os.system("rosrun lis_pr2_pkg dance_%s.py" % x)
announce = lambda x:  os.system(\
        'rosrun sound_play say.py "running the %s dance"' % x.replace("_"," "))

rospy.init_node("alldances")
uc = Uber()
uc.lift_torso()


if __name__ == "__main__":
    rospy.init_node("alldances")
    uc = Uber()
    uc.lift_torso()

    dances = ["caelan_clement", "caris_alex", "rohan_ferran"]

    for dance in dances:
        # reset arms
        uc.move_arm_to_side("l", blocking=False)
        uc.move_arm_to_side("r", blocking = True)
        #start dances
        announce(dance)
        run_dance(dance)
        

