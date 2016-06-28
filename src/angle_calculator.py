import numpy as np
import math as math

shoulder_hand = [0.242, -0.000, -0.299]
shoulder_elbow = [0.265, 0.000, -0.000]
elbow_hand = [0.302, 0.000, -0.000]
def distance_func(arr):
    return math.sqrt(sum([i**2 for i in arr]))

def c_law_of_cosines(arr1, arr2, arr3):
    A = distance_func(arr1)
    B = distance_func(arr2)
    C = distance_func(arr3)
    return math.degrees(math.acos((A**2 + B**2 - C**2)/(2*A*B)))

def law_of_cosines(A_,B_,C_, elbow = True):
    if elbow:
    	A = distance_func(list(A_))
    	B = distance_func(list(B_))
    	C = distance_func(list(C_))
    return math.acos((A**2 + B**2 - C**2)/(2*A*B))

#The corresponding elbow_flex_joint angle is simply negative of this
print "Human angle: ", c_law_of_cosines(elbow_hand, shoulder_elbow, shoulder_hand) 
print "Robot angle: ", -1 * c_law_of_cosines(elbow_hand, shoulder_elbow, shoulder_hand) 
