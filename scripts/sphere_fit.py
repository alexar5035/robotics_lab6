#!/usr/bin/env python3
import rospy
import numpy as np
import math
from robot_vision_lectures.msg import XYZarray
from robot_vision_lectures.msg import SphereParams

# initial matrices
a_matrix = []
b_matrix = []

# fill output values
fil_outx = 0.0
fil_outy = 0.0
fil_outz = 0.0
fil_outr = 0.0

# flags
valid_params = False
first_filter = True

def receive(point_data):
	global a_matrix
	global b_matrix
	
	# loop through point_data to create A and B matrices
	# A = 2Xn, 2Yn, 2Zn, 1
	# B = Xn^2 + Yn^2 + Zn^2
	for point in point_data.points:
			a_matrix.append([2*point.x, 2*point.y, 2*point.z, 1])
			b_matrix.append([point.x**2 + point.y**2 + point.z**2])
			
def model_fitting(a_matrix, b_matrix):
	# detect if sphere params are valid and unfiltered
	global valid_params
	# arrays created with initial matrices
	A = np.array(a_matrix)
	B = np.array(b_matrix)
	P = np.array([])
	
	# empty sphere params message for unfiltered
	ufil_sParams = SphereParams()
	
	# calculate P and account for mismatched dimensions with requirements
	try: 
		P = np.linalg.lstsq(A, B, rcond = None)[0]
		# set uf center sParams
		ufil_sParams.xc = P[0]
		ufil_sParams.yc = P[1]
		ufil_sParams.zc = P[2]
		# radius calculation: SQRT(P[3] + Xc^2 + Yc^2 + Zc^2)
		ufil_sParams.radius = math.sqrt(P[3] + P[0]**2 + P[1]**2 + P[2]**2)
		valid_params = True
		# return the parameter message
		return ufil_sParams
		
	except:
		# flag due to parameter calculation error
		valid_params = False
		print("matrice dimensions mismatch, continue")

def filter(ufil_sParams, fil_gain): 
	# detects first filter
	global first_filter
	# ref initial filter values
	global fil_outx
	global fil_outy
	global fil_outz
	global fil_outr
	
	# empty sParam for filtered
	fil_sParams = SphereParams()
	
	# stores first parameter
	if first_filter:
		# set output values
		fil_outx = ufil_sParams.xc
		fil_outy = ufil_sParams.yc
		fil_outz = ufil_sParams.zc
		fil_outr = ufil_sParams.radius
		# change first_filter to false since a filter is about to occur
		first_filter = False
		
	# set up input values
	fil_inx = ufil_sParams.xc
	fil_iny = ufil_sParams.yc
	fil_inz = ufil_sParams.zc
	fil_inr = ufil_sParams.radius
	
	# filter position
	fil_outx = fil_gain*fil_inx + (1-fil_gain)*fil_outx
	fil_outy = fil_gain*fil_iny + (1 - fil_gain)*fil_outy
	fil_outz = fil_gain*fil_inz + (1 - fil_gain)*fil_outz
	fil_outr = fil_gain*fil_inr + (1 - fil_gain)*fil_outr
	
	# set message values based on input
	fil_sParams.xc = fil_outx
	fil_sParams.yc = fil_outy
	fil_sParams.zc = fil_outz
	fil_sParams.radius = fil_outr
	
	return fil_sParams
	
	
if __name__ == '__main__':
	# ball detection node initialized
	rospy.init_node('sphere_fit', anonymous = True)
	# subscriber for point_data
	pd_sub = rospy.Subscriber("/xyz_cropped_ball", XYZarray, receive)
	# publisher fot sphere paramaters
	sp_pub = rospy.Publisher("/sphere_params", SphereParams, queue_size = 10)
	# loop rate
	rate = rospy.Rate(10)
	# set fil gain
	fil_gain = 0.05 # how much of the most recent input is included 
	
	while not rospy.is_shutdown():
		# check if matrices are not empty then run model_fitting
		if len(a_matrix) > 0 and len(b_matrix) > 0:
			ufil_sParams = model_fitting(a_matrix, b_matrix)
			if valid_params:
				fil_sParams = filter(ufil_sParams)
				# publish sphere params
				sp_pub.publish(fil_sParams)
		rate.sleep()

