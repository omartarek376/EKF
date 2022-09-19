#!/usr/bin/env python3
import rospy
import numpy as np
from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import MultiArrayLayout
from std_msgs.msg import MultiArrayDimension
import math 

# Initialization of the start values and matrices

gps = [0.0, 0.0, 0.0] 
steer = 0
vel = 0
H = np.eye(3)
Kn = np.array([[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]])
cov_x_bar = np.eye(3) * 0.1
cov_x_cap = np.eye(3) * 0.1

rospy.init_node("state_estimation")


def gps_callback(data):
    global gps
    gps = data.data


def steer_callback(data):
    global steer
    steer = data.data


def vel_callback(data):
    global vel
    vel = data.data

 # Initializing the Node   


state_pub = rospy.Publisher('vehicle_model/state', Float64MultiArray, queue_size=10)
rospy.Subscriber("/gps", Float64MultiArray, gps_callback)
rospy.Subscriber("/car_actions/steer", Float64, steer_callback)
rospy.Subscriber("/car_actions/vel", Float64, vel_callback)
r = rospy.Rate(10)

  



while not rospy.is_shutdown():

    beta = np.arctan(0.5 * np.tan(steer))

    Zn = [ gps[0], gps[1], gps[2] ]

    x_bar = np.array([[vel * math.cos(Zn[2] + beta) ]  ,[vel * math.sin(Zn[2] + beta) ]  ,[(vel*math.tan(steer)*math.cos(beta) / 4.9 ) ]])

    jacob = np.array([[1, 0, -vel*math.sin(Zn[2]) ] ,[0, 1, vel*math.cos(Zn[2])] ,[0, 0, 1]])

    cov_x_bar = (jacob @ cov_x_cap @ jacob.T) 

    Kn = cov_x_bar @ H.T @ np.linalg.pinv(H @ cov_x_bar @ H.T )

    x_cap = ( np.eye(3) - Kn @ H) @ x_bar + (Kn @ x_bar)

    cov_x_cap = ( np.eye(3) - Kn @ H) @ cov_x_bar
    
    car_state = [x_cap[0][0], x_cap[1][0], x_cap[2][0]]
 

    current_state = Float64MultiArray()
    layout = MultiArrayLayout()
    dimension = MultiArrayDimension()
    dimension.label = "current_state"
    dimension.size = 3
    dimension.stride = 3
    layout.data_offset = 0
    layout.dim = [dimension]
    current_state.layout = layout
    current_state.data = car_state
    state_pub.publish(current_state)
    r.sleep()