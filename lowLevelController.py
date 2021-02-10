#!/usr/bin/env python


# 30/01/2020 
# Low level flight controller for a quadrotor with X configuration
# The design of the controller is based on the ideas at https://web.archive.org/web/20181114173019/https://blog.owenson.me/build-your-own-quadcopter-flight-controller/
# by Fatih Gokce


import rospy, time, math, sys
from squaternion import euler2quat, quat2euler, Quaternion
import numpy as np
from matplotlib import pyplot as plt
import pygame
from pid import PID
import os

import sensor_msgs
from sensor_msgs.msg import FluidPressure
from sensor_msgs.msg import Imu
from mav_msgs.msg import Actuators
from std_msgs.msg import String

rr_pid = PID(p=0.7, i=1, d=0, imax=10)
pr_pid = PID(p=0.7, i=1, d=0, imax=10)
yr_pid = PID(p=8.0, i=1, d=0, imax=10)
rs_pid = PID(p=4.5)
ps_pid = PID(p=4.5)
ys_pid = PID(p=10)

STABILIZE 	= 0
ALTHOLD		= 1 # For future use

mode = STABILIZE
roll = 0.0
pitch = 0.0
yaw = 0.0
gyroPitch = 0.0
gyroRoll = 0.0
gyroYaw = 0.0
imuDataReceived = 0
counter = 0
yaw_target = 0.0
msgTime = 0.0 # in msecs


def wrap_180(x):
	if x < -180.0:
		return x + 360
	elif x > 180:
		return x - 360
	else:
		return x

def plot_x(PE, RE, YE):
	global counter

	plt.plot(counter, PE, '*')
	plt.plot(counter, RE, '+')
	plt.axis("equal")
	plt.draw()
	plt.pause(0.00000000001)

	counter += 1

def imu_callback(msg):
	global gyroPitch, gyroRoll, gyroYaw, imuDataReceived, roll, pitch, yaw, msgTime

	msgTime = msg.header.stamp.nsecs/1000.0

	m = msg.orientation
	q = Quaternion(m.w,m.x,m.y,m.z)
	[roll, pitch, yaw] = quat2euler(*q, degrees=True)
	#print ("%1.2f, %1.2f, %1.2f" % (roll, pitch, yaw))
	
	g = msg.angular_velocity
	[gyroRoll,gyroPitch,gyroYaw] = np.degrees([g.x, g.y, g.z])
	#print ("%1.2f, %1.2f, %1.2f" % (gyroRoll, gyroPitch, gyroYaw))
	
	imuDataReceived = 1

if __name__=='__main__':
	rospy.init_node('imu_subscriber',anonymous=True)
	subImu=rospy.Subscriber('ardrone/imu', Imu, imu_callback)
	#sub=rospy.Subscriber('ardrone/ground_truth/imu', Imu, imu_callback)

	pub = rospy.Publisher('ardrone/command/motor_speed', Actuators, queue_size=10)

	pygame.init()
	pygame.joystick.init()
	joystick = pygame.joystick.Joystick(0)
	joystick.init()
	name = joystick.get_name()
	print(name)

	#buttons = joystick.get_numbuttons()
	#print(buttons)

	r = rospy.Rate(500) # 500hz 
	while not rospy.is_shutdown():
		pygame.event.pump()
		if joystick.get_button(1) and mode != STABILIZE: # if stabilize mode button (B) is pressed
			mode = STABILIZE
			print("Mode: STABILIZE")
		if joystick.get_button(0)  and mode != ALTHOLD: # if alt hold mode button (A) is pressed, not implemented yet, for future use only
			mode = ALTHOLD
			currentThr = rcthr
			print("Mode: ALT HOLD")

		if joystick.get_button(2) == 1: # if reset button (X) is pressed
			resetCmd = "rosservice call /gazebo/reset_world \"" + "{}\""
			print(resetCmd)
			os.system(resetCmd)
			#sys.cmd("rosservice call /gazebo/reset_world \"{}\"")
		rcthr = (float(joystick.get_axis(1))-1.0)*-500.0		# rcthr
		rcyaw = float(joystick.get_axis(0))*-180.0				# rcyaw
		rcpit = float(joystick.get_axis(4))*-30.0				# rcpit
		rcroll = float(joystick.get_axis(3))*30.0				# rcroll
		#print(str(rcthr) + "\t" + str(rcyaw) + "\t" + str(rcpit) + "\t" + str(rcroll))

		if mode == STABILIZE:
			if rcthr > 100 and imuDataReceived == 1:
				## Stablise PIDS
				pitch_stab_output = max(min(ps_pid.get_pid(rcpit - pitch, 1, msgTime), 25), -25)
				roll_stab_output  = max(min(rs_pid.get_pid(rcroll - roll, 1, msgTime), 25), -25)
				yaw_stab_output   = max(min(ys_pid.get_pid(wrap_180(yaw_target - yaw), 1, msgTime), 40), -40)
				#print('%s  %s  %s' % (rol_stab_out, pit_stab_out, yaw_stab_out))
				if(abs(rcyaw ) > 5):
					yaw_stab_output = rcyaw
					yaw_target = yaw

				## Rate PIDS
				pitch_output =  max(min(pr_pid.get_pid(pitch_stab_output - gyroPitch, 1, msgTime), 50), -50)
				roll_output  =  max(min(rr_pid.get_pid(roll_stab_output - gyroRoll, 1, msgTime), 50), -50)
				yaw_output   =  max(min(yr_pid.get_pid(yaw_stab_output - gyroYaw, 1, msgTime), 50), -50)

				motorVelFL = min(rcthr + roll_output - pitch_output + yaw_output, 1000)
				motorVelBL = min(rcthr + roll_output + pitch_output - yaw_output, 1000)
				motorVelFR = min(rcthr - roll_output - pitch_output - yaw_output, 1000)
				motorVelBR = min(rcthr - roll_output + pitch_output + yaw_output, 1000)
			
				msg = Actuators()
				msg.angular_velocities = [motorVelFR, motorVelBL, motorVelFL, motorVelBR]
				pub.publish(msg)

				imuDataReceived = 0
			else:
				rr_pid.reset_I()
				pr_pid.reset_I()
				yr_pid.reset_I()
				ps_pid.reset_I()
				rs_pid.reset_I()
				ys_pid.reset_I()
				yaw_target = yaw

		#elif mode == ALTHOLD: % For future use


		plt.ion()
		plt.show()
		r.sleep()	
	    #rospy.spinOnce()
