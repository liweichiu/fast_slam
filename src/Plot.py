#!/usr/bin/env python

import rospy
import math
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse 
import numpy as np
from numpy import linalg as LA
from std_msgs.msg import Float64MultiArray

PI = 3.1415926
s = 5.9915

zoom = 1
plt_get = False
temp = 0

pose = np.array([2,0,PI/2])
LM_X = np.zeros(30)
LM_Y = np.zeros(30)
theta = 0
for i in range(18):
	LM_X[i] = 2.5 * math.cos(theta)
	LM_Y[i] = 2.5 * math.sin(theta)
	theta += 20.0 / 180.0 * PI
theta = 0
for i in range(18,30):
	LM_X[i] = 1.5 * math.cos(theta)
	LM_Y[i] = 1.5 * math.sin(theta)
	theta += 30.0 / 180.0 * PI
'''
fig, ax = plt.subplots()
(ln,) = ax.plot(LM_X,LM_Y,'b^',animated=True)
(ln1,) = ax.plot(pose[0],pose[1],'ro',animated=True)
(ln2,) = ax.plot([pose[0],pose[0]+0.5*math.cos(pose[2])],[pose[1],pose[1]+0.5*math.sin(pose[2])],'r',animated=True)
ax.axis('equal')
plt.show(block=False)
plt.pause(0.01)
bg = fig.canvas.copy_from_bbox(fig.bbox)
ax.draw_artist(ln)
fig.canvas.blit(fig.bbox)
'''


def callback(msg):
	pose[0] = msg.data[0]
	pose[1] = msg.data[1]	
	pose[2] = msg.data[2]
	global plt_
	plt_ = np.zeros(len(msg.data)-3)
	for i in range(len(msg.data)-3):
		plt_[i] = msg.data[3+i]
	global plt_get
	plt_get = True
	

if __name__ == '__main__':
	rospy.init_node('Plot', anonymous=True)
	rospy.Subscriber('status',Float64MultiArray,callback)

	rate = rospy.Rate(100) # 100hz
	while not rospy.is_shutdown():
		'''
		fig.canvas.restore_region(bg)
		ln1.set_xdata(pose[0])
		ln1.set_ydata(pose[1])
		ln2.set_xdata([pose[0],pose[0]+0.5*math.cos(pose[2])])
		ln2.set_ydata([pose[1],pose[1]+0.5*math.sin(pose[2])])
		ax.draw_artist(ln)
		ax.draw_artist(ln1)
		ax.draw_artist(ln2)
		#e = Ellipse((10,10),10,50,animated=True)
		#ax.draw_artist(e)
		fig.canvas.blit(fig.bbox)
		fig.canvas.flush_events()
		'''
		plt.clf()
		plt.plot(LM_X,LM_Y,'b^',pose[0],pose[1],'ro',[pose[0],pose[0]+0.5*math.cos(pose[2])],[pose[1],pose[1]+0.5*math.sin(pose[2])],'r')
		
		if plt_get == True:
			for i in range(plt_.size/6):
				cor = np.array([[plt_[6*i+2],plt_[6*i+3]],[plt_[6*i+4],plt_[6*i+5]]])
				w,v = LA.eig(cor)	
				plt.axes().add_artist(Ellipse((plt_[6*i],plt_[6*i+1]),zoom*2*math.sqrt(s*np.amin(w)),zoom*2*math.sqrt(s*np.amax(w)),angle=180/PI*math.atan2(v[np.argmax(w)][0],v[np.argmax(w)][1]),edgecolor='r',facecolor='none'))
	
		plt.axis('equal')
		plt.draw()
		plt.pause(0.0001)
		
		rospy.loginfo(rospy.get_time()-temp)
		temp = rospy.get_time()-temp
		
		rate.sleep()
	
