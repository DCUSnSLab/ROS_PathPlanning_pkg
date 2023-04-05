#!/usr/bin/python2
# -*- coding: utf-8 -*-

import rospy

from sensor_msgs.msg import NavSatFix

class Converter():
	def __init__(self):
		print("Converter")
		rospy.init_node("GPS_Converter")
		self.sub = rospy.Subscriber('/gps_data', NavSatFix, self.convert_callback, queue_size=1)
		self.pub = rospy.Publisher('/converted_gps_data', NavSatFix, queue_size=1)
		rospy.spin()

	def convert_callback(self, data):
		origin_latitude = data.latitude
		origin_longitude = data.longitude

		data.latitude = int(str(origin_latitude)[0:2]) + ((int(str(origin_latitude)[2:4]) + origin_latitude - int(origin_latitude)) / 60)
		data.longitude = int(str(origin_longitude)[0:3]) + ((int(str(origin_longitude)[3:5]) + origin_longitude - int(origin_longitude)) / 60)

		self.pub.publish(data)

if __name__=="__main__":
	try:
		Converter()
	except:
		pass