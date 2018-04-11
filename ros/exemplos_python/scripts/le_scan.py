#! /usr/bin/env python
# -*- coding:utf-8 -*-

import rospy

import numpy as np

from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan


def scaneou(dado):
	#print("Faixa valida: ", dado.range_min , " - ", dado.range_max )
	#print("Leituras:")
	#print(np.array(dado.ranges).round(decimals=2))
	#dists=(np.array(dado.ranges).round(decimals=2))
	dists = np.split(np.array(dado.ranges).round(decimals=2), 8)
	#print(dists[0])
	#print(dists[-1])
	v90gra = np.concatenate([dists[0],dists[-1]])
	#prnint("Intensities")
	#print(np.array(dado.intensities).round(decimals=2))
	#return np.array(dado.ranges).round(decimals=2)
	#listaL.append(dists[315:359])
	#listaL.append(dists[0:45])
	'''for t in v90gra:
		if t < 0.5 and t != 0:
			velocidade = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
			velocidade_saida.publish(velocidade)
			rospy.sleep(1)
			velocidade = Twist(Vector3(0.5, 0, 0), Vector3(0, 0, 0))
			velocidade_saida.publish(velocidade)
			rospy.sleep(1)
		else: 
			velocidade = Twist(Vector3(-0.5, 0, 0), Vector3(0, 0, 0))
			velocidade_saida.publish(velocidade)'''
	print(rospy.Time())
	return v90gra


	#dists315 a 360
	#dists0 a 45


if __name__=="__main__":

	rospy.init_node("le_scan")

	velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 3 )
	recebe_scan = rospy.Subscriber("/scan", LaserScan, scaneou)


	while not rospy.is_shutdown():
		print(recebe_scan)
		#velocidade = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
		#velocidade_saida.publish(velocidade)
		rospy.sleep(2)


