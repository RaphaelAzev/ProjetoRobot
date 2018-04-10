#! /usr/bin/env python
# -*- coding:utf-8 -*-

__author__ = ["Rachel P. B. Moraes", "Igor Montagner", "Fabio Miranda"]


import rospy
import numpy as np
import tf
import math
import cv2
import time
from geometry_msgs.msg import Twist, Vector3, Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import smach
import smach_ros
import le_scan
from sensor_msgs.msg import LaserScan

import cormodule

bridge = CvBridge()

cv_image = None

# Variáveis para permitir que o roda_todo_frame troque dados com a máquina de estados
media = []
centro = []
area = 0.0
distancia = 0.0



tolerancia_x = 40
tolerancia_y = 20
ang_speed = 0.2
area_ideal = 75000 # área da distancia ideal do contorno - note que varia com a resolução da câmera
tolerancia_area = 18000

# Atraso máximo permitido entre a imagem sair do Turbletbot3 e chegar no laptop do aluno
atraso = 1.5
check_delay = True# Só usar se os relógios ROS da Raspberry e do Linux desktop estiverem sincronizados

def roda_todo_frame(imagem):
	print("frame")
	global cv_image
	global media
	global centro
	global area
	global distancia

	now = rospy.get_rostime()
	imgtime = imagem.header.stamp
	lag = now-imgtime
	delay = lag.secs
	if delay > atraso and check_delay==True:
		return 
	try:
		antes = time.clock()
		cv_image = bridge.compressed_imgmsg_to_cv2(imagem, "bgr8")
		media, centro, area, distancia = cormodule.identifica_cor(cv_image)
		#laser = le_scan.scaneou(dado)
		depois = time.clock()
		cv2.imshow("Camera", cv_image)
	except CvBridgeError as e:
		print('ex', e)
	
def scaneou(dado):
	global v90gra1	
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
	for t in v90gra:
		if t < 0.4 and t != 0:
			v90gra1 = True
		else: 
			v90gra1 = False

## Classes - estados


class Girando(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['alinhou', 'girando'])

    def execute(self, userdata):
		global velocidade_saida

		if media is None or len(media)==0:
			return 'girando'

		if  math.fabs(media[0]) > math.fabs(centro[0] + tolerancia_x):
			vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, -ang_speed))
			velocidade_saida.publish(vel)
			rospy.sleep(0.1)
			return 'girando'
		if math.fabs(media[0]) < math.fabs(centro[0] - tolerancia_x):
			vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, ang_speed))
			velocidade_saida.publish(vel)
			rospy.sleep(0.1)
			return 'girando'

		if area>7000:
			vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
			velocidade_saida.publish(vel)
			return 'alinhou'

		else:
			vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
			velocidade_saida.publish(vel)
			return 'alinhou'


class Centralizado(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['alinhando', 'alinhado','parado'])

    def execute(self, userdata):
		global velocidade_saida

		if media is None:
			return 'alinhou'
		if  math.fabs(media[0]) > math.fabs(centro[0] + tolerancia_x) or area<6000:
			return 'alinhando'

		if math.fabs(media[0]) < math.fabs(centro[0] - tolerancia_x) or area<6000:
			return 'alinhando'
		else:
			if (v90gra1 == True):
				return 'parado'
			elif area>=5000:
				vel = Twist(Vector3(-0.5, 0, 0), Vector3(0, 0, 0))
				velocidade_saida.publish(vel)
				return 'alinhado'
			else:
				vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
				velocidade_saida.publish(vel)
				return 'alinhado'

class Parado(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['andando', 'parado','alinhando'])

    def execute(self, userdata):
		global velocidade_saida

		if media is None:
			return 'parado'
		if (v90gra1 == True):
			vel= Twist(Vector3(3,0,0),Vector3(0,0,0))
			velocidade_saida.publish(vel)
			rospy.sleep(0.1)
			#if (v90gra1 == False):
			#	vel= Twist(Vector3(0,0,0),Vector3(0,0,5))
			#	velocidade_saida.publish(vel)
			#	return 'alinhando'

			return 'parado'		
		if  math.fabs(media[0]) > math.fabs(centro[0] + tolerancia_x) or area<6000:
			return 'alinhando'
		if math.fabs(media[0]) < math.fabs(centro[0] - tolerancia_x) or area<6000:
			return'alinhando'
		elif (v90gra1 == False) and area>5000:
			return 'andando'


# main
def main():
	global velocidade_saida
	global buffer
	rospy.init_node('cor_estados')

	# Para usar a webcam 
	#recebedor = rospy.Subscriber("/cv_camera/image_raw/compressed", CompressedImage, roda_todo_frame, queue_size=1, buff_size = 2**24)
	
	recebe_scan = rospy.Subscriber("/scan", LaserScan, scaneou, queue_size=3)

	recebedor = rospy.Subscriber("/raspicam_node/image/compressed", CompressedImage, roda_todo_frame, queue_size=5, buff_size = 2**24)
	
	velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)

	# Create a SMACH state machine
	sm = smach.StateMachine(outcomes=['terminei'])

	# Open the container
	with sm:
	    # Add states to the container
	    #smach.StateMachine.add('LONGE', Longe(), 
	    #                       transitions={'ainda_longe':'ANDANDO', 
	    #                                    'perto':'terminei'})
	    #smach.StateMachine.add('ANDANDO', Andando(), 
	    #                       transitions={'ainda_longe':'LONGE'})
	    smach.StateMachine.add('GIRANDO', Girando(),
	                            transitions={'girando': 'GIRANDO',
	                            'alinhou':'CENTRO'})
	    smach.StateMachine.add('CENTRO', Centralizado(),
	                            transitions={'alinhando': 'GIRANDO',
	                            'alinhado':'CENTRO','parado':'PARADO'})
	    smach.StateMachine.add('PARADO', Parado(),
	                            transitions={'andando': 'CENTRO',
	                            'parado':'PARADO','alinhando':'GIRANDO'})
	# Execute SMACH plan
	outcome = sm.execute()
	#rospy.spin()


if __name__ == '__main__':

	main()
