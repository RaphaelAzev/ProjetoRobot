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


CadernoDetectado = 0
cv_image = None

# Variáveis para permitir que o roda_todo_frame troque dados com a máquina de estados
media = []
centro = []
area = 0.0
#distancia = 0.0

#Variáveis dos features#
featuresIMG = cv2.imread("Caderno.jpeg",0)
detector= cv2.xfeatures2d.SIFT_create()
trainKP,trainDesc=detector.detectAndCompute(featuresIMG,None)
FLANN_INDEX_KDITREE=0
flannParam=dict(algorithm=FLANN_INDEX_KDITREE,tree=5)
flann=cv2.FlannBasedMatcher(flannParam,{})


tolerancia_x = 40
tolerancia_y = 20
ang_speed = 0.2
area_ideal = 75000 # área da distancia ideal do contorno - note que varia com a resolução da câmera
tolerancia_area = 18000

# Atraso máximo permitido entre a imagem sair do Turbletbot3 e chegar no laptop do aluno
atraso = 0.4E9
check_delay = True# Só usar se os relógios ROS da Raspberry e do Linux desktop estiverem sincronizados

def roda_todo_frame(imagem):
	print("frame")
	global cv_image
	global media
	global centro
	global area
	#global distancia
	global now

	now = rospy.get_rostime()
	imgtime = imagem.header.stamp
	lag = now-imgtime
	delay = lag.nsecs
	if delay > atraso and check_delay==True:
		return 
	try:
		antes = time.clock()
		cv_image = bridge.compressed_imgmsg_to_cv2(imagem, "bgr8")
		media, centro, area = cormodule.identifica_cor(cv_image) #distancia removida#
		#laser = le_scan.scaneou(dado)
		depois = time.clock()
		Features(cv_image)
		#HoughLinesCode(cv_image)
		cv2.imshow("Camera", cv_image)

	except CvBridgeError as e:
		print('ex', e)

def HoughLinesCode(frame):
    dst =cv2.Canny(frame,50,200)
    if True: # HoughLinesP
        lines = cv2.HoughLinesP(dst, 10, math.pi/180.0, 100, np.array([]), 5, 5)
        #print("Used Probabilistic Rough Transform")
        #print("The probabilistic hough transform returns the end points of the detected lines")
        a,b,c = lines.shape
        #print("Valor de A",a, "valor de lines.shape", lines.shape)
        for i in range(a):
            # Faz uma linha ligando o ponto inicial ao ponto final, com a cor vermelha (BGR)
            cv2.line(dst, (lines[i][0][0], lines[i][0][1]), (lines[i][0][2], lines[i][0][3]), (0, 0, 255), 3, cv2.LINE_AA)

    else:    # HoughLines
        # Esperemos nao cair neste caso
        lines = cv2.HoughLines(dst, 1, math.pi/180.0, 50, np.array([]), 0, 0)
        a,b,c = lines.shape
        for i in range(a):
            rho = lines[i][0][0]
            theta = lines[i][0][1]
            a = math.cos(theta)
            b = math.sin(theta)
            x0, y0 = a*rho, b*rho
            pt1 = ( int(x0+1000*(-b)), int(y0+1000*(a)) )
            pt2 = ( int(x0-1000*(-b)), int(y0-1000*(a)) )
            cv2.line(dst, pt1, pt2, (0, 0, 255), 3, cv2.LINE_AA)
        #print("Used old vanilla Hough transform")
        #print("Returned points will be radius and angles")




def scaneou(dado):
	global v90gra1
	#global v90graE
	#global menosparedeD	
	#print("Faixa valida: ", dado.range_min , " - ", dado.range_max )
	#print("Leituras:")
	#print(np.array(dado.ranges).round(decimals=2))
	#dists=(np.array(dado.ranges).round(decimals=2))
	dists = np.split(np.array(dado.ranges).round(decimals=2), 8)
	#print(dists[0])
	#print(dists[-1])
	v90gra = np.concatenate([dists[0],dists[-1]])
	#v45graD = dists[0]
	#v45graE = dists[-1]

	#if np.mean(v45graD) > np.mean(v45graE):
		#menosparedeD = True
	#else:
	#	menosparedeD = False

	#prnint("Intensities")
	#print(np.array(dado.intensities).round(decimals=2))
	#return np.array(dado.ranges).round(decimals=2)
	#listaL.append(dists[315:359])
	#listaL.append(dists[0:45])
	for t in v90gra:
		if t < 0.3 and t != 0:
			v90gra1 = True
		else: 
			v90gra1 = False



def Features(QueryImgBGR):
	global CadernoDetectado
	global featuresIMG
	global trainKP
	global trainDesc
	global detector
	global FLANN_INDEX_KDITREE
	global flannParam
	global flann


    MIN_MATCH_COUNT=50

    ########

    #FLANN_INDEX_KDITREE=0
    #flannParam=dict(algorithm=FLANN_INDEX_KDITREE,tree=5)
    #flann=cv2.FlannBasedMatcher(flannParam,{})

    CadernoDetectado = 0

    font = cv2.FONT_HERSHEY_SIMPLEX

    #print(QueryImgBGR)
    
    QueryImg=cv2.cvtColor(QueryImgBGR,cv2.COLOR_BGR2GRAY)
    queryKP,queryDesc=detector.detectAndCompute(QueryImg,None) ########
    matches=flann.knnMatch(queryDesc,trainDesc,k=2)   ########

    goodMatch=[]
    for m,n in matches:
        if(m.distance<0.75*n.distance):
            goodMatch.append(m)
    if(len(goodMatch)>MIN_MATCH_COUNT):
        tp=[]
        qp=[]
        for m in goodMatch:
            tp.append(trainKP[m.trainIdx].pt)
            qp.append(queryKP[m.queryIdx].pt)
        tp,qp=np.float32((tp,qp))
        H,status=cv2.findHomography(tp,qp,cv2.RANSAC,3.0)
        h,w=featuresIMG.shape
        trainBorder=np.float32([[[0,0],[0,h-1],[w-1,h-1],[w-1,0]]])
        queryBorder=cv2.perspectiveTransform(trainBorder,H)
        cv2.polylines(QueryImgBGR,[np.int32(queryBorder)],True,(0,255,0),5)
        CadernoDetectado = True

    else:
        CadernoDetectado = False
        
    #gray = cv2.medianBlur(cv2.cvtColor(cam.read()[1], cv2.COLOR_BGR2GRAY),5)
    
    if(CadernoDetectado==True):
        print("ACHOU!!")
    #if cv2.waitKey(10)==ord('q'):
       # break
        
    #cam.release()
    #cv2.destroyAllWindows()


## Classes - estados


class Girando(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['alinhou', 'girando', 'parado','features'])

    def execute(self, userdata):
		global velocidade_saida
		global start

		if media is None or len(media)==0:
			return 'girando'
		if CadernoDetectado ==True:
			return 'features'

		if  math.fabs(media[0]) > math.fabs(centro[0] + tolerancia_x):
			vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, -ang_speed))
			velocidade_saida.publish(vel)
			#rospy.sleep(0.1)
			rospy.sleep(0.1)
			return 'girando'
		if math.fabs(media[0]) < math.fabs(centro[0] - tolerancia_x):
			vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, ang_speed))
			velocidade_saida.publish(vel)
			#rospy.sleep(0.1)
			rospy.sleep(0.1)
			return 'girando'

		if area>7000:
			vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
			velocidade_saida.publish(vel)
			rospy.sleep(0.1)
			return 'alinhou'

		if (v90gra1 == True):
			return 'parado'

		else:
			vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
			velocidade_saida.publish(vel)
			rospy.sleep(0.1)
			return 'alinhou'


class Centralizado(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['alinhando', 'alinhado','parado'])

    def execute(self, userdata):
		global velocidade_saida
		global start


		if media is None:
			return 'alinhou'
		if  math.fabs(media[0]) > math.fabs(centro[0] + tolerancia_x) or area<6000:
			#vel = Twist(Vector3(-0.5, 0, 0), Vector3(0, 0, 0.5))
			#velocidade_saida.publish(vel)
			#return 'alinhado'
			return 'alinhando'
		if math.fabs(media[0]) < math.fabs(centro[0] - tolerancia_x) or area<6000:
			#vel = Twist(Vector3(-0.5, 0, 0), Vector3(0, 0, -0.5))
			#velocidade_saida.publish(vel)
			#return 'alinhado'
			return 'alinhando'
		else:
			if (v90gra1 == True):
				start=rospy.get_rostime()
				return 'parado'
			elif area>=5000:
				vel = Twist(Vector3(-0.5, 0, 0), Vector3(0, 0, 0))
				velocidade_saida.publish(vel)
				rospy.sleep(0.1)
				return 'alinhado'
			else:
				vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
				velocidade_saida.publish(vel)
				rospy.sleep(0.1)
				return 'alinhado'

class Parado(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['andando', 'parado','alinhando'])

    def execute(self, userdata):
		global velocidade_saida
		global start

		if media is None:
			return 'parado'
		'''if (v90gra1 == True):
			vel= Twist(Vector3(4,0,0),Vector3(0,0,0))
			velocidade_saida.publish(vel)
			#rospy.sleep(2)
			#if (v90gra1 == False):
			vel= Twist(Vector3(0,0,0),Vector3(0,0,6))
			velocidade_saida.publish(vel)
			#rospy.sleep(2)
			#	return 'alinhando'
			vel= Twist(Vector3(-2,0,0),Vector3(0,0,0))
			velocidade_saida.publish(vel)
			#rospy.sleep()
xs
			#rospy.sleep(10)

			return 'parado'''
		if (v90gra1 == True):
			start = rospy.get_rostime()
			repor = rospy.Duration(0.15)
			girepor = rospy.Duration(0.05)
			andepor = rospy.Duration(0.2)
			tempopassado = now-start
			while tempopassado < repor:
				vel = Twist(Vector3(3,0,0),Vector3(0,0,0))
				velocidade_saida.publish(vel)
				tempopassado = now-start
			start=rospy.get_rostime()
			tempopassado = now-start
			while tempopassado < girepor:
				vel = Twist(Vector3(0,0,0),Vector3(0,0,2))
				velocidade_saida.publish(vel)
				tempopassado = now-start
			start = rospy.get_rostime()
			tempopassado = now-start
			while tempopassado<andepor:	
				vel = Twist(Vector3(-3,0,0),Vector3(0,0,0))
				velocidade_saida.publish(vel)
				tempopassado = now-start
			start=rospy.get_rostime()
			tempopassado = now-start
			return 'alinhando'

			#return 'parado'	
		if  math.fabs(media[0]) > math.fabs(centro[0] + tolerancia_x) or area<6000:
			return 'alinhando'
		if math.fabs(media[0]) < math.fabs(centro[0] - tolerancia_x) or area<6000:
			return'alinhando'
		elif (v90gra1 == False) and area>5000:
			return 'andando'
		elif (v90gra1 == False) and area<5000:
			vel= Twist(Vector3(0,0,0),Vector3(0,0,5))
			velocidade_saida.publish(vel)
			rospy.sleep(0.1)
			start=rospy.get_rostime()
			return 'parado'

class FeaturesReact(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['girando'])

    def execute(self, userdata):
		global velocidade_saida
		global start
		start = rospy.get_rostime()
		girepor = rospy.Duration(0.5)
		tempopassado = now-start
		while tempopassado < repor:
			vel = Twist(Vector3(0,0,0),Vector3(0,0,10))
			velocidade_saida.publish(vel)
			tempopassado = now-start
		start=rospy.get_rostime()
		return "girando"


# main
def main():
	global velocidade_saida
	global buffer
	rospy.init_node('cor_estados')

	# Para usar a webcam 
	#recebedor = rospy.Subscriber("/cv_camera/image_raw/compressed", CompressedImage, roda_todo_frame, queue_size=1, buff_size = 2**24)
	
	recebe_scan = rospy.Subscriber("/scan", LaserScan, scaneou)#, queue_size=1)

	recebedor = rospy.Subscriber("/raspicam_node/image/compressed", CompressedImage, roda_todo_frame, queue_size=1, buff_size = 2**24)
	
	#caderno = rospy.Subscriber("/raspicam_node/image/compressed", CompressedImage, Features, queue_size=1, buff_size = 2**24)

	velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)

	# Create a SMACH state machineFeatures
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
	                            'alinhou':'CENTRO', 'parado':'PARADO','features':'FEATURES'})
	    smach.StateMachine.add('CENTRO', Centralizado(),
	                            transitions={'alinhando': 'GIRANDO',
	                            'alinhado':'CENTRO','parado':'PARADO'})
	    smach.StateMachine.add('PARADO', Parado(),
	                            transitions={'andando': 'CENTRO',
	                            'parado':'PARADO','alinhando':'GIRANDO'})
	    smach.StateMachine.add('FEATURES', FeaturesReact(),
	                            transitions={'girando': 'CENTRO'})	    
	# Execute SMACH plan
	outcome = sm.execute()
	#rospy.spin()

if __name__ == '__main__':

	main()