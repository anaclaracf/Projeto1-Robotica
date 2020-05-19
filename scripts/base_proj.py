#! /usr/bin/env python
# -*- coding:utf-8 -*-
 
from __future__ import print_function, division
import rospy
import numpy as np 
import numpy 
import tf
import math
import cv2
import time 
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
from numpy import linalg
from tf import transformations
from tf import TransformerROS
import tf2_ros 
from geometry_msgs.msg import Twist, Vector3, Pose, Vector3Stamped
from ar_track_alvar_msgs.msg import AlvarMarker, AlvarMarkers
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
 
 
import visao_module

 
bridge = CvBridge()
bx=0
cv_image = None
imagem = None
media = []
centro = None
maior_area=0
reconheceu=False
pegou_creeper=False
atraso = 1.5E9 # 1 segundo e meio. Em nanossegundos


area = 0.0 # Variavel com a area do maior contorno

# Só usar se os relógios ROS da Raspberry e do Linux desktop estiverem sincronizados. 
# Descarta imagens que chegam atrasadas demais
check_delay = False 

resultados = [] # Criacao de uma variavel global para guardar os resultados vistos
cx=0
bx=None
x = 0
y = 0
z = 0 
id = 0

frame = "camera_link"
# frame = "head_camera"  # DESCOMENTE para usar com webcam USB via roslaunch tag_tracking usbcam

tfl = 0

tf_buffer = tf2_ros.Buffer()

# corr=str(input("Qual cor (pink, red, green)?"))

 
def recebe(msg):
    # Para forçar maior atualização dos sistemas de coordenadas 
    # roslaunch turtlebot3_gazebo turtlebot3_gazebo_rviz.launch
	global x # O global impede a recriacao de uma variavel local, para podermos usar o x global ja'  declarado
	global y
	global z
	global id
	for marker in msg.markers:
		id = marker.id
		marcador = "ar_marker_" + str(id)

		#print(tf_buffer.can_transform(frame, marcador, rospy.Time(0)))
		header = Header(frame_id=marcador)
		# Procura a transformacao em sistema de coordenadas entre a base do robo e o marcador numero 100
		# Note que para seu projeto 1 voce nao vai precisar de nada que tem abaixo, a 
		# Nao ser que queira levar angulos em conta
		trans = tf_buffer.lookup_transform(frame, marcador, rospy.Time(0))
		
		# Separa as translacoes das rotacoes
		x = trans.transform.translation.x
		y = trans.transform.translation.y
		z = trans.transform.translation.z
		# ATENCAO: tudo o que vem a seguir e'  so para calcular um angulo
		# Para medirmos o angulo entre marcador e robo vamos projetar o eixo Z do marcador (perpendicular) 
		# no eixo X do robo (que e'  a direcao para a frente)
		t = transformations.translation_matrix([x, y, z])
		# Encontra as rotacoes e cria uma matriz de rotacao a partir dos quaternions
		r = transformations.quaternion_matrix([trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w])
		m = numpy.dot(r,t) # Criamos a matriz composta por translacoes e rotacoes
		z_marker = [0,0,1,0] # Sao 4 coordenadas porque e'  um vetor em coordenadas homogeneas
		v2 = numpy.dot(m, z_marker)
		v2_n = v2[0:-1] # Descartamos a ultima posicao
		n2 = v2_n/linalg.norm(v2_n) # Normalizamos o vetor
		x_robo = [1,0,0]
		cosa = numpy.dot(n2, x_robo) # Projecao do vetor normal ao marcador no x do robo
		angulo_marcador_robo = math.degrees(math.acos(cosa))

		# Terminamos
		# print("id: {} x {} y {} z {} angulo {} ".format(id, x,y,z, angulo_marcador_robo))



# A função a seguir é chamada sempre que chega um novo frame
def roda_todo_frame(imagem):
    global cv_image
    global media
    global centro
    global maior_area
    global resultados
    global pink
    global red 
    global green
    global bx
    global reconheceu
    # global imagem
    
    now = rospy.get_rostime()
    imgtime = imagem.header.stamp
    lag = now-imgtime # calcula o lag 
    delay = lag.nsecs
    # print("delay ", "{:.3f}".format(delay/1.0E9))
    if delay > atraso and check_delay==True:
        print("Descartando por causa do delay do frame:", delay)
        return 
    try:
        antes = time.clock()
        cv_image = bridge.compressed_imgmsg_to_cv2(imagem, "bgr8")
        # Note que os resultados já são guardados automaticamente na variável
        # chamada resultados
        green="green"
        red="red"
        pink = "pink"
        bicycle= "bicycle"
        dog = "dog"
        cat = "cat"
        bird = "bird"
        centro, imagem, resultados, reconheceu =  visao_module.processa(cv_image,dog )
        maior_area,bx = visao_module.identifica_cor(cv_image,green)

        #qualquer, centro, maior_area, media =  visao_module.identifica_cor(cv_image)

        for r in resultados:
            # print(r) - print feito para documentar e entender
            # o resultado            
            pass

        depois = time.clock()
        # Desnecessário - Hough e MobileNet já abrem janelas
        # cv2.imshow("Camera", cv_image)
        #cv2.waitKey(2)

    except CvBridgeError as e:
        print('ex', e)



if __name__=="__main__":
    rospy.init_node("base_proj")

    topico_imagem = "/camera/rgb/image_raw/compressed"

    recebedor = rospy.Subscriber(topico_imagem, CompressedImage, roda_todo_frame, queue_size=4, buff_size = 2**24)
    recebe_marker = rospy.Subscriber("/ar_pose_marker", AlvarMarkers, recebe) # Para recebermos notificacoes de que marcadores foram vistos


    print("Usando ", topico_imagem)
    
    velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)

    tfl = tf2_ros.TransformListener(tf_buffer) #conversao do sistema de coordenadas 
    tolerancia = 25

    # Exemplo de categoria de resultados
    # [('chair', 86.965459585189819, (90, 141), (177, 265))]

    try:
        # IniciTypeError: alizando - por default gira no sentido anti-horário
        # vel = Twist(Vector3(0,0,0), Vector3(0,0,math.pi/10.0))
        
        # lista_velx=[]
        
        while not rospy.is_shutdown():
            reconheceu2=reconheceu
            for r in resultados:
                print(r)
            #velocidade_saida.publish(vel)

            # if cv_image is not None:
                # Note que o imshow precisa ficar *ou* no codigo de tratamento de eventos *ou* no thread principal, não em ambos
                # cv2.imshow("cv_image no loop principal", cv_image)
                # cv2.waitKey(1)
                # rospy.sleep(0.1)
            if cv_image is not None:
                
                img_rgb1 = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
                hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
                amarelo1 = numpy.array([20,  50,  50])
                amarelo2 = numpy.array([30, 255, 255])
                mask = cv2.inRange(hsv, amarelo1, amarelo2)

                # Mr = cv2.moments(mask_azul)
                # if Mr['m00'] > 0:
                #     bx = int(Mr['m10']/Mr['m00'])
                #     by = int(Mr['m01']/Mr['m00'])
                #     # print("RX",rx)
                #     # lista_velx.append(cx)
                #     # print(centro)
                #     cv2.circle(cv_image, (bx, by), 20, (255,0,0), -1)
 

                M = cv2.moments(mask)
                if M['m00'] > 0:
                    cx = int(M['m10']/M['m00'])
                    cy = int(M['m01']/M['m00'])
                    #print('Cx', cx)
                    # lista_velx.append(cx)
                    # print(centro)
                    cv2.circle(cv_image, (cx, cy), 20, (0,0,255), -1)

                h, w, d = cv_image.shape
                search_top = int(3*h/4)
                search_bot = search_top + 20
                mask[0:search_top, 0:w] = 0
                mask[search_bot:h, 0:w] = 0

                cv2.imshow("Robot Cam", cv_image)
                # cv2.imshow("Cam rosa", mask_azul)
                # cv2.imshow("Cam amarela", mask)
                if visao_module.debug_frame is not None:
                    cv2.imshow("Debug Frame", visao_module.debug_frame)
                print("AREA", maior_area)
                print("ID",id)
            
                cv2.waitKey(4)
                vel = Twist(Vector3(0.095, 0, 0), Vector3(0, 0, 0))
                velocidade_saida.publish(vel)
                rospy.sleep(0.1)
                if centro is not None :
                    if maior_area == None or maior_area < 7000: 
                        if (centro[0] < cx):
                            vel = Twist(Vector3(0,0,0), Vector3(0,0,-0.1))
                            # vel = Twist(Vector3(0.05, 0, 0), Vector3(0, 0, 0))
                            print('focou amarelo')
                        elif (centro[0]>cx):
                            vel = Twist(Vector3(0,0,0), Vector3(0,0,0.1))
                            # vel = Twist(Vector3(0.05, 0, 0), Vector3(0, 0, 0))
                            print('focou amarelo')
                        
                    elif maior_area >= 7000 and maior_area < 170000:
                        print("entrou area")
                        if (centro[0] < bx):
                            vel = Twist(Vector3(0,0,0), Vector3(0,0,-0.1))
                            # vel = Twist(Vector3(0.05, 0, 0), Vector3(0, 0, 0))
                            print('focou creeper')
                        elif (centro[0] > bx):
                            vel = Twist(Vector3(0,0,0), Vector3(0,0,0.1))
                            # vel = Twist(Vector3(0.05, 0, 0), Vector3(0, 0, 0))
                            print("focou creeper")
                    elif maior_area >= 170000 and not pegou_creeper:
                        vel = Twist(Vector3(0,0,0), Vector3(0,0,0))
                        velocidade_saida.publish(vel)
                        raw_input("enter")
                        pegou_creeper=True
                        vel = Twist(Vector3(0,0,0), Vector3(0,0,-0.4))
                        velocidade_saida.publish(vel)
                        rospy.sleep(1.5)
                        print('focou creeper e PAROU')

                    print("Se reconheceu creeper: ",reconheceu2)
                    print ("Se pegou creeper (fora do if): ", pegou_creeper)
                    if reconheceu2 and pegou_creeper:
                        print ("Se pegou creeper: ", pegou_creeper)
                        print (" UHUUUUU Reconheceu a base")
                        vel = Twist(Vector3(0,0,0), Vector3(0,0,0))
                        velocidade_saida.publish(vel)
                        raw_input("enter")  

                    velocidade_saida.publish(vel)
                    rospy.sleep(0.1)
            

    except rospy.ROSInterruptException:
        print("Ocorreu uma exceção com o rospy")

