#!/usr/bin/env python
from __future__ import print_function
import rospy
import roslib
# roslib.load_manifest('my_package')
import sys
import cv2
import numpy as np


from std_msgs.msg import String
from std_msgs.msg import Bool
from std_msgs.msg import Int8
from signs_sender.msg import DetectObj
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

traffic_size = 200
speed_size = 200
acc_size = 200
con1_size = 200
con2_size = 200

#!/usr/bin/env python3.6
import darknet
net = darknet.load_net("/home/kroad/catkin_ws/src/signs_sender/scripts/test/yolov4-custom.cfg".encode('utf-8'),
                        "/home/kroad/catkin_ws/src/signs_sender/scripts/test/yolov4-custom_best.weights".encode('utf-8'), 0)
meta = darknet.load_meta("/home/kroad/catkin_ws/src/signs_sender/scripts/test/obj.data".encode('utf-8'))

def image_callback(data):
    global cv_image
    try:
        cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
        print(error)

    # (rows,cols,channels) = cv_image.shape
    #print("rows: %d, cols: %d" %(rows, cols))
    # if cols > 60 and rows > 60 :
    #     cv2.circle(cv_image, (50,50), 10, 255)
    # cv2.imshow("Image window", cv_image)
    cv2.waitKey(3)


def retbox(detections,i,frame) :
    label = detections[i][0].decode('utf-8')
    score = detections[i][1]
    #print(label, score)
    x1 = int(round((detections[i][2][0]) - (detections[i][2][2]/2.0))) # top left x1 
    y1 = int(round((detections[i][2][1]) - (detections[i][2][3]/2.0))) # top left xy 
    x2 = int(round((detections[i][2][0]) + (detections[i][2][2]/2.0))) # bottom right x2 
    y2 = int(round((detections[i][2][1]) + (detections[i][2][3]/2.0))) # bottom right y2 
    area = int(abs(x1 - x2) * abs(y1 - y2))
    BB_center_x = detections[i][2][0]
    BB_center_y = detections[i][2][1]
    #print("area: %d" %(area))
    #print("BB_center_x: %f" %(BB_center_x))
    #print("BB_center_y: %f" %(BB_center_y))



    box = np.array([x1,y1,x2,y2])
    #print("box: %d"%(len(box)))
    #print(box)
    return label, score, box, area, BB_center_x, BB_center_y


def detection(sign, cap2, threshold):
    #global cap
    #frame = cv2.resize(frame, dsize=(640, 480), interpolation=cv2.INTER_AREA)
    detections = darknet.detect_image(net,meta, cap2 , thresh=.1)

    #cv2.rectangle(cap2, (1,1),(100,100), (255,0,0), 10)
    #print("length: %d"%(len(detections)))

    
######detected num######
    traffic_num = 0
    speed_num = 0
    acc_num = 0
    con1_num = 0
    con2_num = 0

######detected info######
    labels = []
    scores = []
    areas = []
    BBx_sign = []
    BBy_sign = []
######detected BB size####
    traffic_area = [0]*100
    speed_area = [0]*100
    acc_area = [0]*100
    con1_area = [0]*100
    con2_area = [0]*100

###########devide group for area###############
    
    for i in range(len(detections)) :
        label , score , box, area, BB_center_x, BB_center_y  = retbox(detections,i,cap2)
        labels.append(label)
        scores.append(score)
        areas.append(area)
        BBx_sign.append(BB_center_x)
        BBy_sign.append(BB_center_y)

        if labels[i] == "green" or labels[i] == "red":
            traffic_area[traffic_num] = areas[i]
            traffic_num+=1
        elif labels[i] == "30km/h" or labels[i] == "40km/h" or labels[i] == "50km/h" or labels[i] == "10km/h" or labels[i] == "15km/h" or labels[i] == "20km/h" or labels[i] == "25km/h" or labels[i] == "35km/h" or labels[i] == "45km/h":
            speed_area[speed_num] = areas[i]
            speed_num+=1
        elif labels[i] == "accident":
            acc_area[acc_num] = areas[i]
            acc_num+=1
        elif labels[i] == "construction1":
            con1_area[con1_num] = areas[i]
            con1_num+=1
        elif labels[i] == "construction2":
            con2_area[con2_num] = areas[i]
            con2_num+=1

        #left,top,right,bottom=box
        cv2.rectangle(cap2, (box[0],box[1]),(box[2],box[3]), (255,0,255), 7) 
        cv2.putText(cap2, labels[i], (box[0],box[1]), cv2.FONT_HERSHEY_PLAIN, 0.8, (0, 255, 0))
        
########max BB size###########
    traffic_max = 0
    speed_max = 0
    acc_max = 0
    con1_max = 0
    con2_max = 0

########distance of BB##########
    traffic_dist = 0
    speed_dist = 0
    acc_dist = 0
    con1_dist = 0
    con2_dist = 0

##########class###########
    traffic_sign = 0
    speed_sign = 0
    acc_sign = 0
    con_sign = 0
    
##########center point#######

    traffic_result = DetectObj()
    speed_result = DetectObj()
    acc_result = DetectObj()
    con_result = DetectObj()


    

    for j in range(len(detections)):
####################traffic_sign########################
        if labels[j] == "green" or labels[j] == "red":
            traffic_max = traffic_area[0]
            for num in traffic_area:
                if traffic_max < num:
                    traffic_max = num
                    
            
            #traffic_dist  = traffic_max/100 #distance of detected obj
            #print("traffic_max: %d"%(traffic_max))

            if(traffic_max > traffic_size):
                if labels[j] == "green":
                    traffic_sign = 1
                elif labels[j] == "red":
                    traffic_sign = 2

            traffic_result.obj = traffic_sign
            traffic_result.BB_pointX = BBx_sign[j]
            traffic_result.BB_pointY = BBy_sign[j]
####################speed_sign########################
        elif labels[j] == "30km/h" or labels[j] == "40km/h" or labels[j] == "50km/h":
            speed_max = speed_area[0]
            for num in speed_area:
                if speed_max < num:
                    speed_max = num
            speed_dist  = speed_max/100 #distance of detected obj
            #print("speed_max: %d"%(speed_max))

            if(speed_max >speed_size):
                if labels[j] == "30km/h":
                    speed_sign = 1
                elif labels[j] == "40km/h":
                    speed_sign = 2
                elif labels[j] == "50km/h":
                    speed_sign = 3
            
            speed_result.obj = speed_sign
            speed_result.BB_pointX = BBx_sign[j]
            speed_result.BB_pointY = BBy_sign[j]
####################accident_sign########################
        elif labels[j] == "accident":
            acc_max = acc_area[0]
            for num in acc_area:
                if acc_max < num:
                    acc_max = num
            acc_dist  = acc_max/100 #distance of detected obj
            #print("acc_max: %d"%(acc_max))

            if(acc_max >acc_size):
                acc_sign = 1

            acc_result.obj = acc_sign
            acc_result.BB_pointX = BBx_sign[j]
            acc_result.BB_pointY = BBy_sign[j]
####################construction_sign########################
        elif labels[j] == "construction1":
            con1_max = con1_area[0]
            for num in con1_area:
                if con1_max < num:
                    con1_max = num
            con1_dist  = con1_max/100 #distance of detected obj
            #print("con1_max: %d"%(con1_max))

            if(con1_max >con1_size):
                con_sign = 1
            
        elif labels[j] == "construction2":
            con2_max = con2_area[0]
            for num in con2_area:
                if con2_max < num:
                    con2_max = num
            con2_dist  = con2_max/100 #distance of detected obj
            #print("con2_max: %f"%(con2_max))

            if(con2_max >con2_size):
                con_sign = 1

            con_result.obj = con_sign
            con_result.BB_pointX = BBx_sign[j]
            con_result.BB_pointY = BBy_sign[j]

        
    return traffic_result, speed_result, acc_result, con_result, cap2

def talker(sign, threshold):
   
    rospy.init_node('yoloyolo', anonymous = True)
    # bridge = CvBridge()

    image_sub = rospy.Subscriber("/vds_node_localhost_2210/image_raw", Image, image_callback, queue_size=1)
    
    traffic_pub = rospy.Publisher('traffic_result', DetectObj, queue_size= 1)
    speed_pub = rospy.Publisher('speed_result', DetectObj, queue_size= 1)
    acc_pub = rospy.Publisher('acc_result', DetectObj, queue_size= 1)
    con_pub = rospy.Publisher('con_result', DetectObj, queue_size= 1)

    ####################################################result
    ##result_pub = rospy.Publisher('final_result', sw, queue_size=10)


    rate = rospy.Rate(30) #30hz
    while not rospy.is_shutdown():

        global cv_image
        cap1 = cv_image

        traffic_result, speed_result, acc_result, con_result, frames = detection(sign, cap1, threshold)

        cv2.imshow("Image window", frames)
        
        #rospy.loginfo("%d", sign)
        ###########publisher################
        traffic_pub.publish(traffic_result)
        #speed_pub.publish(speed_result)
        acc_pub.publish(acc_result)
        #con_pub.publish(con_result)
        

        ####################################################result
        print("###########################")
        print("traffic")
        print(traffic_result.obj)
        print(traffic_result.BB_pointX)
        print(traffic_result.BB_pointY)
        print("--------------------------")
        print("speed")
        print(speed_result.obj)
        print(speed_result.BB_pointX)
        print(speed_result.BB_pointY)
        print("--------------------------")
        print("accident")
        print(acc_result.obj)
        print(acc_result.BB_pointX)
        print(acc_result.BB_pointY)
        print("--------------------------")
        print("construction")
        print(con_result.obj)
        print(con_result.BB_pointX)
        print(con_result.BB_pointY)
        #print("traffic: %d"%(traffic_result.obj))
        #print("speed: %d"%(speed_result.obj))
        #print("acc: %d"%(acc_result.obj))
        #print("con: %d"%(con_result.obj))


        rate.sleep()
        cv2.waitKey(1)
bridge = CvBridge()


######################main###########################        

sign = 0
# cap = Image img

width=640
height=480
cap = np.zeros((height, width, 3), np.uint8)
cv_image = np.zeros((height, width, 3), np.uint8)

#cap = cv2.VideoCapture(cv_image)
###cup = cv2.VideoCapture("/home/kroad/catkin_ws/src/signs_sender/scripts/movie(all).mp4")
###cup.set(3, 640)		#width
###cup.set(4, 480)		#height
# cap.set(3, 640)		#width
# cap.set(4, 480)		#height

threshold = 0.5

if __name__=='__main__':
    try:
        talker(sign, threshold) 
    except rospy.ROSInterruptException:
        pass

        



