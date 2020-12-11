from PIL import Image
import cv2
import matplotlib.pyplot as plt

import numpy as np
import tensorflow as tf
from tensorflow import keras
import rospy
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String
bridge = CvBridge()
from sensor_msgs.msg import Image
import time

from rosgraph_msgs.msg import Clock
rospy.init_node('plate_detector')


#tf_config = some_custom_config
sess = tf.Session()
graph = tf.get_default_graph()

# IMPORTANT: models have to be loaded AFTER SETTING THE SESSION for keras! 
# Otherwise, their weights will be unavailable in the threads after the session there has been set
keras.backend.set_session(sess)

spotModel= letterModel = keras.models.load_model('/home/fizzer/Documents/plateCNNspots')
letterModel = keras.models.load_model('/home/fizzer/Documents/plateCNNletters')
numberModel = keras.models.load_model('/home/fizzer/Documents/plateCNNnumbers')



plates=np.chararray((8,4))
certs=np.zeros((8,4))
stopped=False
inside=False


def findPlate(image):
  
  if(inside):
    img=image[250:,900:]
  else:
    img=image[250:,:1280-600]
  cv2.imshow("text",img)
  cv2.waitKey(1)

  hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
  uh = 136
  us = 255
  uv = 255
  lh = 100
  ls = 120
  lv = 0
  lower_hsv = np.array([lh,ls,lv])
  upper_hsv = np.array([uh,us,uv])
  mask = cv2.inRange(hsv, lower_hsv, upper_hsv)


  kernel = np.ones((5,5),np.uint8)
  mask=cv2.erode(mask,kernel)
  
  mask=cv2.dilate(mask,kernel)

  
  

  _,contours,hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

  empty=np.array(0)
  vals=np.array([[1000,empty],[1000,empty],[1000,empty],[1000,empty],[1000,empty],[1000,empty],[1000,empty],[1000,empty],[1000,empty],[1000,empty],[1000,empty]])
  cont=[]
  if(contours):
    a= None
    am=0.0
    ay=0
    ax=0
    index=-1
    for m in contours:
      cont.append(m)
      index=index+1
      M=cv2.moments(m)
      x=M["m10"]/M["m00"]
      vals[index]=[x,index]

      if(M["m00"]>am):
        a=m
        am=M["m00"]
        ax=vals[index,0]
      
    

    
    #print(vals)
    vals=vals[np.argsort(vals[:,0])]
    #print(vals)

    left=None
    right=a
    if(time<170):
      for i in range(len(contours)):
        if(vals[i,0]==ax):
          left=cont[vals[i-1,1]]
          break
    else:
      for i in range(len(contours)):
        if(vals[i,0]==ax):
          left=right
          right=cont[vals[i+1,1]]
          break
    


    coef=0.01
    epsilon = coef*cv2.arcLength(left,True)
    left = cv2.approxPolyDP(left,epsilon,True)[:,0]
    epsilon = coef*cv2.arcLength(right,True)
    right =cv2.approxPolyDP(right,epsilon,True)[:,0]
    #print(left)
    #print(right)

    zero= (np.max(left[:,0]),np.min(left[:,1]))
    one = (np.min(right[:,0]),np.min(right[:,1]))
    two = (np.max(left[:,0]),np.max(left[:,1]))
    three = (np.min(right[:,0]),np.max(right[:,1]))
    fullpts=np.float32([zero,one,two,three])

    w=1000
    h=1200
    pts2 = np.float32([[0, 0], [w, 0], [0, h], [w, h]])
    
    matrix = cv2.getPerspectiveTransform(fullpts, pts2)
    entire = cv2.warpPerspective(img.copy(), matrix, (w, h))
    #cv2.imshow("bruh",img)
    #cv2.waitKey(0)
    #cv2.imshow("bruh",entire)
    #cv2.waitKey(0)






    #spot #
    top =entire[300:900,500:975,2]

    _,top=cv2.threshold(top,60,255,cv2.THRESH_BINARY_INV)
    
    _,contours,hierarchy = cv2.findContours(top, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)


    images=[]
    if(contours):

      #I am stupid, fuck off
      A=None
      
      
      a=0
      
     
      
      
      for x in contours:
        M=cv2.moments(x)
        area=M["m00"]

        if(area>a):
          a=area
          A=x




      if(a>60000):
        print("spot too big")
        return
      if(a<5000):
        print("spot too small")
        return
     
     
      left= np.min(A[:,0,0])
      right =np.max(A[:,0,0])
      toop=np.min(A[:,0,1])
      bot=np.max(A[:,0,1])
      
      
      height=350
      width=450
      
      h=(height-(bot-toop))//2
      y1=bot+h
      y0=y1-height
      
      x0=12
      x1=462

      
      images.append(top[y0:y1,x0:x1])  
      


    entire=entire[500:1150,25:975]
   


    uh = 144
    us = 255
    uv = 255
    lh = 106
    ls = 105
    lv = 70
    lower_hsv = np.array([lh,ls,lv])
    upper_hsv = np.array([uh,us,uv])
    hsv = cv2.cvtColor(entire, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower_hsv, upper_hsv)
    mask=cv2.erode(mask,kernel)
    
    
    _,contours,hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    if(contours):

      #I am stupid, fuck off
      A=None
      B=None
      C=None
      D=None
      
      a=0
      b=0
      c=0
      d=0
      ax=0
      bx=0
      cx=0
      dx=0
      for x in contours:
        M=cv2.moments(x)
        area=M["m00"]

        if(area>a):
          d=c
          c=b
          b=a
          a=area
          D=C
          C=B
          B=A
          A=x
          dx=cx
          cx=bx
          bx=ax
          ax=M["m10"]/M["m00"]

        elif(area>b):
          d=c 
          c=b 
          b=area
          D=C
          C=B
          B=x
          dx=cx
          cx=bx
          bx=M["m10"]/M["m00"]
         
        elif(area>c):
          d=c
          c=area
          D=C
          C=x
          dx=cx
          cx=M["m10"]/M["m00"]
          
        elif(area>d):
          d=area
          D=x
          dx=M["m10"]/M["m00"]



    vals=np.array([[ax,A],[bx,B],[cx,C],[dx,D]])
      
    vals=vals[np.argsort(vals[:,0])]


    print(a)
    print(b)
    print(c)
    print(d)

    print(vals[0,0])

    print(vals[1,0])
    print(vals[2,0])
    print(vals[3,0])


    #we can do some checks here

    if((a>15000) or (a<1500)):
      print("size issue")
      return 
    if((b>15000) or (b<1500)):
      print("size issue")
      return 
    if((c>15000) or (c<1500)):
      print("size issue")
      return 
    if((d>15000) or (d<1500)):
      print("size issue")
      return 

    #if((b>1000) or (b<80)):
     # print("too small")
      #return 
    #if((c>1000) or (c<80)):
     # print("too small")
     # return 
    #if((d>1000) or (d<80)):
     # print("too small")
      #return 

    if(vals[0,0]>300):
      print("bad first")
      return

    
    if(vals[1,0]-vals[0,0]<20):
      print("bad spacing1")
      return

    if(vals[2,0]-vals[1,0]<20):
      print("bad spacing2")
      return


    if(vals[3,0]-vals[2,0]<20):
      print("bad spacing3")
      return

    #cv2.imshow("bruh",top)
    #cv2.waitKey(0)
   
    uh = 144
    us = 255
    uv = 255
    lh = 106
    ls = 75
    lv = 70
    lower_hsv = np.array([lh,ls,lv])
    upper_hsv = np.array([uh,us,uv])
    hsv = cv2.cvtColor(entire, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower_hsv, upper_hsv)
    mask=cv2.erode(mask,kernel)

    height=130
    width=245
    for i in range(4):
      v=vals[i,1]
      left= np.min(v[:,0,0])

      right =np.max(v[:,0,0])
      top=np.min(v[:,0,1])
      bot=np.max(v[:,0,1])
      h=(height-(bot-top))//2
      y1=bot+h
      y0=y1-height
      w=(width-(right-left))//2
      x1=right+w
      x0=x1-width
      images.append(mask[y0:y1,x0:x1])

    return images



def number_to_letter(x):
  switcher = {

    0: 'A',
    1: 'B',
    2:'C',
    3:  'D',
    4:  'E',
    5:  'F',
    6:  'G',
    7:  'H',
    8:  'I',
    9:  'J',
    10:  'K',
    11:  'L',
    12:  'M',
    13:  'N',
    14:  'O',
    15:  'P',
    16:  'Q',
    17:  'R',
    18:  'S',
    19:  'T',
    20:  'U',
    21:  'V',
    22:  'W',
    23:  'X',
    24:  'Y',
    25:  'Z',
      
  }
  return switcher.get(x,26)


def getPlate(img):

  images=findPlate(img)
  
 
  img1=images[1]
 
  img2=images[2]
  img3=images[3]

  img4=images[4]

  img0=images[0]
  

  global sess
  global graph
  with graph.as_default():

    keras.backend.set_session(sess)



    images1=[]
    images1.append(img1)
    images1.append(img2)
    array1=np.array(images1)
    array1.shape=(2,130,245,1)
    results1=letterModel.predict(array1)


    images2=[]
    images2.append(img3)
    images2.append(img4)
    array2=np.array(images2)
    array2.shape=(2,130,245,1)
    results2=numberModel.predict(array2)


    bob=np.zeros((350,450))
    bob[:img0.shape[0],:img0.shape[1]]=img0
    
    array3=np.array(bob)
    array3.shape=(1,350,450,1)
    results3=spotModel.predict(array3)

  hot1=results1.argmax(1)
  hot2=results2.argmax(1)
  hot3=results3.argmax(1)
  
  
  #print(results1)
  #print(results2)
  #print(results3)


  if(results3.max<0.95):
    print("bad spot reading")
    return
    
  spot=hot3[0]+1
  print(spot)
  cert=np.array([results1[0,hot1[0]],results1[1,hot1[1]],results2[0,hot2[0]],results2[1,hot2[1]]])
  print(cert)
  plate=np.array([number_to_letter(hot1[0]),number_to_letter(hot1[1]),hot2[0],hot2[1]])
  print(plate)
  return spot,plate,cert

frame=-1

def img_callback(image_msg):
  global stopped
  if((time>240)and not stopped):
    switch.publish("stop")
    stopped=True

  if((time>120) and (not(inside))):
    inside=True
    switch.publish("turn")

  global frame
  frame=frame+1
  if(frame%1!=0):
    return

  try:
    cv_image = bridge.imgmsg_to_cv2(image_msg, "bgr8")
  except CvBridgeError as e:
    print(e)    
    return


  try:
    
    spot,plate,cert=getPlate(cv_image)

    #images=getPlate(cv_image)

    #if(images[4]):
      #return
    #t=images[4][0][0]
    #folder='/home/fizzer/Documents/runs/'

 #   cv2.imwrite(folder+'L/'+str(frame)+str(time.time())+'_1.jpg', images[1])
#    cv2.imwrite(folder+'L/'+str(frame)+str(time.time())+'_2.jpg', images[2])
    #cv2.imwrite(folder+'N/'+str(frame)+str(time.time())+'_1.jpg', images[3])
  #  cv2.imwrite(folder+'N/'+str(frame)+str(time.time())+'_2.jpg', images[4])
   # cv2.imwrite(folder+'P/'+str(frame)+str(time.time())+'.jpg', images[0])
    #print("saved")

    




    if(spot):
      spot=spot-1
      changed=False
      global plates
      global certs
      for i in range(4):
        if(cert[i]>certs[spot,i] and (plate[i] != plates[spot,i])):
          changed=True
          plates[spot,i]=plate[i]
          certs[spot,i]=cert[i]

      if(changed):
        print("published plate")
        message=str('Dante_Leo,password,'+str(spot+1)+','+plates[spot,0]+plates[spot,1]+plates[spot,2]+plates[spot,3])
        license_pub.publish(message)


    global inside
    if((np.min(certs[0:6,:])>0.9) and not inside):
      inside=True
      switch.publish("turn")

    global stopped
    if(np.min(certs)>0.9 and not stopped):
      print(certs)
      stopped=True
      switch.publish("stop")

  except CvBridgeError as e:
    print("uh oh")
    return


def timer(clocky):
  global start
  if(start==-1):
    start=clocky.clock.secs
  global time
  #print(clocky)
  time = clocky.clock.secs-start
  


start=-1
time=0


license_pub = rospy.Publisher("/license_plate", String, queue_size=500000, latch=False)

sub_camera = rospy.Subscriber("/R1/pi_camera/image_raw", Image, img_callback)

clock=rospy.Subscriber("/clock",Clock,timer)
switch=rospy.Publisher("/plate_reader_command",String, queue_size=50, latch=False)

rospy.spin()

while license_pub.get_num_connections() == 0:
    print("Waiting for subscribers")
    rospy.sleep(0.5)
#if __name__ == '__main__':
 

  #img = np.array(Image.open('/home/fizzer/Pictures/newRun/D.png'))
 # getPlate(img)
  
  