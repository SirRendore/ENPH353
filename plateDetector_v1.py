from PIL import Image
import cv2
import matplotlib.pyplot as plt

import numpy as np

from tensorflow import keras
import rospy
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String
bridge = CvBridge()








plates=np.chararray((8,4))
certs=np.zeros((8,4))


def findPlate(image):

  #this could be improved to remove road sides
  filtered=filter(image)
  
  kernel = np.ones((5,5),np.uint8)
  filtered=cv2.erode(filtered,kernel)
  filtered=cv2.dilate(filtered,kernel)
  filtered=cv2.dilate(filtered,kernel)
  cv2.imshow("pop",filtered)
  cv2.waitKey(0)

  _,contours,hierarchy = cv2.findContours(filtered, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)



  
  if(contours):
    a= None
    am=0.0
    ay=0
    b= None
    bm=0.0
    by=0
    for m in contours:
      M=cv2.moments(m)
      if(M["m00"]>am):
        if(am>0.0):
          bm=am
          b=a
          by=ay
        a=m
        am=M["m00"]
        ay=M["m01"]/M["m00"]
      
      elif(M["m00"]>bm):
        b=m
        bm=M["m00"]
        by=M["m01"]/M["m00"]




    coef=0.02
    epsilon = coef*cv2.arcLength(a,True)
    alpha = cv2.approxPolyDP(a,epsilon,True)
    epsilon = coef*cv2.arcLength(b,True)
    beta =cv2.approxPolyDP(b,epsilon,True)
    limit =10
    trys=0
    while(trys<limit):
      trys=trys+1
      epsilon = coef*cv2.arcLength(a,True)
      alpha = cv2.approxPolyDP(a,epsilon,True)
      if(len(alpha)==4):
        break;
      elif(len(alpha)>4):
        coef=coef*1.1
      elif(len(alpha)<4):
        coef=coef/1.1


    if(trys==limit):
      return None

    trys=0
    coef=0.02 
    while(trys<limit):
      epsilon = coef*cv2.arcLength(b,True)
      beta =cv2.approxPolyDP(b,epsilon,True)
      if(len(beta)==4):
        break;
      elif(len(beta)>4):
        coef=coef*1.1
      elif(len(beta)<4):
        coef=coef/1.1

    if(trys==limit):
      return None

    upper=rectifyPoints(alpha)
    lower=rectifyPoints(beta)
    fullpts=np.float32([upper[0],upper[1],lower[2],lower[3]])
    
    platepts=np.float32([upper[2],upper[3],lower[0],lower[1]])
    
   

    
    spotpts=np.float32([upper[0],upper[1],upper[2],upper[3]])

    #print(platepts)
    

    w=1000
    h=1200
    #pts2 = np.float32([[0, 0], [w, 0], [0, h], [w, h]])
    
    #matrix = cv2.getPerspectiveTransform(fullpts, pts2)
    #result = cv2.warpPerspective(image.copy(), matrix, (w, h))
    #plt.imshow(result)
    #plt.figure()

    w=310*5
    h=66*5
    plateSize= np.float32([[0, 0], [w, 0], [0, h], [w, h]])
    matrix = cv2.getPerspectiveTransform(platepts, plateSize)
    plate = cv2.warpPerspective(image.copy(), matrix, (w, h))
    cv2.imshow("bob", plate)
    cv2.waitKey(0)
    

    w=310*3
    h=290*3
    spotSize= np.float32([[0, 0], [w, 0], [0, h], [w, h]])
    matrix = cv2.getPerspectiveTransform(spotpts, spotSize)
    spot = cv2.warpPerspective(image.copy(), matrix, (w, h))
    cv2.imshow("bob", spot)
    cv2.waitKey(0)
    #plt.imshow(spot)
    #plt.figure()
    return plate,spot

def filter(image):
  
  rows,cols,_ = image.shape
  result=np.ones([rows,cols],np.uint8)*255
  goods=0
  sums=0
  for i in range(rows):
    for j in range(cols):
      pixel=image[i,j]
      good =False
      
        
      if(pixel[0]==pixel[1]==pixel[2] and pixel[0]>60):
       
        if(pixel[0]>230):
          
          #kill circle
          for a in range(-7,8):
            for b in range(-7,8):
              if(i+a<720 and i+a>-1 and j+b<1280 and j+b>-1):
                result[i+a,j+b]=0

        else:
          goods=goods+1
          sums=sums+pixel[0]
          
      else:
        result[i,j]=0

  average=sums//goods
  for i in range(rows):
    for j in range(cols):
      pixel=image[i,j]
    
      if((pixel[0]<average)):
        result[i,j]=0

  return result

def rectifyPoints(points):
  averageX=(points[0][0][0]+points[1][0][0]+points[2][0][0]+points[3][0][0])/4.0
  vals=np.float32([[0.0,0],[0,0],[0,0],[0,0]])
  lefts=[]
  rights=[]
  for p in points:
    
    if(p[0][0]>averageX):
      rights.append(p[0])
    else:
      lefts.append(p[0])
  
  if((lefts[0])[1]>(lefts[1])[1]):
    vals[0]=lefts[1]
    vals[2]=lefts[0]
  else:
    vals[0]=lefts[0]
    vals[2]=lefts[1]

  if((rights[0])[1]>(rights[1])[1]):
    vals[1]=rights[1]
    vals[3]=rights[0]
  else:
    vals[3]=rights[1]
    vals[1]=rights[0]

  return vals

#def rectifyPoints(points):
 # averageX=(points[0][0]+points[1][0]+points[2][0]+points[3][0])/4.0
 # averageY=(points[0][1]+points[1][1]+points[2][1]+points[3][1])/4.0
  #vals=np.float32([[0.0,0],[0,0],[0,0],[0,0]])
  #lefts=np.float32
  #for p in points:
   # if(p[0]>averageX and p[1]>averageY):
    #  vals[3]=p
    #elif(p[0]<averageX and p[1]>averageY):
    #  vals[2]=p
    #elif(p[0]<averageX and p[1]<averageY):
    #  vals[0]=p
    #elif(p[0]>averageX and p[1]<averageY):
    #  vals[1]=p
  #return vals

	
def clean(image,background):
  
  _,image= cv2.threshold(image[:,:,0], background-20, 255, cv2.THRESH_BINARY_INV)

  #cv2.imshow("poop",image)
  #cv2.waitKey(0)
  kernel5 = np.ones((5,5),np.uint8)
  
  image = cv2.dilate(image,kernel5)
  
  image= cv2.erode(image,kernel5)

  return image

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

  cv2.imshow('frame',img)
  cv2.waitKey(1)
  plate,spot=findPlate(img)

  
  
  img1=plate[90:250,95:390,:]
  img2=plate[90:250,368:663,:]
  img3=plate[90:250,900:1195,:]
  img4=plate[90:250,1155:1450,:]
 
  
  background=np.mean(plate[280:320,1:80,2])
  #blank corner

  images=[]
  images.append(clean(img1,background))
  images.append(clean(img2,background))


  array=np.array(images)

  array.shape=(2,160,295,1)

  letterModel = keras.models.load_model('/home/fizzer/Documents/plateCNNletters')
  numberModel = keras.models.load_model('/home/fizzer/Documents/plateCNNnumbers')
  results1=letterModel.predict(array)
  
  hot1=results1.argmax(1)
  


  images=[]
  images.append(clean(img3,background))
  images.append(clean(img4,background))
  array=np.array(images)


  
  array.shape=(2,160,295,1)
  

  results2=numberModel.predict(array)
  
  hot2=results2.argmax(1)

  img0=spot[460:800,500:900,:]
  img0=cv2.resize(img0,(295,160))
  img0=clean(img0,100)

  array=np.array(img0)

  array.shape=(1,160,295,1)

  results3=numberModel.predict(array)
  hot3=results3.argmax(1)
  
  #print(results1)
  #print(results2)
  #print(results3)



  spot=hot3[0]
  print(spot)
  cert=np.array([results1[0,hot1[0]],results1[1,hot1[1]],results2[0,hot2[0]],results2[1,hot2[1]]])
  print(cert)
  plate=np.array([number_to_letter(hot1[0]),number_to_letter(hot1[1]),hot2[0],hot2[1]])
  print(plate)
  return spot,plate,cert






if __name__ == '__main__':
 

  img = np.array(Image.open('/home/fizzer/Pictures/candid_plates/E.png'))
  getPlate(img)
  
  