import cv2
import numpy as np
from opcua import  *
from time import sleep

cam1 = cv2.VideoCapture(1)
cam1.set(cv2.CAP_PROP_FRAME_HEIGHT,720)
cam1.set(cv2.CAP_PROP_FRAME_WIDTH,1280)

color1LowerBound=np.array([ 92, 75, 206])#Blue Color 
color1UpperBound=np.array([118,255,255])
color2LowerBound=np.array([71,59,80])#Green Color
color2UpperBound=np.array([88,255,255])

x1=0
y1=0
x0=0
y0=0

def establish_opc_conn():
    url = "opc.tcp://192.168.0.3:3000"
    client_conn = Client(url)

    while True:
        try:
            client_conn.connect()
            print('Connection established')
            break
        except: 
            print('Reconnecting after 5 secs...')
            sleep(5)
    client_conn.get_root_node()  
    return client_conn

def read_input_value(client_fn, node_id):
    client_node = Client.get_node(client_fn, node_id)  # get node
    client_node_value = client_node.get_value()  # read node value
    return client_node_value

def write_value_bool(node_id, value, client_fn):
    client_node = client_fn.get_node(node_id)  # get node
    client_node_value = value
    client_node_dv = ua.DataValue(ua.Variant(client_node_value, ua.VariantType.Boolean))
    client_node.set_value(client_node_dv)

def write_value_int(node_id, value, client_fn):
    client_node = Client.get_node(client_fn, node_id)  # get node
    client_node_value = value
    client_node_dv = ua.DataValue(ua.Variant(client_node_value, ua.VariantType.Int16))
    client_node.set_value(client_node_dv)

def calculateParameters(x0,y0,x1,y1):
    if x1==0 and y1==0 | x1==0 and y1==0:
        angle=720 
        distance=0
    else:
        angle = np.arctan2(y1 - y0, x1- x0)
        angle = np.degrees(angle)
        # ensure angle is positive and in the range [0, 360)
        if angle < 0:
            angle += 360
        distance=np.hypot(x1-x0,y1-y0)
    return (angle,distance)

def getAngle(mask1,mask2,frame):
    x0=0
    y0=0
    x1=0
    y1=0
    contours1,junk=cv2.findContours(mask1,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    for contour in contours1:
        area=cv2.contourArea(contour)
        if area>200:
            x,y,w,h=cv2.boundingRect(contour)
            x0=x+int(w/2)
            y0=y+int(h/2)
            cv2.rectangle(frame,(x,y),(x+w,y+h),(0,0,255),2)          
    contours2,junk=cv2.findContours(mask2,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    for cntr in contours2:
        area2=cv2.contourArea(cntr)
        if area2>200:
            x2,y2,w2,h2=cv2.boundingRect(cntr)
            x1=x2+int(w2/2)
            y1=y2+int(h2/2)
            cv2.rectangle(frame,(x2,y2),(x2+w2,y2+h2),(0,255,0),2)
    cameraParams=calculateParameters(x0,y0,x1,y1)
    return cameraParams
def mouseClick(event,xPos,yPos,flags,params):
    global evt
    global pt1
    global pt2
    if event==cv2.EVENT_LBUTTONDOWN:
        print(event)
        evt=event
        pt1=(xPos,yPos)
    if event==cv2.EVENT_LBUTTONUP:
        print(event)
        evt=event
        pt2=(xPos,yPos)
count=0 
evt=0
roi1Gotten=False
cv2.namedWindow('Frame1')
cv2.setMouseCallback('Frame1',mouseClick) 
while True:
    ignore,frame1=cam1.read()
    frame1=cv2.flip(frame1,0)
    frame1=cv2.flip(frame1,1)
    if count==0:
        print('Select your Region Of Interest For Camera 1')
        print()
        print('Make sure the workpiece is at the start position')
    if evt==4:
        cv2.rectangle(frame1,pt1,pt2,(0,255,0),2)
        ROI1=frame1[pt1[1]:pt2[1],pt1[0]:pt2[0]]
        roi1Gotten=True
    cv2.imshow('Frame1',frame1)
    cv2.moveWindow('Frame1',0,0)
    count=count+1
    if roi1Gotten==True:
        break
    if cv2.waitKey(1)&0xff==ord('q'):
        break
cam1.release()
cv2.destroyAllWindows()

cam1=cv2.VideoCapture(1)
cam1.set(cv2.CAP_PROP_FRAME_HEIGHT,720)
cam1.set(cv2.CAP_PROP_FRAME_WIDTH,1280)
plc_client_1 = establish_opc_conn()
movement=False
prevMovement=False
steps=0
ratioCount=0
prevDistance=0
prevAngle=0 
prevSteps=0
ready=True  
#Main program which gets the angles and distances:
while True:
    write_value_bool('ns=3;s="station3_and_4"."ready"', ready, plc_client_1)
    ignore, frame = cam1.read()
    frame=cv2.flip(frame,0)
    frame=cv2.flip(frame,1)
    frameROI=frame[pt1[1]:pt2[1],pt1[0]:pt2[0]]
    #frameROI = cv2.resize(frameROI, (0,0), fx=2, fy=2)
    frameHSV=cv2.cvtColor(frameROI,cv2.COLOR_BGR2HSV)

    myMask1=cv2.inRange(frameHSV,color1LowerBound,color1UpperBound)
    myMask2=cv2.inRange(frameHSV,color2LowerBound,color2UpperBound)
    camera1Params=getAngle(myMask1,myMask2,frameROI)
    camera1Angle=camera1Params[0]
    camera1Distance=camera1Params[1]
    if ratioCount==0:
        verifiedCamera1Distance=422
        ratio=camera1Distance/verifiedCamera1Distance
    camera1Distance=camera1Distance/ratio

    #print('Camera 1 Angle is: {}' .format(int(camera1Angle)))
    #print('Camera 1 distance is: {}' .format(int(camera1Distance)))
    if camera1Angle>=189 and camera1Angle<=195 and camera1Distance>=418 and camera1Distance<=424:
        #print('Start position')
        steps=1
    if camera1Angle>=215 and camera1Angle<=225 and camera1Distance>=373 and camera1Distance<=385:
        #print('Orientation check')
        steps=2
    if camera1Angle>=241 and camera1Angle<=246 and camera1Distance>=320 and camera1Distance<=331:
        #print('Flipping part 1')
        steps=3
    if camera1Angle>=241 and camera1Angle<=246 and camera1Distance>=337 and camera1Distance<=347:
        #print('Flipping part 2')
        steps=4
    if camera1Angle>=304 and camera1Angle<=312 and camera1Distance>=115 and camera1Distance<=125 :
        #print('Before next station')
        steps=5
    if camera1Angle>=318 and camera1Angle<=325 and camera1Distance>=138 and camera1Distance<=145:
        #print('Transfer to next station')
        steps=6
    if camera1Angle>=333 and camera1Angle<=340 and camera1Distance>=227 and camera1Distance<=240:
        #print('Differentiating position')
        steps=7
    if camera1Angle>=318 and camera1Angle<=325 and camera1Distance>=260 and camera1Distance<=272:
        #print('Metalic Sorter Rod Extended')
        steps=8
    if camera1Angle>=345 and camera1Angle<=350 and camera1Distance>=438 and camera1Distance<=450:
        #print('Release Acrylic')
        steps=9
    if camera1Angle>=348 and camera1Angle<=351 and camera1Distance>=515 and camera1Distance<=530:
        #print('End of acrylic partition')
        steps=10
    if camera1Angle>=334 and camera1Angle<=340.5 and camera1Distance>=430 and camera1Distance<=445:
        #print('Release Metalic')
        steps=11
    if camera1Angle>=338 and camera1Angle<=344 and camera1Distance>=510 and camera1Distance<=535:
        #print('End of metalic partition')
        steps=12
    if camera1Angle==720 or camera1Distance==0:
        print('One or more object(s) not detected')
    deltaDistance=abs(camera1Distance-prevDistance)
    deltaAngle=abs(camera1Angle-prevAngle)
    deltaSteps=steps-prevSteps
    if deltaSteps!=0:
        print('Steps: ',steps)
   
    if prevAngle==720 or prevDistance==0 or prevAngle==0:
        pass
    else:
        if deltaDistance>=20 or deltaAngle>=3:
            movement=True
            #print('Moving.......')
        else:
            movement=False
            #print('Stationary')

    write_value_int('ns=3;s="station3_and_4"."pythonstep"', steps, plc_client_1) #node ID
    write_value_bool('ns=3;s="station3_and_4"."movement"', movement, plc_client_1) #node ID
    ratioCount=ratioCount+1
    prevDistance=camera1Distance
    prevAngle=camera1Angle
    prevSteps=steps
    prevMovement=movement
    cv2.imshow('my ROI',frameROI)
    cv2.moveWindow('my ROI',0,0)
    #print('Movement:    ' ,movement )
    #print(steps)
    sleep(0.0005)
    if cv2.waitKey(1)&0xff==ord('q'):
        break
ready=False
write_value_bool('ns=3;s="station3_and_4"."ready"', ready, plc_client_1)
cam1.release()
cv2.destroyAllWindows()





    




    

