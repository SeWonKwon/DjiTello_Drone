from djitellopy import tello
from basic import KeyPressModule as kp
import numpy as np
import cv2
import time
import math


# https://www.youtube.com/watch?v=LmEcyQnfpDA&t=7261s
#################### PARAMETERS ########################

fSpeed = 117/10 # Forward Speed cm/s (15cm/s)
aSpeed = 360/10 # Angular Speed Degrees/s (50d/s)
interval = 0.25

dInterval = fSpeed*interval
aInterval = aSpeed*interval

##########################################################


kp.init() # KeypressModule init
me = tello.Tello() # Tello init
me.connect() #
print(me.get_battery())
x,y = 500, 500
a = 0
yaw = 0
points=[(0,0),(0,0)]
me.streamon()
flightModeLIST =['Emergency Stop', 'KeyControl','Automatic return',
                 'Face Tracking', 'Plate Number Collecting']
flightMode = flightModeLIST[1]

def getKeyboardInput():
    lr, fb, ud, yv = 0,0,0,0
    speed = 15
    aspeed = 50
    d = 0
    global x, y, yaw, a, flightMode

    if kp.getKey("LEFT"): lr = -speed; d,a = dInterval, -180
    elif kp.getKey("RIGHT"): lr = speed; d,a = -dInterval, 180
    elif kp.getKey("UP"): fb = speed; d,a = dInterval, 270
    elif kp.getKey("DOWN"): fb = -speed ; d,a = -dInterval, -90

    if kp.getKey("w"): ud = speed
    elif kp.getKey("s"): ud = -speed

    if kp.getKey("a"): yv = -aspeed; yaw-= aInterval
    elif kp.getKey("d"): yv = aspeed; yaw += aInterval

    if kp.getKey("q"): me.land(); time.sleep(3)
    if kp.getKey("e"): me.takeoff()

    if kp.getKey("ESCAPE"): flightMode = flightModeLIST[0]
    elif kp.getKey("1"): flightMode = flightModeLIST[1]
    elif kp.getKey("2"): flightMode = flightModeLIST[2]
    elif kp.getKey("3"): flightMode = flightModeLIST[3]
    elif kp.getKey("4"): flightMode = flightModeLIST[4]

    time.sleep(interval)

    a += yaw
    x += int(d * math.cos(math.radians(a)))
    y += int(d * math.sin(math.radians(a)))
    yaw = (yaw+360)%360
    return [lr, fb, ud, yv, x, y]

def getAutoreturn():
    lr, fb, ud, yv = 0,0,0,0
    speed = 15
    aspeed = 50
    d = 0
    interval = 0.25
    dInterval = fSpeed * interval
    aInterval = aSpeed * interval
    global x, y, yaw, a, flightMode, points
    d_00 = math.sqrt(math.pow(((points[-1][0]-500 )/ 100),2)+math.pow((-(points[-1][1]-500) / 100),2))
    d_01 = math.sqrt(math.pow(((points[-2][0]-500 )/ 100),2)+math.pow((-(points[-2][1]-500) / 100),2))
    angle_00 = math.degrees(math.atan(-(points[-1][1] - 500) / (points[-1][0] - 500)))

    if ((points[-1][0] - 500) >= 0) and (-(points[-1][1] - 500) >= 0):
        aim = - (90 + angle_00)
        aim = (aim + 360) % 360

    elif ((points[-1][0] - 500) < 0) and (-(points[-1][1] - 500) >= 0):
        aim =  (90 - angle_00)
        aim = (aim+360) % 360

    if abs(aim - yaw) > 9:
        if yaw > aim:
            yv = -aspeed
            yaw -= aInterval

        elif yaw <= aim:
            yv = aspeed
            yaw += aInterval

    elif d_00 >0.05:
        fb = speed
        d, a = dInterval, 270

    else:
        if int(me.get_height()) > 60: ud = -speed
        elif int(me.get_height()) <= 60: me.land(); print('Home sweet Home'); time.sleep(3)

    if kp.getKey("ESCAPE"): flightMode = flightModeLIST[0]
    elif kp.getKey("1"): flightMode = flightModeLIST[1]
    elif kp.getKey("2"): flightMode = flightModeLIST[2]
    elif kp.getKey("3"): flightMode = flightModeLIST[3]
    elif kp.getKey("4"): flightMode = flightModeLIST[4]

    time.sleep(interval)

    a += yaw
    x += int(d * math.cos(math.radians(a)))
    y += int(d * math.sin(math.radians(a)))
    yaw = (yaw + 360) % 360
    return [lr, fb, ud, yv, x, y, d_00, yaw, angle_00, aim]

def drawPoints(img, points):
    for point in points:
        cv2.circle(img, point,3,(0,0,255), cv2.FILLED)

    pts = np.array([[points[-1][0]+(-10*math.cos(math.radians(yaw))), points[-1][1]+(-10*math.sin(math.radians(yaw)))],
                         [points[-1][0]-(-30*math.sin(math.radians(yaw))), points[-1][1]+(-30*math.cos(math.radians(yaw)))],
                         [points[-1][0]+(10*math.cos(math.radians(yaw))), points[-1][1]+(10*math.sin(math.radians(yaw)))]], dtype=np.int32 )
    cv2.polylines(img , [pts], True , (0,255,0),1)

    cv2.putText(img, f'({(points[-1][0]-500 )/ 100},{-(points[-1][1]-500) / 100})m',
                (points[-1][0] + 10, points[-1][1] + 30), cv2.FONT_HERSHEY_PLAIN, 1,
                (255, 0, 255), 1)

while True:
    if flightMode == flightModeLIST[1]:
        vals = getKeyboardInput()
        me.send_rc_control(vals[0], vals[1], vals[2], vals[3])

    elif flightMode == flightModeLIST[2]:
        vals = getAutoreturn()
        me.send_rc_control(vals[0], vals[1], vals[2], vals[3])

        print('yaw :', vals[7])
        print('d_00 :', vals[6])
        print('angle_00:', vals[8])
        print('aim :', vals[9])

    elif flightMode == flightModeLIST[0]:
        me.send_rc_control(0,0,0,0)
        me.send_rc_control(0,0,-20,0)
        me.land()
        time.sleep(3)

    img = np.zeros((1000, 1000, 3), dtype=np.uint8)

    if (points[-1][0] != vals[4] or points[-1][1] != vals[5]):
        points.append((vals[4], vals[5]))


    cv2.putText(img, flightMode,(20,50), cv2.FONT_HERSHEY_PLAIN, 2,  (255, 111, 255), 1)
    drawPoints(img, points)
    cv2.imshow("Output",img)

    # img_cam = me.get_frame_read().frame
    # img_cam = cv2.resize(img_cam, (360, 240))
    # cv2.imshow('Image', img_cam)

    cv2.waitKey(1)
