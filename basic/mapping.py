from djitellopy import tello
import KeyPressModule as kp
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

def getKeyboardInput():
    lr, fb, ud, yv = 0,0,0,0
    speed = 15
    aspeed = 50
    d = 0
    global x, y, yaw, a

    # if kp.getKey("LEFT"): lr = -speed; d,a = dInterval, -180
    # elif kp.getKey("RIGHT"): lr = speed; d,a = -dInterval, 180
    #
    # if kp.getKey("UP"): fb = speed; d,a = dInterval, 270
    # elif kp.getKey("DOWN"): fb = -speed ; d,a = -dInterval, -90

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

    time.sleep(interval)

    a += yaw
    x += int(d * math.cos(math.radians(a)))
    y += int(d * math.sin(math.radians(a)))

    return [lr, fb, ud, yv, x, y]

def drawPoints(img, points):
    for point in points:
        cv2.circle(img, point,8,(0,0,255), cv2.FILLED)
    cv2.circle(img, points[-1], 8, (0,255,0), cv2.FILLED)
    cv2.putText(img, f'({(points[-1][0]-500 )/ 100},{(points[-1][1]-500) / 100})m',
                (points[-1][0] + 10, points[-1][1] + 30), cv2.FONT_HERSHEY_PLAIN, 1,
                (255, 0, 255), 1)

while True:
    vals = getKeyboardInput()
    me.send_rc_control(vals[0], vals[1], vals[2], vals[3])
    print(points)
    img = np.zeros((1000,1000,3),dtype=np.uint8)
    if (points[-1][0] != vals[4] or points[-1][1] != vals[5]):
        points.append((vals[4], vals[5]))

    drawPoints(img, points)
    cv2.imshow("Output",img)
    cv2.waitKey(1)
