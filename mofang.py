import queue
import threading
import numpy as np
import cv2
from collections import deque
import RPi.GPIO as GPIO
import time
import wiringpi as wpi

def getDistance():
    global distance
    try:
        address = 0x74 #i2c device address
        h = wpi.wiringPiI2CSetup(address) #open device at address
        wr_cmd = 0xb0  #range 0-5m, return distance(mm)
        #rd_cmd = 0xb2 
        ##range 0-5m, return flight time(us), remember divided by 2
        while True:
            wpi.wiringPiI2CWriteReg8(h, 0x2, wr_cmd)
            wpi.delay(33) #unit:ms  MIN ~ 33
            HighByte = wpi.wiringPiI2CReadReg8(h, 0x2)
            LowByte = wpi.wiringPiI2CReadReg8(h, 0x3)
            Dist = (HighByte << 8) + LowByte
            distance = Dist/10.0
            print('Distance:', distance, 'cm')
    except KeyboardInterrupt:
        pass
        print("END!!!")

def ValidSpeed(speed):
    if speed > 100:
        return 100
    elif speed < 0:
        return 0
    return speed


def turnLeft():
    pwmRight.ChangeDutyCycle(rightSpeed)
    pwmLeft.ChangeDutyCycle(ValidSpeed(leftSpeed + adjust[2]*0.5))
    time.sleep(0.1)
    moveForward()
    return LEFT


def turnRight():
    pwmRight.ChangeDutyCycle(ValidSpeed(rightSpeed - adjust[2]*0.5)*0.9)
    pwmLeft.ChangeDutyCycle(leftSpeed*0.9)
    time.sleep(0.1)
    moveForward()
    return RIGHT


def turnLeftInPlace():
    pwmRight.ChangeDutyCycle(20)
    pwmLeft.ChangeDutyCycle(0)
    time.sleep(0.1)
    stop()
    return LEFT


def turnRightInPlace():
    pwmRight.ChangeDutyCycle(0)
    pwmLeft.ChangeDutyCycle(20)
    time.sleep(0.1)
    stop()
    return RIGHT

def turnLeftNotInPlace():
    pwmRight.ChangeDutyCycle(30)
    pwmLeft.ChangeDutyCycle(20)
    time.sleep(0.1)
    moveForward()
    return LEFT


def turnRightNotInPlace():
    pwmRight.ChangeDutyCycle(20)
    pwmLeft.ChangeDutyCycle(30)
    time.sleep(0.1)
    moveForward()
    return RIGHT

def moveForward():
    pwmRight.ChangeDutyCycle(rightSpeed)
    pwmLeft.ChangeDutyCycle(leftSpeed)

def stop():
    pwmRight.ChangeDutyCycle(0)
    pwmLeft.ChangeDutyCycle(0)


def CanForward():
    return distance > 20


def GetThrough():
    # Turn to a safe direction
    while not CanForward():
        if nowTurnDirection == LEFT:
            turnLeftInPlace()
        elif nowTurnDirection == RIGHT:
            turnRightInPlace()

    # Move forward
    moveForward()
    time.sleep(0.5)


def getCornerXCor(image):
    img = image[:, :-50]
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)  # Convert captured frame from RGB to HSV

    kernel = np.ones((3, 3), np.uint8)

    lower = lowerRange[nowColor]
    upper = upperRange[nowColor]

    mask = cv2.inRange(hsv, lower, upper)

    mask = cv2.erode(mask, kernel, iterations=2)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
    mask = cv2.dilate(mask, kernel, iterations=5)
    res = cv2.bitwise_and(img, img, mask=mask)

    cnts, heir = cv2.findContours(mask.copy(), cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)[-2:]

    if len(cnts) > 0:  # If contours are found
        c = max(cnts, key=cv2.contourArea)  # Find the largest contour by area
        ((x, y), radius) = cv2.minEnclosingCircle(c)  # Find the minimum enclosing circle

        M = cv2.moments(c)
        center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
        if radius > 5:
            cv2.circle(img, (int(x), int(y)), int(radius), (0, 255, 255), 2)
            cv2.circle(img, center, 5, (0, 0, 255), -1)

        # Get the specified corner point
        corner_point = get_corner_from_contour(c, nowTurnDirection)
        cv2.circle(img, (corner_point[0], center[1]), 5, (255, 0, 0), -1)

        # Print the x-coordinate of the corner point
        print(f"X-coordinate of the corner point: {corner_point[0]}")
        xC = corner_point[0]
        cv2.imshow("img", img)
        cv2.imshow("mask", mask)
        cv2.imshow("res", res)
        return xC
    else:
        return None
        # pts.appendleft(center)
        # for i in range(1, len(pts)):
        #     if pts[i - 1] is None or pts[i] is None:
        #         continue
        #     thick = int(np.sqrt(len(pts) / float(i + 1)) * 2.5)
        #     cv2.line(img, pts[i - 1], pts[i], (0, 0, 225), thick)  # Draw line


def findTarget(image):
    while getCornerXCor(image) is None or distance > 100:
        if nowTurnDirection == LEFT:
            turnRightInPlace()
        elif nowTurnDirection == RIGHT:
            turnLeftInPlace()


def turnAroundToFindTarget(image):
    while getCornerXCor(image) is None:
        turnRightInPlace()
        time.sleep(0.5)
        turnLeftInPlace()
        time.sleep(0.5)


def get_corner_from_contour(contour, corner):
    """
    Get the specified corner (left_bottom or right_bottom) from the contour.
    """
    if corner == LEFT:
        # 获取最底部的点
        bottom_point = tuple(contour[contour[:, :, 1].argmax()][0])
        # 获取最底部点的最左侧的点
        point = min([pt[0] for pt in contour if pt[0][1] == bottom_point[1]], key=lambda x: x[0])
    elif corner == RIGHT:
        # 获取最底部的点
        bottom_point = tuple(contour[contour[:, :, 1].argmax()][0])
        # 获取最底部点的最右侧的点
        point = max([pt[0] for pt in contour if pt[0][1] == bottom_point[1]], key=lambda x: x[0])
    else:
        raise ValueError("Corner must be 'left_bottom' or 'right_bottom'")
    
    return point

def getDiff() :
    global diff
    global img
    try:
        while True:
            ret, img = cap.read()
            if not ret:
                print("Failed to capture image")
                break
            
            img = img[:, :-50]
            hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)  # Convert captured frame from RGB to HSV

            kernel = np.ones((3, 3), np.uint8)

            lower = lowerRange[nowColor]
            upper = upperRange[nowColor]

            mask = cv2.inRange(hsv, lower, upper)

            mask = cv2.erode(mask, kernel, iterations=2)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
            mask = cv2.dilate(mask, kernel, iterations=5)
            res = cv2.bitwise_and(img, img, mask=mask)

            cnts, heir = cv2.findContours(mask.copy(), cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)[-2:]
            center = None

            if len(cnts) > 0:  # If contours are found
                c = max(cnts, key=cv2.contourArea)  # Find the largest contour by area
                ((x, y), radius) = cv2.minEnclosingCircle(c)  # Find the minimum enclosing circle

                M = cv2.moments(c)
                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
                if radius > 5:
                    cv2.circle(img, (int(x), int(y)), int(radius), (0, 255, 255), 2)
                    cv2.circle(img, center, 5, (0, 0, 255), -1)

                direction = Color2Direction[nowColor]
                # Get the specified corner point
                corner_point = get_corner_from_contour(c, direction)
                cv2.circle(img, (corner_point[0], center[1]), 5, (255, 0, 0), -1)
                
                # Print the x-coordinate of the corner point
                print(f"X-coordinate of the corner point: {corner_point[0]}")

            # pts.appendleft(center)
            # for i in range(1, len(pts)):
            #     if pts[i - 1] is None or pts[i] is None:
            #         continue
            #     thick = int(np.sqrt(len(pts) / float(i + 1)) * 2.5)
            #     cv2.line(img, pts[i - 1], pts[i], (0, 0, 225), thick)  # Draw line
                xCor = corner_point[0]
                diff = xCor - 320
                print("The difference：", diff)

                

            cv2.imshow("img", img)
            cv2.imshow("mask", mask)
            cv2.imshow("res", res)

            k = cv2.waitKey(30) & 0xFF
            if k == 32:  # Press space to exit
                break
    except KeyboardInterrupt:
        print("END!!!")
        pass
'''
-------------------------------START OF MAIN---------------------------------------
'''
# Video capture from the default camera
cap = cv2.VideoCapture(0)

# Queue to store the points
pts = deque(maxlen=128)


# 初始设置（引脚与pwm波）
EA, I2, I1, EB, I4, I3 = (13, 19, 26, 16, 20, 21)
FREQUENCY = 55
GPIO.setmode(GPIO.BCM)
GPIO.setup([EA, I2, I1, EB, I4, I3], GPIO.OUT)
GPIO.output([EA, I2, EB, I3], GPIO.LOW)
GPIO.output([I1, I4], GPIO.HIGH)
pwmRight = GPIO.PWM(EA, FREQUENCY)
pwmLeft = GPIO.PWM(EB, FREQUENCY)
pwmRight.start(0)
pwmLeft.start(0)

# 增量式PID 定义和初始化三个error和adjust数据
error = [0.0] * 3
adjust = [0.0] * 3

# PID参数、中心值、左右轮占空比和占空比控制数值
kp = 1.8
ki = 0.1
kd = 0.001
imageCenter = 320
leftSpeed = 30
rightSpeed = 30
control = 30



lastAct = 0
LEFT = 1
RIGHT = 2



# Define the HSV color range
lower_red = np.array([0, 80, 50])
upper_red = np.array([8, 255, 220])

lower_green = np.array([35, 43, 46])
upper_green = np.array([77, 255, 255])

lower_blue = np.array([100, 40, 40])
upper_blue = np.array([140, 255, 255])

lowerRange = {"red": lower_red, "green": lower_green, "blue": lower_blue}
upperRange = {"red": upper_red, "green": upper_green, "blue": upper_blue}


Color2Direction = {"red":LEFT, "green":RIGHT, "blue":RIGHT}
Colors = queue.Queue(3)
Colors.put("red")
Colors.put("blue")
Colors.put("green")

print("ready")
input()
nowColor = Colors.get()
nowTurnDirection = Color2Direction[nowColor]
# Video capture from the default camera

diff = 0
distance = 0

thread1 = threading.Thread(target = getDistance)
thread1.start()

thread2 = threading.Thread(target = getDiff)
thread2.start()
time.sleep(3)

try:
    while True:
        # updates the pid
        # 更新PID误差
        error[0] = error[1]
        error[1] = error[2]
        error[2] = diff


        # 更新PID输出（增量式PID表达式）
        adjust[0] = adjust[1]
        adjust[1] = adjust[2]
        adjust[2] = adjust[1] + kp * (error[2] - error[1]) + ki * error[2] + kd * (
                error[2] - 2 * error[1] + error[0])
        print(adjust[2])
        # 饱和输出限制在control绝对值之内
        if adjust[2] > control:
            adjust[2] = control
        elif adjust[2] < -control:
            adjust[2] = -control


        if CanForward():
            # Safe. Keep moving.
            # Turn right
            if adjust[2] > 10:
                lastAct = turnRightNotInPlace()

            # Turn left
            elif adjust[2] < -10:
                lastAct = turnLeftNotInPlace()

            # Go forward
            else:
                moveForward()
        else:
            # # Too close. Get through the obstacle.
            GetThrough()
            stop()
            time.sleep(1)
            print("Get the current target!")
            # Updates the nowColor
            if Colors.empty():
                moveForward()
            else:
                nowColor = Colors.get()
                stop()
                time.sleep(1)
                # Finds the new target
                findTarget(img)
                nowTurnDirection = Color2Direction[nowColor]
            
        # else:
        #     # If not found, then turn around inplace
        #     turnAroundToFindTarget(img)
        
except KeyboardInterrupt:
    stop()
    pass
    print("END!!!")



# Cleanup the camera and close any open windows
stop()
cap.release()
cv2.destroyAllWindows()
pwmRight.stop()
pwmLeft.stop()
GPIO.cleanup()
