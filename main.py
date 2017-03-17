# coding=utf-8
import numpy as np
import cv2 as cv
import RPi.GPIO as gpio
import threading as thr

##CLASS GPIO#######################################################################################################################################
class gpioControl:

	#setup of the pin mode as board
	def setMode():
		print("Setting up the GPIO mode as board...")
		try:
		gpio.setmode(gpio.board)
		except RuntimeError:
			print("Unable to set the GPIO module. The GPIO lib hasn't been imported or your board is defective. Send your complains to /dev/null")
		else:
			print("Setting succeful!")
			
#set the mode of the pins that control the base stepper##################
    def SetModeBase():
        try:
            gpio.setup(pin11, gpio.OUT)
            gpio.setup(pin12, gpio.OUT)
            gpio.setup(pin13, gpio.OUT)
            gpio.setup(pin15, gpio.OUT)
        except RuntimeError:
            print("Base pins setup as output has failed.")
        except RuntimeError:
            print("Base pins succefuly seted up as output!")
#############################################################################
	
	#set the mode of the pins that control the camera stepper################
    def SetModeCamera):
		try:
		gpio.setup(pin29, gpio.OUT)
		gpio.setup(pin31, gpio.OUT)
		gpio.setup(pin33, gpio.OUT)
		gpio.setup(pin35, gpio.OUT)
		except RuntimeError:
			print("Camera pins setup as output has failed.")
		else:
			print("Camera pins succefuly setep up as output!")
			
	#########################################################################
	#self explanatory########################################################
	def step1(pin1, pin2, pin3, pin4):
		GPIO.output(pin1, LOW)
		GPIO.output(pin2, HIGH)
		GPIO.output(pin3, HIGH
		GPIO.output(pin4, LOW)
	
	def step2(pin1, pin2, pin3, pin4):
		GPIO.output(pin1, LOW)
		GPIO.output(pin2, HIGH)
		GPIO.output(pin3, LOW)
		GPIO.output(pin4, HIGH)

	def step3(pin1, pin2, pin3, pin4):
		GPIO.output(pin1, HIGH)
		GPIO.output(pin2, LOW)
		GPIO.output(pin3, LOW)
		GPIO.output(pin4, HIGH)
	def step4(pin1, pin2, pin3, pin4):
		GPIO.output(pin1, HIGH)
		GPIO.output(pin2, LOW)
		GPIO.output(pin3, HIGH)
		GPIO.output(pin4, LOW)
	
	def sequenceLeft(pin1, pin2, pin3, pin4):
		step1(pin1, pin2, pin3, pin4)
		step2(pin1, pin2, pin3, pin4)
		step3(pin1, pin2, pin3, pin4)
		step4(pin1, pin2, pin3, pin4)

    def sequenceRight(pin1, pin2, pin3, pin4):
		step4(pin1, pin2, pin3, pin4)
		step3(pin1, pin2, pin3, pin4)
		step2(pin1, pin2, pin3, pin4)
		step1(pin1, pin2, pin3, pin4)

    def fullReset(pin1, pin2, pin3, pin4, pin5, pin6, pin7, pin8):
		GPIO.output(pin1, LOW)
		GPIO.output(pin2, LOW)
		GPIO.output(pin3, LOW)
		GPIO.output(pin4, LOW)
		GPIO.output(pin5, LOW)
		GPIO.output(pin6, LOW)
		GPIO.output(pin7, LOW)
		GPIO.output(pin8, LOW)
##GPIO CLASS END################################################################################################

##IMAGE PROCESSING CLASS##########################################################################################
class processingImage:
    def __init__(self):
        self.lower = np.array([0,0,0])
        self.upper = np.array([255,255,255])
        self.cap = cv.VideoCapture(0)
        self.flag, self.img = self.cap.read()
        imgHeight, imgWidth, channels = self.img.shape
        self.posX = imgWidth / 2
        self.posY = imgHeight / 2
        #I needed to put x and y center upper in the hierarchy to use them in the main
        self.xcenter = self.posX
        self.ycenter = self.posY
    def setHSV(self, x, y):
        colorPixel = self.imgHSV[y, x]

        if (colorPixel[0] - 3) <= 0:
            H_Min = 0
        else:
            H_Min = colorPixel[0] - 3

        if (colorPixel[0] + 3) >= 255:
            H_Max = 255
        else:
            H_Max = colorPixel[0] + 3

        if (colorPixel[1] - 10) <= 0:
            S_Min = 0
        else:
            S_Min = colorPixel[1] - 10

        if (colorPixel[1] + 10) >= 255:
            S_Max = 255
        else:
            S_Max = colorPixel[1] + 10

        if (colorPixel[2] - 60) <= 0:
            V_Min = 0
        else:
            V_Min = colorPixel[2] - 60

        if (colorPixel[2] + 60) >= 255:
            V_Max = 255
        else:
            V_Max = colorPixel[2] + 60

        self.lower = np.array([H_Min,S_Min,V_Min])
        self.upper = np.array([H_Max,S_Max,V_Max])

    def setPosxPosy(self, x, y):
        self.posX = x
        self.posY = y

    def setImg(self):
        self.flag, self.img = self.cap.read()
        self.img = cv.flip(self.img, 1)
        kernel = np.ones((3,3),np.float32)/25
        self.img = cv.filter2D(self.img,-1,kernel)
        self.imgHSV = cv.cvtColor(self.img, cv.COLOR_RGB2HSV)
        self.imgThreshold = cv.inRange(self.imgHSV, self.lower, self.upper)

    def morph(self):
        kernel = np.ones((10,10), np.uint8)
        cv.morphologyEx(self.imgThreshold, cv.MORPH_OPEN, kernel)
        cv.morphologyEx(self.imgThreshold, cv.MORPH_CLOSE, kernel)

    def trackObject(self):
        flag2, contours, hierarchy = cv.findContours(self.imgThreshold,cv.RETR_LIST,cv.CHAIN_APPROX_SIMPLE)
        i = 0
        i2 = -1
        for cnt in contours:
            x,y,w,h = cv.boundingRect(cnt)
            xcenter = (x + w) / 2
            ycenter = (y + h) / 2

            if i == 0:
                dist = ((xcenter-self.posX)**2+(ycenter-self.posY)**2)**.5
                i2 = 0
            elif dist > ((xcenter-self.posX)**2+(ycenter-self.posY)**2)**.5:
                dist = ((xcenter-self.posX)**2+(ycenter-self.posY)**2)**.5
                i2 = i
            i = i + 1

        if i2 != -1:
            x,y,w,h = cv.boundingRect(contours[i2])
            cv.rectangle(self.img, (x,y), (x+w, y+h), (0, 255, 0), 2)

            self.setPosxPosy(x + w / 2, y + h / 2)
            colorPixel = self.imgHSV[y + h / 2, x + w / 2]

    def drawObject(self):
        cv.circle(self.img, (self.posX, self.posY), 10, (0,255,0), -1)
##IMAGE PROCESSING CLASS END################################################################################################

#MAIN FUNCTION#############################################################################################################
procImg = processingImage()
#the pins that that control the coils of the base stepper
pin11 = 11
pin12 = 12
pin13 = 13
pin15 = 15

#the pins that control the coils of the camera stepper
pin29 = 29
pin31 = 31
pin33 = 33
pin35 = 35

def clickMouse(event, x, y, flags,param):
    if event == cv.EVENT_LBUTTONDBLCLK:
        procImg.setHSV(x, y)
        procImg.setPosxPosy(x, y)

cv.namedWindow('Image')
cv.namedWindow('Image HSV')
cv.namedWindow('Image Threshold')

cv.setMouseCallback('Image',clickMouse)


##this gives the signal sequences for the base stepper
def trackXpos(pin11, pin12, pin13, pin15):
    if (procImg.xcenter < procImg.posX):
        gpio.sequenceLeft(pin11, pin12, pin13, pin15)
    elif (procImg.xcenter > procImg.posX):
        gpio.sequenceRight(pin11, pin12, pin13, pin15)
    else:
        print("Target X located")
##same shit as above but for the camera stepper
def trackYpos(pin29, pin31, pin33, pin35):
    if procImg.ycenter < procImg.posY:
        gpio.sequenceLeft(pin29, pin31, pin33, pin35)
    elif procImg.ycenter > procImg.posY:
        gpio.sequenceRight(pin11, pin12, pin13, pin15)
    else:
        print("Target Y located")

#Threading:
#Thread1 tracks the object. trackObject takes no arguments
Thread1 = thr.Thread(target = procImg.trackObject, args=())
#Thread2: this thread controls the x stepper
Thread2 = thr.Thread(target=trackXpos, args=(pin11,pin12,pin13,pin15))
#Thread3: this thread controls the y stepper
Thread3 = thr.Thread(target=trackXpos, args=(pin29,pin31,pin33,pin35))
#the forth thread will do the rest

while (cv.waitKey(10) != 27):
    procImg.setImg()
    procImg.morph()
    Thread1.start()
    procImg.drawObject()
    frame = procImg.img
    frameHSV = procImg.imgHSV
    frameThr = procImg.imgThreshold
    Thread2.start()
    Thread3.start()
    cv.imshow("Image", frame)
    cv.imshow("Image HSV", frameHSV)
    cv.imshow("Image Threshold", frameThr)
##MAIN FUNCTION END################################################################################################