import cv2
import RPi.GPIO as GPIO
import face_recognition as fr
import time


video = cv2.VideoCapture(0)

GPIO.setmode(GPIO.BOARD)
SERVO = 3
GPIO.setup(SERVO, GPIO.OUT)
servo = GPIO.PWM(SERVO, 500)
servo.start(0)


processThisFrame = True
resizeFactor = 3

maxX = 240
desiredX = maxX/2

correctionAngle = 45
servo_off = 75

XPrecValue = desiredX
XITerm = 0

def calcPID(setpoint, value, valueprec, Iterm):

    kP = 0.10
    kI = 0.01
    kD = 0.1

    err = (setpoint - value)
    prec_err = (setpoint - valueprec)

    P = kP * err
    D = kD * (err - prec_err)
    I = kI * err + Iterm

    PID = P + D + I

    print(P, I, D)

    return PID, I


time1 = time.time()
while True:
    ret, frame = video.read()
    
    frame = cv2.resize(frame, (0,0), fx=1/resizeFactor, fy=1/resizeFactor)

    rgb_frame = frame[:,:,::-1]

    if processThisFrame:
        face_locations = fr.face_locations(rgb_frame)

        if not face_locations == []:
            y1, x1, y2, x2 = face_locations[0]
            y = (y1 + y2)/2
            x = (x1 + x2)/2
            
            correctionAngle, XITerm = calcPID(desiredX, x, XPrecValue, XITerm)
            correctionAngle += servo_off

            XPrecValue = x

            if correctionAngle < 50:
                correctionAngle = 0
            if correctionAngle > 100:
                correctionAngle = 100
            
            if time.time() - time1 > 0.02:
                servo.ChangeDutyCycle(correctionAngle)
                time.sleep(0.002)
                servo.ChangeDutyCycle(0)
                time1 = time.time()

            #print(correctionAngle)

    cv2.imshow("followcam", frame)
    cv2.waitKey(1)
