import cv2
import RPi.GPIO as GPIO


video = cv2.VideoCapture(0)

GPIO.setmode(GPIO.BOARD)
SERVO = 3
GPIO.setup(SERVO, GPIO.OUT)
servo = GPIO.PWM(SERVO, 700)
servo.start(0)


count = 0
while True:
    ret, frame = video.read()

    servo.ChangeDutyCycle(count)

    count = count + 0.8
    print(count)

    #print(frame)
