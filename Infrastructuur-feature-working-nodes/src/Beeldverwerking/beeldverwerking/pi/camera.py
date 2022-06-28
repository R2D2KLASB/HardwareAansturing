## @package beeldverwerking.pi.camera
# todo. make photo's with a pi and listen to an GPIO pin


# from time import sleep
# import cv2
# import RPi.GPIO as GPIO




# class Camera():

#     def __init__(self, pin):
#         self.pin = pin
#         GPIO.setmode(GPIO.BCM)
#         GPIO.setup(self.pin, GPIO.IN, pull_up_down = GPIO.PUD_DOWN)
#         self.camera = cv2.VideoCapture('/dev/video0', cv2.CAP_V4L)
#         self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
#         self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)


#     def createPic(self, path, file):
#         ret, frame = self.camera.read()
#         cv2.imwrite(path + '/' + file, frame)

#     def checkButton(self):
#         if GPIO.input(self.pin) == GPIO.HIGH:
#             print('Butten pressed')
#             return True
#         return False
