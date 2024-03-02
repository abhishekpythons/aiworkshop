import RPi.GPIO as GPIO
import numpy as np
import time 
import cv2  # OpenCV    # pip install opencv-python
from PIL import ImageGrab

GPIO.setmode(GPIO.BCM)
GPIO.setup(17, GPIO.OUT)

pwm = GPIO.PWM(17, 50)  # PWM frequency = 50 Hz
pwm.start(7.5)  # Start with 105 degree position (7.5% duty cycle)

box = {'top': 654, 'left': 540, 'width': 20, 'height': 21}

def image_processing(init_img):
    processed_image = cv2.cvtColor(init_img, cv2.COLOR_BGR2GRAY)
    processed_image = cv2.Canny(processed_image, threshold1=200, threshold2=200)
    return processed_image

def screen_record():
    last_time = time.time()
    time.sleep(5)

    while True:
        img = ImageGrab.grab(bbox=(box['left'], box['top'], box['left']+box['width'], box['top']+box['height']))
        print('loop processed for {} seconds'.format(time.time() - last_time))
        last_time = time.time()
        img = np.array(img)

        processed_image = image_processing(img)
        mean = np.mean(processed_image)
        print('mean = ', mean)

        if not mean == float(0):
            pwm.ChangeDutyCycle(5)  # Set duty cycle to 70% (70 degree position)
            time.sleep(0.09)
            pwm.ChangeDutyCycle(7.5)  # Set duty cycle back to 105% (105 degree position)
            time.sleep(0.09)

        if cv2.waitKey(25) & 0xFF == ord('q'):
            cv2.destroyAllWindows()
            break

screen_record()
