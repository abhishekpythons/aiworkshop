from mss.windows import MSS as mss
import pyautogui as pg
import numpy as np
import time
import cv2
import serial
from PIL import Image

nano_port = "COM4"
nano = serial.Serial(nano_port, 9600)


box = {'top': 654, 'left': 540, 'width': 20, 'height': 21}
            
def image_processing(init_img):
    processed_image = cv2.cvtColor(init_img, cv2.COLOR_BGR2GRAY)
    processed_image = cv2.Canny(processed_image, threshold1 = 200, threshold2 = 200)
    return processed_image

def screen_record():
    sct = mss()
    last_time = time.time()
    time.sleep(5)
    
    while (True):
        img = sct.grab(box)
        print('loop processed for {} seconds'.format(time.time() - last_time))
        last_time = time.time()
        img = np.array(img)
                                                
        processed_image = image_processing(img)
        mean = np.mean(processed_image)
        print('mean = ', mean)

        if not mean == float(0):
            nano.write('105'.encode())
            time.sleep(0.09)
            nano.write('70'.encode())
            time.sleep(0.09)


        if cv2.waitKey(25) & 0xFF == ord('q'):
            cv2.destroyAllWindows()
            break

screen_record()

