import RPi.GPIO as GPIO
import time
import smbus2
import board
import busio
import picamera
import cv2
import numpy as np

# pins
GPIO.setmode(GPIO.BCM)
GPIO.setup(17, GPIO.OUT)  # Motor 1
GPIO.setup(23, GPIO.OUT)  # Motor 2
GPIO.setup(24, GPIO.OUT)  # Motor 3 (cleaning motor)
GPIO.setup(25, GPIO.IN)   # Ultrasonic sensor trigger
GPIO.setup(26, GPIO.IN)   # Ultrasonic sensor echo
GPIO.setup(27, GPIO.IN)   # Infrared sensor


#cameras
camera1 = picamera.PiCamera()
camera1.rotation = 180
camera2 = picamera.PiCamera()
camera2.rotation = 0
#movement of robot

def forward():
    GPIO.output(17, GPIO.HIGH)
    GPIO.output(23, GPIO.LOW)

def reverse():
    GPIO.output(17, GPIO.LOW)
    GPIO.output(23, GPIO.HIGH)

def activate_cleaning():
    GPIO.output(24, GPIO.HIGH)

def deactivate_cleaning():
    GPIO.output(24, GPIO.LOW)

def Distance():
    GPIO.output(25, GPIO.HIGH)
    time.sleep(0.00001)
    GPIO.output(25, GPIO.LOW)
    start_time = time.time()
    while GPIO.input(26) == 0:
        start_time = time.time()
    stop_time = time.time()
    distance = (stop_time - start_time) * 34300 / 2
    return distance

def infrared_reading():
    return GPIO.input(27)

def camera_images():
    image1 = np.empty((480, 640, 3), dtype=np.uint8)
    image2 = np.empty((480, 640, 3), dtype=np.uint8)
    camera1.capture(image1, format='bgr')
    camera2.capture(image2, format='bgr')
    return image1, image2

def final_images(image1, image2):
    stitcher = cv2.Stitcher_create()
    result = stitcher.stitch([image1, image2])
    return result[1]

def main():
    while True:
        distance = get_distance()
        infrared_reading = get_infrared_reading()
        image1, image2 = get_camera_images()
        final_images = final_images(image1, image2)

        if distance < 20:
            reverse()
        elif infrared_reading == 1:
            forward()
        else:
            deactivate_cleaning()

        if distance < 10:
            activate_cleaning()

        cv2.imshow('Stitched Image', stitched_image)
        cv2.waitKey(1)

if __name__ == '__main__':
    main()