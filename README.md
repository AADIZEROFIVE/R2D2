# R2D2
https://www.raspberrypi.com/documentation/computers/remote-access.html

https://youtu.be/w6OsICbnJbA?si=lr2aQkIp4HFyw6tm

https://github.com/opencv/opencv/issues/25072

https://libcamera.org/open-projects.html

https://github.com/opencv/opencv/issues/21653

## Generate and save ssh key
$ ssh-keygen
> use the address in the paranthesis on terminal<br>
> then go to that location using "cd"<br>
>
$ cat id_rsa.pub<br>
> copy and upload that key in pi_ssh file
>


import cv2
from picamzero import Camera
from time import sleep

# Initialize the camera using picamzero
cam = Camera()
cam.start_preview()

# Load the pre-trained Haar Cascade classifier for face detection
face_cascade = cv2.CascadeClassifier('haarcascade_frontalface_default.xml')

# Wait for the camera to warm up
sleep(2)

# Start capturing frames from the camera
while True:
    # Capture a frame from the picamzero camera
    frame = cam.capture_frame(0)  # This method grabs a frame from the PiCamera

    # Convert the frame to grayscale (Haar cascades work better with grayscale images)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Detect faces in the grayscale image
    faces = face_cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30))

    # Draw rectangles around the detected faces
    for (x, y, w, h) in faces:
        cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 2)

    # Display the resulting frame with face detection
    cv2.imshow('Face Detection', frame)

    # Break the loop if the 'q' key is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release resources and close OpenCV windows
cam.stop_preview()
cv2.destroyAllWindows()


import cv2
from picamera2 import Picamera2
from time import sleep

# Initialize the camera using Picamera2
camera = Picamera2()
camera.configure(camera.create_preview_configuration())
camera.start()

# Load the pre-trained Haar Cascade classifier for face detection
face_cascade = cv2.CascadeClassifier('haarcascade_frontalface_default.xml')

# Wait for the camera to warm up
sleep(2)

while True:
    # Capture a frame from the camera
    frame = camera.capture_image()

    # Convert the frame to grayscale (Haar cascades work better with grayscale images)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Detect faces in the grayscale image
    faces = face_cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5,   
 minSize=(30, 30))

    # Draw rectangles around the detected faces
    for (x, y, w, h) in faces:
        cv2.rectangle(frame,   
 (x, y), (x+w, y+h), (255, 0, 0), 2)

    # Display the resulting   
 frame with face detection
    cv2.imshow('Face Detection', frame)

    # Break the loop if the 'q' key is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release   
 resources and close OpenCV windows
camera.stop()
cv2.destroyAllWindows()


ERROR

[0:30:41.770174494] [3557]  INFO Camera camera_manager.cpp:325 libcamera v0.3.2+27-7330f29b
[0:30:41.777849015] [3560]  INFO RPI pisp.cpp:695 libpisp version v1.0.7 28196ed6edcf 29-08-2024 (16:33:32)
[0:30:41.788418984] [3560]  INFO RPI pisp.cpp:1154 Registered camera /base/axi/pcie@120000/rp1/i2c@88000/ov5647@36 to CFE device /dev/media0 and ISP device /dev/media2 using PiSP variant BCM2712_C0
[0:30:41.791243909] [3557]  INFO Camera camera.cpp:1197 configuring streams: (0) 640x480-XBGR8888 (1) 640x480-GBRG_PISP_COMP1
[0:30:41.791351120] [3560]  INFO RPI pisp.cpp:1450 Sensor: /base/axi/pcie@120000/rp1/i2c@88000/ov5647@36 - Selected sensor format: 640x480-SGBRG10_1X10 - Selected CFE format: 640x480-PC1g
Traceback (most recent call last):
  File "/home/aadish/facedetection/src/cam_try.py", line 21, in <module>
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
           ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
cv2.error: OpenCV(4.6.0) :-1: error: (-5:Bad argument) in function 'cvtColor'
> Overload resolution failed:
>  - src is not a numpy array, neither a scalar
>  - Expected Ptr<cv::UMat> for argument 'src'


sol1

import cv2
from PIL import Image

# Assuming you have a PIL Image object named 'pil_image'
pil_image = Image.open('your_image.jpg')

# Convert the PIL Image to a NumPy array
open_cv_image = np.array(pil_image)

# Convert RGB to BGR (OpenCV's color channel order)
open_cv_image = open_cv_image[:, :, ::-1]

# Now you can use open_cv_image with OpenCV functions
gray = cv2.cvtColor(open_cv_image, cv2.COLOR_BGR2GRAY)
