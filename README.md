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
