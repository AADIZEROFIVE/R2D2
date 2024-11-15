# import cv2
# from picamera2 import Picamera2
# 
# picam2 = Picamera2()
# picam2.start()
# while True:
#     image = picam2.capture_array()
#     cv2.imshow("Frame", image)
#     if(cv2.waitKey(1) == ord("q")):
#         cv2.imwrite("test_frame.png", image)
#         break
# 
# cv2.destroyAllWindows()
#


import cv2
from picamera2 import Picamera2

# Initialize PiCamera2
picam2 = Picamera2()
picam2.start()

# Load Haar Cascade classifier
face_cascade = cv2.CascadeClassifier('haarcascade_frontalface_default.xml')

while True:
    # Capture image from PiCamera2
    image = picam2.capture_array()

    # Convert to grayscale
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # Detect faces
    faces = face_cascade.detectMultiScale(gray, 1.1, 4)

    # Draw rectangles around detected faces
    for (x, y, w, h) in faces:
        cv2.rectangle(image, (x, y), (x+w, y+h), (0, 255, 0), 2)

    # Display the image with detected faces
    cv2.imshow('Frame', image)

    # Exit if 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release resources
cv2.destroyAllWindows()
picam2.stop()