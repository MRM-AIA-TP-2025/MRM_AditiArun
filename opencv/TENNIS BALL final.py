import cv2 as cv
import numpy as np

videoCapture = cv.VideoCapture(2)
prevCircle = None

# Square of distance function
dist = lambda x1, y1, x2, y2: np.float64(np.sum(np.square(np.subtract([x1, y1], [x2, y2]))))

while True:
    ret, frame = videoCapture.read()
    if not ret:
        break
    
    grayFrame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    blurFrame = cv.GaussianBlur(grayFrame, (17, 17), 0)

    # Detect circles
    circles = cv.HoughCircles(blurFrame, cv.HOUGH_GRADIENT, 1, 50, param1=25, param2=35, minRadius=10, maxRadius=400)
    
    if circles is not None:
        circles = np.uint16(np.around(circles))
        
        # Convert frame to HSV
        hsvFrame = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
        
        # Define lower and upper bounds for yellow color
        lower_bound = np.array([20, 100, 100])
        upper_bound = np.array([40, 255, 255])

        # Masking
        mask = cv.inRange(hsvFrame, lower_bound, upper_bound)
        mask = cv.erode(mask, None, iterations=2)
        mask = cv.dilate(mask, None, iterations=4)
        
        # Find contours
        contours, _ = cv.findContours(mask.copy(), cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        
        # Check if contours exist
        if contours:
    # Iterate through detected contours
            for contour in contours:
        # Fit a circle to the contour
                (x, y), radius = cv.minEnclosingCircle(contour)
                center = (int(x), int(y))
                radius = int(radius)
        
        # Iterate through detected circles
                for circle in circles[0, :]:
                    circle_center = (circle[0], circle[1])
                    circle_radius = circle[2]
            
            # Check if the detected circle is sufficiently circular and its center is close to the contour center
                    if ((abs(1 - (circle_radius / radius)) < 0.5) or (abs(1 - (radius / circle_radius)) < 0.5)) and (dist(circle_center[0], circle_center[1], center[0], center[1]) < 660):
                # Draw the circle around the detected tennis ball
                        cv.circle(frame, (circle_center[0], circle_center[1]), 1, (0, 100, 100), 3)
                        cv.circle(frame, (circle_center[0], circle_center[1]), circle_radius, (255, 0, 255), 3)
                        prevCircle = circle_center

    
    cv.imshow("circles", frame)
    #cv.imshow("mask", mask)
    key = cv.waitKey(1)
    if key == ord("q"):
        break

videoCapture.release()
cv.destroyAllWindows()
