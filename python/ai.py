# calibrate.py
import cv2, numpy as np

img = cv2.imread("image.png")
H, W = img.shape[:2]
hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

# Initial guess for the yellow surface
LOWER_YELLOW = np.array([18, 60, 60])
UPPER_YELLOW = np.array([45, 255, 255])

yellow = cv2.inRange(hsv, LOWER_YELLOW, UPPER_YELLOW)
obstacle = cv2.bitwise_not(yellow)   # everything not yellow

# Side-by-side view
cv2.imshow("Original", img)
cv2.imshow("Yellow mask (surface)", yellow)
cv2.imshow("Obstacle mask (inverted)", obstacle)
cv2.waitKey(0)
cv2.destroyAllWindows()
