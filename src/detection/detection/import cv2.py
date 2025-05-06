import cv2
import numpy as np

# Load the image
image_path = "image.png"  # Change this to your image path
image = cv2.imread(image_path)
hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

# Function to display HSV values on click
def get_hsv_value(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:  # Left mouse click
        hsv_value = hsv_image[y, x]  # Get HSV value at (x, y)
        print(f"HSV at ({x}, {y}): H={hsv_value[0]}, S={hsv_value[1]}, V={hsv_value[2]}")

# Show image and wait for click
cv2.imshow("Click to get HSV", image)
cv2.setMouseCallback("Click to get HSV", get_hsv_value)

cv2.waitKey(0)
cv2.destroyAllWindows()
