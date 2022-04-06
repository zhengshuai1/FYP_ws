import numpy as np
import cv2 as cv
from matplotlib import pyplot as plt

filename = './doc/image4.png'
img = cv.imread(filename)
gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
corners = cv.goodFeaturesToTrack(gray, 25, 0.01, 10)
corners = np.int0(corners)
corners = np.squeeze(corners)
sort_corners = np.sort(corners, axis=0)
print(sort_corners.shape, sort_corners)
for i in corners:
    x, y = i.ravel()
    cv.circle(img, (x, y), 3, (0, 0, 255), -1)
plt.imshow(img[:, :, ::-1])
plt.show()
