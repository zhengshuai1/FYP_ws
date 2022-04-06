import numpy as np
import cv2 as cv
from matplotlib import pyplot as plt

filename = './doc/image4.png'
img = cv.imread(filename)
gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
corners = cv.goodFeaturesToTrack(gray, 25, 0.01, 10)
corners = np.int0(corners)
corners = np.squeeze(corners)
# 按照x坐标排序
sort_corners = corners[np.argsort(corners[:, 0])]
# print(sort_corners.shape, sort_corners)
for i in sort_corners:
    x, y = i.ravel()
    cv.circle(img, (x, y), 9, (0, 0, 255), -1)
plt.imshow(img[:, :, ::-1])
plt.savefig('./doc/demo.png')
plt.show()

print(f'第一条边和第四条边之间的距离为{sort_corners[0, 1]- sort_corners[-1, 1]} pixel')
