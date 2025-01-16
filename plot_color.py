import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm
from matplotlib import colors
import os
import cv2
import PIL

img = cv2.imread('capturedimg.png')

red, green, blue = cv2.split(img)
fig = plt.figure(1)
axis = fig.add_subplot(1, 1, 1, projection="3d")
pixel_colors = img.reshape((np.shape(img)[0]*np.shape(img)[1], 3))
norm = colors.Normalize(vmin=-1.,vmax=1.)
norm.autoscale(pixel_colors)
pixel_colors = norm(pixel_colors).tolist()
axis.scatter(red.flatten(), green.flatten(), blue.flatten(), facecolors=pixel_colors, marker=".")
axis.set_xlabel("Red")
axis.set_ylabel("Green")
axis.set_zlabel("Blue")
# plt.show()

hue, saturation, lightness = cv2.split(img)
fig = plt.figure(2)
axis = fig.add_subplot(1, 1, 1, projection="3d")
axis.scatter(hue.flatten(), saturation.flatten(), lightness.flatten(), facecolors=pixel_colors, marker=".")
axis.set_xlabel("Hue")
axis.set_ylabel("Saturation")
axis.set_zlabel("Lightness")
# plt.show()

plt.figure(3)
plt.imshow(img)
# plt.show()

plt.figure(4)
img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
# cv2.imshow('4', img )
plt.imshow(img)

plt.figure(5)
img = cv2.cvtColor(img, cv2.COLOR_RGB2HLS)
plt.imshow(img)
# cv2.imshow('5', img )


plt.figure(7)
hsl_img = cv2.cvtColor(img, cv2.COLOR_RGB2HLS)
low_threshold = np.array([0, 200, 0], dtype=np.uint8)
high_threshold = np.array([180, 255, 255], dtype=np.uint8)
mask = cv2.inRange(hsl_img, low_threshold, high_threshold)
white_parts = cv2.bitwise_and(img, img, mask = mask)
blur = cv2.GaussianBlur(white_parts, (7,7), 0)
plt.imshow(img)
# cv2.imshow('6', img )


plt.show()