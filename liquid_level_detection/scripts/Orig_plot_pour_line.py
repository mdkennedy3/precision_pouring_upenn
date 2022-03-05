import matplotlib.pyplot as plt
import numpy as np
import cv2
I = cv2.imread('test.jpg')
Ib = I[:,:,2]
Iline = Ib[:,420]
plt.plot(Iline)
plt.show()

