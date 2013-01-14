import cv2

im = cv2.imread('images/easy.png')
im = cv2.resize(im, (640,580))
cv2.imshow('window', im)
cv2.waitKey(0)
