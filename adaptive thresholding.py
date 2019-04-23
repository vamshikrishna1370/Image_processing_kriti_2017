import cv2
import numpy as np
cap = cv2.VideoCapture(0)
while 1:
	ret,frame = cap.read()
	a =  cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
	a1 = cv2.GaussianBlur(a,(5,5),0)
	#ret,thr =cv2.threshold(a,0,255,cv2.THRESH_OTSU)
	#cv2.imshow('otsu_thres without gaussian blur',thr)
	#ret1,thr1 =cv2.threshold(a1,0,255,cv2.THRESH_OTSU)
	#cv2.imshow('otsu_thres with gaussian blur',thr1)
	cv2.imshow('orginal', frame)	
	th2 = cv2.adaptiveThreshold(a,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,cv2.THRESH_BINARY,11,2)
	cv2.imshow('adaptivethresholding with gaussian blur',th2)
	blur = cv2.GaussianBlur(th2,(5,5),0)	
	#blur = cv2.bilateralFilter(th2,9,75,75)
	ret,blur = cv2.threshold(blur,127,255,cv2.THRESH_BINARY)	
	cv2.imshow('after lateral blur',blur)	
	if cv2.waitKey(1) & 0xFF == ord('q'):
        	break

cap.release()  
cv2.destroyAllWindows()


