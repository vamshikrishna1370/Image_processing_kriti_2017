#Imported required Libraries
import numpy as np
import cv2

cap=cv2.VideoCapture(1)

while(1):
	ret,frame=cap.read()
	cv2.imshow("Bachi",frame)
	hsv=cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
	#Threshold for Laser
	lower_bound=np.array([0,0,0])
	upper_bound=np.array([0,0,255])

	mask=cv2.inRange(hsv,lower_bound,upper_bound)
	cv2.imshow("Laser",mask)

	#Threshold for Green Point
	lower_bound_0=np.array([0,0,0])
	upper_bound_0=np.array([5,229,255])

	mask_1=cv2.inRange(hsv,lower_bound_0,upper_bound_0)
	cv2.imshow("Green",mask_1)

	#Threshold for Orange Point
	lower_bound_1=np.array([0,0,0])
	upper_bound_1=np.array([179,255,103])

	mask_2=cv2.inRange(hsv,lower_bound_1,upper_bound_1)
	cv2.imshow("Orange",mask_2)

	try:
		#Contour for Laser
		im2,contours,hierarchy = cv2.findContours(mask,2, 2)
		cnt = contours[0]
		M = cv2.moments(cnt)
		cx = int(M['m10']/M['m00'])
		cy = int(M['m01']/M['m00'])
		print  "Red: " +cx,cy

		#Contour for Green
		im2,contours_1,hierarchy = cv2.findContours(mask_1,2, 2)
		cnt_1 = contours_1[0]
		M_1 = cv2.moments(cnt_1)
		cx_1 = int(M_1['m10']/M_1['m00'])
		cy_1 = int(M_1['m01']/M_1['m00'])
		print  "Green : " + cx_1,cy_1

		#Contour for Orange
		im2,contours_2,hierarchy = cv2.findContours(mask_2,2, 2)
		cnt_2 = contours_2[0]
		M_2 = cv2.moments(cnt_2)
		cx_2 = int(M_2['m10']/M_2['m00'])
		cy_2 = int(M_2['m01']/M_2['m00'])
		print "Orange : " + cx_2,cy_2
		print

		rect = cv2.minAreaRect(cnt)
		box = cv2.boxPoints(rect)
		box = np.int0(box)
		cv2.circle(frame,(cx,cy), 4, (0,0,255), -1)
		cv2.imshow("output",frame)

		k = cv2.waitKey(1)&0xFF
		if k==27:
			break

	except:
		pass
cap.release()
cv2.destroyAllWindows()
