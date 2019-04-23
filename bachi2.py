#Imported required Libraries
import numpy as np
import cv2

class PID:
	"""
	Discrete PID control
	"""

	def __init__(self, P=2.0, I=0.0, D=1.0, Derivator=0, Integrator=0, Integrator_max=500, Integrator_min=-500):

		self.Kp=P
		self.Ki=I
		self.Kd=D
		self.Derivator=Derivator
		self.Integrator=Integrator
		self.Integrator_max=Integrator_max
		self.Integrator_min=Integrator_min

		self.set_point=0.0
		self.error=0.0

	def update(self,current_value):
		"""
		Calculate PID output value for given reference input and feedback
		"""

		self.error = self.set_point - current_value

		self.P_value = self.Kp * self.error
		self.D_value = self.Kd * ( self.error - self.Derivator)
		self.Derivator = self.error

		self.Integrator = self.Integrator + self.error

		if self.Integrator > self.Integrator_max:
			self.Integrator = self.Integrator_max
		elif self.Integrator < self.Integrator_min:
			self.Integrator = self.Integrator_min

		self.I_value = self.Integrator * self.Ki

		PID = self.P_value + self.I_value + self.D_value

		return PID

	def setPoint(self,set_point):
		"""
		Initilize the setpoint of PID
		"""
		self.set_point = set_point
		self.Integrator=0
		self.Derivator=0

	def setIntegrator(self, Integrator):
		self.Integrator = Integrator

	def setDerivator(self, Derivator):
		self.Derivator = Derivator

	def setKp(self,P):
		self.Kp=P

	def setKi(self,I):
		self.Ki=I

	def setKd(self,D):
		self.Kd=D

	def getPoint(self):
		return self.set_point

	def getError(self):
		return self.error

	def getIntegrator(self):
		return self.Integrator

	def getDerivator(self):
		return self.Derivator

cap=cv2.VideoCapture(1)

while(1):
	ret,frame=cap.read()
	cv2.imshow("Bachi",frame)
	hsv=cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
	#Threshold for Laser
	lower_bound=np.array([0,0,221])
	upper_bound=np.array([0,0,255])

	mask=cv2.inRange(hsv,lower_bound,upper_bound)
	cv2.imshow("Laser",mask)

	#Threshold for Green Point
	lower_bound_0=np.array([0,80,0])
	upper_bound_0=np.array([5,229,255])

	mask_1=cv2.inRange(hsv,lower_bound_0,upper_bound_0)
	cv2.imshow("Green",mask_1)

	#Threshold for Orange Point
	lower_bound_1=np.array([60,0,0])
	upper_bound_1=np.array([179,255,103])

	mask_2=cv2.inRange(hsv,lower_bound_1,upper_bound_1)
	cv2.imshow("Orange",mask_2)

	try:
		#Contour for Laser
		contours= cv2.findContours(mask,2, 2)
		cnt = contours[0]
		M = cv2.moments(cnt)
		cx = int(M['m10']/M['m00'])
		cy = int(M['m01']/M['m00'])
		print cx,cy

		#Contour for Green
		contours_1 = cv2.findContours(mask_1,2, 2)
		cnt_1 = contours_1[0]
		M_1 = cv2.moments(cnt_1)
		cx_1 = int(M_1['m10']/M_1['m00'])
		cy_1 = int(M_1['m01']/M_1['m00'])
		print cx_1,cy_1

		#Contour for Orange
		contours_2 = cv2.findContours(mask_2,2, 2)
		cnt_2 = contours_2[0]
		M_2 = cv2.moments(cnt_2)
		cx_2 = int(M_2['m10']/M_2['m00'])
		cy_2 = int(M_2['m01']/M_2['m00'])
		print cx_2,cy_2

		rect = cv2.minAreaRect(cnt)
		box = cv2.boxPoints(rect)
		box = np.int0(box)
		cv2.circle(frame,(cx,cy), 4, (0,0,255), -1)
		cv2.imshow("output",frame)

		#PID
		l_x = PID()
		l_x.setPoint(cx_2)
		x_s = l_x.update(cx)

		l_y = PID()
		l_y.setPoint(cy_2)
		y_s = l_y.update(cy)


		k = cv2.waitKey(1)&0xFF
		if k==27:
			break

	except:
		pass
cap.release()
cv2.destroyAllWindows()

