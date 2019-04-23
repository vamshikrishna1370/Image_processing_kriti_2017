import numpy as np
import cv2
import time
import serial
s = serial.Serial('/dev/ttyACM0', 115200)
s.flushInput()
s.flushOutput()
cap=cv2.VideoCapture(1)

class PID:
	"""
	Discrete PID control
	"""

	def __init__(self, P=0.13333, I=0.0, D=0.0666, Derivator=0, Integrator=0, Integrator_max=500, Integrator_min=-500):

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


while(1):
 	#time.sleep(1)
	
	cx=1370
	cy=1370	
	ret,frame=cap.read()
	#print ret
	cv2.imshow("Bachi",frame)
	hsv=cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
	
	lower_bound=np.array([0,0,225])
	upper_bound=np.array([0,0,255])

	mask=cv2.inRange(hsv,lower_bound,upper_bound)
	
	#res=cv2.bitwise_and(frame,frame,mask=mask)
	cv2.imshow("mask",mask)
	
	try:
		#ret, bin_img = cv2.threshold(mask, 100, 255, cv2.THRESH_BINARY)
		
		im2,contours,hierarchy = cv2.findContours(mask,2, 2)
		
		
		#print M
		#area = cv2.contourArea(cnt)
		
		#print area
		if (len(contours) > 0):
			cnt = contours[0]
			
			M = cv2.moments(cnt)
			
			cx = int(M['m10']/M['m00'])
		        cy = int(M['m01']/M['m00'])
			rect = cv2.minAreaRect(cnt)
			box = cv2.boxPoints(rect)
			box = np.int0(box)
			
			#cv2.drawContours(frame,[box],0,(0,0,255),2)
			cv2.circle(frame,(cx,cy), 4, (0,0,255), -1)
			cv2.circle(frame,(401,167), 4, (0,255,0), -1)		
			cv2.imshow("output",frame)
			
		else:
			print "no object detected"
			cx = 1370
			cy = 1370
		#print cx,cy
	
		
		l_x = PID()
		l_x.setPoint(305)
		if(cx!=1370):
			
			x_s = l_x.update(cx)
		#print x_s

		l_y = PID()
		l_y.setPoint(142)
		if(cy!=1370):		
			y_s = l_y.update(cy)
			
			print str(x_s)+';'+str(y_s)+';'+str(0)+';'
			#s.flushInput()
			s.flushOutput()
			s.write(str(x_s)+';'+str(y_s)+';'+str(1370)+'; $') 
		else:
			print "object not identified"		
		k = cv2.waitKey(97)&0xFF
		if k==27:
			break

	except:
		print "i am at pass loop"		
		pass
		
cap.release()
cv2.destroyAllWindows()
