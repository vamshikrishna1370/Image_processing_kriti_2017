import numpy as np
import cv2
import time
import serial
s = serial.Serial('/dev/ttyACM0', 115200)
s.flushInput()
s.flushOutput()
cap=cv2.VideoCapture(1)
count =0
countsm =0
cumx=0
cumy=0
cxb=0
cyb=0
class PID:
	"""
	Discrete PID control
	"""

	def __init__(self, P=0.21, I=0.00, D=0.06666, Derivator=0, Integrator=0, Integrator_max=500, Integrator_min=-500):

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
	#print "i am at 1"	
	cx=1370
	cy=1370	
	ret,frame=cap.read()
	#print ret
	cv2.imshow("Bachi",frame)
	hsv=cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
	#print "i am at 2"	
	lower_bound=np.array([0,199,130])
	upper_bound=np.array([27,242,255])

	lower_boundb=np.array([0,0,239])
	upper_boundb=np.array([179,70,255])
	#print "i am at 3"
	mask=cv2.inRange(hsv,lower_bound,upper_bound)
	maskb=cv2.inRange(hsv,lower_boundb,upper_boundb)
	#res=cv2.bitwise_and(frame,frame,mask=mask)
	cv2.imshow("mask",mask)
	cv2.imshow("maskb",maskb)
	try:
		#ret, bin_img = cv2.threshold(mask, 100, 255, cv2.THRESH_BINARY)
	#	print "i am at 4"
		im2,contours,hierarchy = cv2.findContours(mask,2, 2)
		im2b,contoursb,hierarchyb = cv2.findContours(maskb,2, 2)	
		
		
		#print M
		#area = cv2.contourArea(cnt)
		
		#print area
		if (len(contours) > 0 ):
			cnt = contours[0]
	#		print "i am at 5"
			M = cv2.moments(cnt)
	#		print "i am at 6"
			if(int(M['m00'])!=0):
				cx = int(M['m10']/M['m00'])
		        cy = int(M['m01']/M['m00'])
			rect = cv2.minAreaRect(cnt)
			box = cv2.boxPoints(rect)
			box = np.int0(box)
	#		print "i am at 7"
			#cv2.drawContours(frame,[box],0,(0,0,255),2)
			cv2.circle(frame,(cx,cy), 4, (0,0,255), -1)
			#cv2.circle(frame,(401,167), 4, (0,255,0), -1)		
			cv2.imshow("output",frame)
	#		print "i am at 8"
			
		else:
			print "no laser found on screen"
			cx = 1370
			cy = 1370
	#		print "i am at 9"
		#print cx,cy
		
		if (len(contoursb) > 0):
	#		print "i am at 10"
			cntb = contoursb[0]
	#		print "i am at 11"
			Mb = cv2.moments(cntb)
	#		print "i am at 12"
			
			if(int(Mb['m00'])!=0):			
				if(count<10):	
					cxbt = int(Mb['m10']/Mb['m00'])
		        	cybt = int(Mb['m01']/Mb['m00'])
					if(countsm==0):
						cumx = cxbt
						cumy = cybt
						countsm = countsm +1

					else if( (cumx)/(countsm) - (cxbt)  < 30 and (cumy)/(countsm) - (cybt)  < 30 ):
						cumx = cumx  + cxbt
						cumy = cumy + cybt
						countsm =countsm+1	
					else if( (cumx)/(countsm) - (cxbt)  > 30 or (cumy)/(countsm) - (cybt)  > 30 and countsm < 2):
						cumx=0
						cumy=0
						countsm =0

					cxb = cxbt
					cyb = cybt 
					

					count =count +1					
				else if(count==10 or count>10):
					cxb = cumx/countsm
					cyb = cumy/countsm

				rectb = cv2.minAreaRect(cntb)
				boxb = cv2.boxPoints(rectb)
				boxb = np.int0(boxb)
				cv2.circle(frame,(cxb,cyb), 4, (0,255,0), -1)
			else:
				print "destination fixed at "+str(cxb,cyb)
				
			print "i am at 13"
			#cv2.drawContours(frame,[box],0,(0,0,255),2)
			cv2.circle(frame,(cxb,cyb), 4, (0,255,0), -1)
			#cv2.circle(frame,(401,167), 4, (0,255,0), -1)		
			cv2.imshow("output",frame)
	#		print "i am at 14"
		else:
			print "no object detected"
			#cxb = 1370
			#cyb = 1370
	#		print "i am at 15"
		#print cx,cy
	
		
		l_x = PID()

		if(cx!=1370 ):
			l_x.setPoint(cxb)			
			x_s = l_x.update(cx)
	#		print "i am at 16"
		#print x_s

		l_y = PID()

		if(cy!=1370 ):	
			l_y.setPoint(cyb)	
			y_s = l_y.update(cy)
	#		print "i am at 17"
			print str(x_s)+';'+str(y_s)+';'+str(0)+';'
			print cx,cy,cxb,cyb			
			#s.flushInput()
			s.flushOutput()
			s.write(str(x_s)+';'+str(y_s)+';'+str(1370)+'; $') 
			
		else:
			print "destination not identified"		
	#		print "i am at 18"
		k = cv2.waitKey(97)&0xFF
		if k==27:
	#		print "i am at 19"			
			break

	except Exception as e:
		print( "Error: %s" % str(e) )
		print "i am at pass loop"		
		pass
		
cap.release()
cv2.destroyAllWindows()
