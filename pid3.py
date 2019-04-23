import time
def pid_bachi( y, yc, h=1, Ti=1, Td=1, Kp=0, u0=0, e0=0 ):
	print "function called"	
	# Step variable
	k = 0

	# Initialization
	ui_prev = u0
	e_prev = e0

	while True:

		# Error between the desired and actual output
		e = yc - y
		print e +' is the updated error. '

		# Integration Input
		ui = ui_prev + 1.0/Ti * h*e
		# Derivation Input
		ud = 1.0/Td * (e - e_prev)/h

		# Adjust previous values
		e_prev = e
		ui_prev = ui

		# Calculate input for the system
		u = Kp * (e + ui + ud)
		
		k += 1.0

		yield u

print 'hllo'
while 1:
	time.sleep(1)
	pid_bachi(10,100)

print 'hellohello'
