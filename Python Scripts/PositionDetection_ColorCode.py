# Import Libraries
import picamera
from picamera.array import PiRGBArray
import cv2
import time
import serial
import math
import numpy as np
from collections import deque
#from imutils.video.pivideostream import PiVideoStream
#import imutils

def setup():
	# ------ Initiate the program -------
	global Data, DataFname
	# Data saving
	Data = np.zeros((1500, 16))
	# ALTER FNAME EACH TEST!
	DataFname = "/home/pi/Desktop/AutoPilot/CV/Data/Data_Collection_Controlled_1.csv"
	
	# Declaring values used in the main loop
	global kernel2, kernel3, kernel4, kernel5
	kernel2 = np.ones((2, 2))
	kernel3 = np.ones((3, 3))  # Used for closing operation
	kernel4 = np.ones((4, 4))
	kernel5 = np.ones((5, 5))
	
	# Declaring control values
	global m,b, Z, P, K
	m = 160
	b = 45
	Z = [0.25, 0.315, 0.21]
	P = [15.0, 5.0, 10.0]
	K = [0.5, 3.0, 6.0]
	global u, a0, c1, c2, c3, dT, t1
	u = [0, 0, 0]
	v = [0, 0, 0, 0, 0, 0]
	a0 = [0, 0, 0]
	c1 = [0, 0, 0]
	c2 = [0, 0, 0]
	c3 = [0, 0, 0]
	dT = 0.015
	t1 = time.clock()
	global u_old, theta_old, position_old, theta_threshold, x_threshold, y_threshold
	theta_old = 0
	position_old = [0, 0]
	u_old = [0, 0, 0]
	theta_threshold = 10
	x_threshold = 20
	y_threshold = 20 
	
	# Open serial port
	global ser
	PORT = '/dev/ttyUSB0'  # Sets the serial port as the USB port
	BAUD_RATE = 115200  # Baud rate which Xbee is configured to read/send data at
	#ser = serial.Serial(PORT, BAUD_RATE)
	
	# Defining the size of the image the camera will take and camera settings
	global rows, cols, imagefocus, portion, offset
	rows = 240
	cols = 320
	# Define the range of the image which we care about
	portion = 4
	# Define the portion as 2/'portion' of img (i.e. center half is portion=4)
	offset = 0
	imagefocus = np.array(
		[[cols / 2 - cols / portion  + offset, cols / 2 + cols / portion  + offset],
		 [rows / 2 - rows / portion, rows / 2 + rows / portion]])
	
	# Camera Settings
	global cap
	#cap = PiVideoStream(resolution = (cols,rows), framerate = 60).start()
	cap = cv2.VideoCapture(0)
	ret = cap.set(cv2.CAP_PROP_FRAME_WIDTH, cols)
	print(ret)
	ret = cap.set(cv2.CAP_PROP_FRAME_HEIGHT, rows)
	print(ret)
	
	# Tracking initialization
	global Buffer_Length, Trail
	Buffer_Length = 64
	Trail = deque(maxlen = Buffer_Length)

	# Import Flame and set alphas, then create rotated versions
	global jet_scale, Flame_front, Flame_rear, Flame_left, Flame_right
	jet_scale = 10
	Flame_imported = cv2.imread("Flame.png",-1)
	Flame = cv2.resize(Flame_imported, dsize = (cols,rows)) 
	non_white_cells = np.where(Flame != [255,255,255,0])
	Flame [non_white_cells[0]][non_white_cells[1]][3] = 255
	Flame_front = Flame
	M = cv2.getRotationMatrix2D((cols/2,rows/2),180,1)
	Flame_rear = cv2.warpAffine(Flame,M,(cols,rows))
	M = cv2.getRotationMatrix2D((cols/2,rows/2),90,1)
	Flame_left = cv2.warpAffine(Flame,M,(cols,rows))
	M = cv2.getRotationMatrix2D((cols/2,rows/2),270,1)
	Flame_right = cv2.warpAffine(Flame,M,(cols,rows))
	
	# Shuttle Projection
	global shuttle, real_length, real_wingspan
	Shuttle_imported = cv2.imread("shuttle.png",-1)
	print(np.shape(Shuttle_imported))
	shuttle = cv2.resize(Shuttle_imported[75:-275,125:-25], dsize = (np.shape(Shuttle_imported)[1]/20,np.shape(Shuttle_imported)[0]/20)) 
	shuttle = np.array(np.append(shuttle,np.zeros((np.shape(shuttle)[0],np.shape(shuttle)[1],1)), axis = 2))
	white_cells = np.where(shuttle == [255, 255, 255, 0])
	shuttle [white_cells[0]][white_cells[1]][3] = 255

	real_length = 0.2032 # 8 inches in meters, rough length of shuttle
	real_wingspan = 0.08
	
	#location of jet anchors in x,y at standard length (26.45)
	global standard_length, standard_wingspan, Anchors
	standard_length = 26.45
	standard_wingspan = (standard_length / real_length) * real_wingspan
	Right_rear = [-15,-5]
	Right_front = [-5,15]
	Left_rear = [-Right_rear[0],Right_rear[1]]
	Left_front = [-Right_front[0],Right_front[1]]
	Front = [0,25]
	Rear = [0,-25]
	Anchors = np.array([Right_front,Left_front,Right_rear,Left_rear,Rear,Front])
	
	# Declare HSV marker values
	global Cyan_MIN, Cyan_MAX, Magenta_MIN, Magenta_MAX, Yellow_MIN, Yellow_MAX
	# Cyan - tip
	Cyan_MIN = np.array([80, 70, 50],np.uint8)
	Cyan_MAX = np.array([130, 255, 255],np.uint8)
	# Yellow - center
	Yellow_MIN = np.array([20, 50, 50],np.uint8)
	Yellow_MAX = np.array([60, 255, 255],np.uint8)
	# Magenta - wingtip
	Magenta_MIN = np.array([140, 30, 40],np.uint8)
	Magenta_MAX = np.array([180, 255, 255],np.uint8)
	
	global center, tip, wing
	center = [25,20]
	tip = [25,25]
	wing = [20,25]

	# Undistortion data
	global mtx, dist
	mtx = np.load('CameraCalibrationMatrix.npy')
	dist = np.load('CameraDistortionValues.npy')
	
	time.sleep(2.0)
	return
	
def Target_Detection():
	global Target, tip_Target, theta_Target, Rotation
	Target=[cols/3-10,rows/3+5]
	tip_Target=[cols/3-10,rows/3-15]
	# ------ Use detection once to find the parking spot -------
	#ret, frame_o = cap.read()
	#image = frame_o.reshape((rows, cols, 3))
	## Converts img data to grayscale, Cuts image down to focus area
	#imageGray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
	#img = imageGray[imagefocus[1, 0]:imagefocus[1, 1],
					#imagefocus[0, 0]:imagefocus[0, 1]]
	## Threshold the image
	#TH, imgTH = cv2.threshold(img, 100, 255, cv2.THRESH_BINARY)
	## Closing the image to complete the shape of the shuttle
	#closed = cv2.morphologyEx(imgTH, cv2.MORPH_ERODE, kernel5, iterations=1)
	##cv2.imshow("Objects", closed)
	## Find the Contours of the img, then specificially calling out the longest one
	#im_dummy, contours, hierarchy = \
		#cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
	#cnt=contours[np.argmax(len(contours[:]))]
	## Moment and centroid definition
	#M = cv2.moments(cnt)
	#Target = [int(M['m10'] / M['m00']), int(M['m01'] / M['m00'])] # y, x
	# Finds the avg of the contour, defines tip as furthest from the avg.
	# This calc is only usefull because of shape of ship
	#dif = np.subtract(cnt[:], Target[:])
	#normal = np.sqrt(np.add(np.multiply(dif[:, 0, 0], dif[:, 0, 0]),
	#                        np.multiply(dif[:, 0, 1], dif[:, 0, 1])))
	#tip_Target = cnt[np.argmax(normal)][0]
	# Angle calculation based on slopes of line
	dy = -Target[0] + tip_Target[0]
	dx = -Target[1] + tip_Target[1]
	theta_Target = math.degrees(math.atan2(dy, dx))
	Rotation = np.array(
		[[math.cos(theta_Target), -math.sin(theta_Target)],
		 [math.sin(theta_Target), math.cos(theta_Target)]])
	return

def Image_Read():
	global img, image, imageHSV, image_focused
	ret, frame = cap.read()
	img = frame.reshape((rows, cols, 3))
	image_undistorted = cv2.undistort(img, mtx, dist, None) 
	image_focused = image_undistorted[imagefocus[1, 0]:imagefocus[1, 1],
									  imagefocus[0, 0]:imagefocus[0, 1]]
	#cv2.imshow('Focused',image_focused)
	# Converts img data to HSV
	imageHSV = cv2.cvtColor(image_focused, cv2.COLOR_BGR2HSV)			
	return

def Image_Processing():
	global Cyan_threshed, Yellow_threshed, Magenta_threshed
	# Threshold the image
	Cyan_threshed = cv2.inRange(imageHSV, Cyan_MIN, Cyan_MAX)
	Yellow_threshed = cv2.inRange(imageHSV, Yellow_MIN, Yellow_MAX)
	Magenta_threshed = cv2.inRange(imageHSV, Magenta_MIN, Magenta_MAX)
	cv2.imshow('Cyan',Cyan_threshed)
	cv2.imshow('YLW',Yellow_threshed)
	cv2.imshow('Magenta',Magenta_threshed)
	cv2.waitKey(1)	
	return

def ContourDetection_findContours():
	global center, tip, wing
	im_dummy, Cyan_contours, hierarchy = \
		cv2.findContours(Cyan_threshed, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
	im_dummy, Yellow_contours, hierarchy = \
		cv2.findContours(Yellow_threshed, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
	im_dummy, Magenta_contours, hierarchy = \
		cv2.findContours(Magenta_threshed, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
	try:
		Yellow_cnt=sorted(Yellow_contours, key=cv2.contourArea, reverse=True)[0]
		M = cv2.moments(Yellow_cnt)
		tip = [int(M['m10'] / M['m00']), int(M['m01'] / M['m00'])] # y, x
		
		Magenta_cnt=sorted(Magenta_contours, key=cv2.contourArea, reverse=True)[0]
		M = cv2.moments(Magenta_cnt)
		center = [int(M['m10'] / M['m00']), int(M['m01'] / M['m00'])] # y, x
		
		Cyan_cnt=sorted(Cyan_contours, key=cv2.contourArea, reverse=True)[0]
		M = cv2.moments(Cyan_cnt)
		wing = [int(M['m10'] / M['m00']),int(M['m01'] / M['m00'])] # y, x
		#wing =[10,20]
	except:
		print "Wow something broke the contours"
	return

def Position_Calc():
	global position, theta, theta_Relative, ship_length, ship_wingspan, scale, velocity, dT
	dy = -center[0] + tip[0]
	dx = -center[1] + tip[1]
	dy_wing = -center[0] + wing[0]
	dx_wing = -center[1] + wing[1]
	theta = math.degrees(math.atan2(dy, dx))
	# Define relative coordinates for command line
	position = np.dot(Rotation,[center[0] - Target[0], center[1] - Target[1]]).astype(int)
	theta_Relative = (theta_Target - theta)
	if theta_Relative > 180:
		theta_Relative = theta_Relative - 360
	elif theta_Relative < -180:
		theta_Relative = theta_Relative + 360
	
	# Use scale of ship to determine actual distances
	ship_length=np.linalg.norm([dx,dy]) # Pixels long
	ship_wingspan = np.linalg.norm([dx_wing,dy_wing])
	#print(ship_length)
	# using known length to determine m/pix scale
	scale = real_length / ship_length
	scale_wing = real_wingspan / ship_wingspan
	
	# Define the velocity vector of the ship
	dT = time.clock() - t1
	velocity = np.divide([theta_Relative - theta_old, position[1] * scale - position_old[1] * scale,
						  position[0] * scale - position_old[0] * scale], dT)
	return

def Command_Calc():
	global u, v, Command, u_old, theta_old, position_old
	# Calculate command coefficients
	for ii in range(0, 3):
		a0[ii] = P[ii] * dT + 2
		c1[ii] = (dT * K[ii] * Z[ii] + 2.0 * K[ii]) / a0[ii]
		c2[ii] = (dT * K[ii] * Z[ii] - 2.0 * K[ii]) / a0[ii]
		c3[ii] = (dT * P[ii] - 1.0) / a0[ii]
	# If theta not resolved yet
	v = [0, 0, 0, 0, 0, 0]
	if abs(theta_Relative) > theta_threshold:
		u[0] = c1[0] * theta_Relative + c2[0] * theta_old - c3[0] * u_old[0]
		if u[0] >= 0:
			v[0] = m*u[0] + b
			v[3] = v[0]
		elif u[0] <= 0:
			v[1] = -m*u[0] + b
			v[2] = v[1]
	else: #abs(position[0]) > x_threshold or abs(position[1]) > y_threshold:
		u[0] = c1[0] * theta_Relative + c2[0] * theta_old - c3[0] * u_old[0]
		# Position commands use scaled position to get values in meters
		u[1] = c1[1] * position[0] + c2[1] * position[0] - c3[1] * u_old[1]
		u[2] = c1[2] * position[1] + c2[2] * position[1] - c3[2] * u_old[2]
		if u[0] <= 0:
			v[0] = -m*u[0]
			v[3] = v[0]
		elif u[0] > 0:
			v[1] = m*u[0]
			v[2] = v[1]
		if u[1] < 0:
			v[0] = -m*u[1] + b + v[0]
			v[2] = v[0] + v[2]
		elif u[1] > 0:
			v[1] = m*u[0] + b + v[1]
			v[3] = v[1] + v[3]
		if u[2] < 0:
			v[4] = -m*u[2] + b
		elif u[2] > 0:
			v[5] = m*u[2] + b
	#else:
		#print('Docked!')
		#v = [0, 0, 0, 0, 0, 0]
	#print('Unsaturated v:')
	#print(v)
	for ii in range(0, 6):
		if v[ii] > 255 or v[ii] < 0:
			v[ii] = 255
		elif v[ii] > 0 and v[ii] < b:
			v[ii] = b
	# Writes to the serial port the command line of motor speeds
	Command = 'A' + str(int(v[0])) + 'B' + str(int(v[1])) + 'C' + str(int(v[2])) + \
			  'D' + str(int(v[3])) + 'E' + str(int(v[4])) + 'F' + str(int(v[5])) + 'Z'
	#ser.write(Command)
	#ser.flush()
	print ('Send to Shuttle:  ' + Command)
	u_old = u
	position_old = position  # Store position for next calculation
	theta_old = theta_Relative
	return

def Flame_Projection():
	# Flame Projection
	# Create array of anchor points as they relate to a downward facing shuttle
	anchor_points = np.divide(Anchors,(standard_length/ship_length))
	
	# Rotate image to figure out layout
	Ship_Rotation = cv2.getRotationMatrix2D((center[0],center[1]),-theta,1)
	rotated = cv2.warpAffine(image_focused,Ship_Rotation,(int(cols*2/portion),int(rows*2/portion)))
	# Projected Flames
	try:
		# Project shuttle
		Scaled_shuttle = cv2.resize(shuttle, dsize = (0,0), fx = (ship_wingspan/standard_wingspan), fy = (ship_length/standard_length))
		shuttle_size = np.shape(Scaled_shuttle)
		print(shuttle_size)
		alpha_s = 1 #Scaled_shuttle[:, :, 3] / 255.0
		alpha_l = 1.0 - alpha_s
		TL = np.array([anchor_points[4][0]+center[0]-shuttle_size[1]/2 + offset, 
					   anchor_points[4][1]+center[1]]).astype('int16')
		BR = np.add(TL, [shuttle_size[1] + offset, shuttle_size[0]]).astype('int16')
		for c in range(0, 3):
			rotated[TL[1]:BR[1], TL[0]:BR[0], c] = (alpha_s * Scaled_shuttle[:, :, c] + alpha_l * rotated[TL[1]:BR[1], TL[0]:BR[0], c])
	
		if v[5] > 0: # Front
			Scaled_Flame = cv2.resize(Flame_front, dsize = (0,0), fx = v[5]/(jet_scale*255.0), fy = v[5]/(jet_scale*255.0))
			Flame_size = np.shape(Scaled_Flame)
			alpha_s = Scaled_Flame[:, :, 3] / 255.0
			alpha_l = 1.0 - alpha_s
			TL = np.array([anchor_points[5][0]+center[0]-Flame_size[1]/2 + offset, 
						   anchor_points[5][1]+center[1]]).astype('int16')
			BR = np.add(TL, [Flame_size[1] + offset, Flame_size[0]]).astype('int16')
			for c in range(0, 3):
				rotated[TL[1]:BR[1], TL[0]:BR[0], c] = (alpha_s * Scaled_Flame[:, :, c] + alpha_l * rotated[TL[1]:BR[1], TL[0]:BR[0], c])
		if v[4] > 0: # rear
			Scaled_Flame = cv2.resize(Flame_rear, dsize = (0,0), fx = v[4]/(jet_scale*255.0), fy = v[4]/(jet_scale*255.0))
			Flame_size = np.shape(Scaled_Flame)
			alpha_s = Scaled_Flame[:, :, 3] / 255.0
			alpha_l = 1.0 - alpha_s
			TL = np.array([anchor_points[4][0]+center[0]-Flame_size[1]/2 + offset, 
						   anchor_points[4][1]+center[1]-Flame_size[0]]).astype('int16')
			BR = np.add(TL, [Flame_size[1] + offset, Flame_size[0]]).astype('int16')
			for c in range(0, 3):
				rotated[TL[1]:BR[1], TL[0]:BR[0], c] = (alpha_s * Scaled_Flame[:, :, c] + alpha_l * rotated[TL[1]:BR[1], TL[0]:BR[0], c])
		if v[0] > 0: # Right Front
			Scaled_Flame = cv2.resize(Flame_right, dsize = (0,0), fx = v[0]/(jet_scale*255.0), fy = v[0]/(jet_scale*255.0))
			Flame_size = np.shape(Scaled_Flame)
			alpha_s = Scaled_Flame[:, :, 3] / 255.0
			alpha_l = 1.0 - alpha_s
			TL = np.array([anchor_points[0][0]+center[0]-Flame_size[1] + offset,
						   anchor_points[0][1]+center[1]-Flame_size[0]/2]).astype('int16')
			BR = np.add(TL, [Flame_size[1] + offset, Flame_size[0]]).astype('int16')
			for c in range(0, 3):
				rotated[TL[1]:BR[1], TL[0]:BR[0], c] = (alpha_s * Scaled_Flame[:, :, c] + alpha_l * rotated[TL[1]:BR[1], TL[0]:BR[0], c])
		if v[2] > 0: # Right back
			Scaled_Flame = cv2.resize(Flame_right, dsize = (0,0), fx = v[2]/(jet_scale*255.0), fy = v[2]/(jet_scale*255.0))
			Flame_size = np.shape(Scaled_Flame)
			alpha_s = Scaled_Flame[:, :, 3] / 255.0
			alpha_l = 1.0 - alpha_s
			TL = np.array([anchor_points[2][0]+center[0]-Flame_size[1] + offset,
						   anchor_points[2][1]+center[1]-Flame_size[0]/2]).astype('int16')
			BR = np.add(TL, [Flame_size[1] + offset, Flame_size[0]]).astype('int16')
			for c in range(0, 3):
				rotated[TL[1]:BR[1], TL[0]:BR[0], c] = (alpha_s * Scaled_Flame[:, :, c] + alpha_l * rotated[TL[1]:BR[1], TL[0]:BR[0], c])
		if v[1] > 0: # left Front
			Scaled_Flame = cv2.resize(Flame_left, dsize = (0,0), fx = v[1]/(jet_scale*255.0), fy = v[1]/(jet_scale*255.0))
			Flame_size = np.shape(Scaled_Flame)
			alpha_s = Scaled_Flame[:, :, 3] / 255.0
			alpha_l = 1.0 - alpha_s
			TL = np.array([anchor_points[1][0]+center[0] + offset, 
						   anchor_points[1][1]+center[1]-Flame_size[0]/2]).astype('int16')
			BR = np.add(TL, [Flame_size[1] + offset, Flame_size[0]]).astype('int16')
			for c in range(0, 3):
				rotated[TL[1]:BR[1], TL[0]:BR[0], c] = (alpha_s * Scaled_Flame[:, :, c] + alpha_l * rotated[TL[1]:BR[1], TL[0]:BR[0], c])
		if v[3] > 0: # left Front
			Scaled_Flame = cv2.resize(Flame_left, dsize = (0,0), fx = v[3]/(jet_scale*255.0), fy = v[3]/(jet_scale*255.0))
			Flame_size = np.shape(Scaled_Flame)
			alpha_s = Scaled_Flame[:, :, 3] / 255.0
			alpha_l = 1.0 - alpha_s
			TL = np.array([anchor_points[3][0]+center[0] + offset,
						   anchor_points[3][1]+center[1]-Flame_size[0]/2]).astype('int16')
			BR = np.add(TL, [Flame_size[1] + offset, Flame_size[0]]).astype('int16')
			for c in range(0, 3):
				rotated[TL[1]:BR[1], TL[0]:BR[0], c] = (alpha_s * Scaled_Flame[:, :, c] + alpha_l * rotated[TL[1]:BR[1], TL[0]:BR[0], c])
	except:
		print('One of these flames got Busted')
	Ship_Rotation = cv2.getRotationMatrix2D((center[0],center[1]),theta+1.25,1)
	unrotated = cv2.warpAffine(rotated,Ship_Rotation,(int(cols*2/portion),int(rows*2/portion)))
	#cv2.imshow('projected',rotated)
	cv2.imshow('projected and returned to OG orientation',unrotated)
	return

def Position_Overlay():
	global img, Trail
	# Shows the centroid, tip, line between (Y,X)
	cv2.line(image_focused, (center[0], center[1]), (tip[0], tip[1]), [0,255,0], 2)
	cv2.circle(image_focused, (center[0], center[1]), 2, [0,0,255], 1)
	cv2.circle(image_focused, (tip[0], tip[1]), 2, [0, 255, 100], 1)
	cv2.circle(image_focused, (wing[0], wing[1]), 2, [255,0,0], 2)
	# Draw Target location
	cv2.circle(image_focused, (Target[0], Target[1]), 2, 200, 2)
	cv2.line(image_focused, (Target[0], Target[1]), (tip_Target[0], tip_Target[1]), 50, 2)
	# Draw trail
	Trail.appendleft((center[0],center[1]))
	for jj in xrange(1, len(Trail)):
		if Trail[jj-1] is None or Trail[jj] is None:
			continue # Skips iteration since no line can be drawn
		thickness = int(np.sqrt(Buffer_Length)/ float(jj+1))*2
		cv2.line(image_focused, Trail[jj-1], Trail[jj], [0,0,125], thickness)
		
	# Display images for testing purposes.
	cv2.imshow('Centroid', image_focused)
	# Display info
	print ('Orientation:', theta_Relative, "Postion:", position, 'dT', dT)
	return

def Save_Data():
	global ii, Data
	# Save data as (rel X, rel y, rel Theta, dT, abs x, abs y, abs theta)
	Data[ii] = [position[1], position[0], theta_Relative, dT,
			   u[1], u[2], u[0], v[0], v[1], v[2], v[3], v[4], v[5],
			   center[1], center[0], int(math.degrees(theta))]	
	# Write an image of the tracked object to a folder
	#IMGfname = "/home/pi/Desktop/AutoPilot/CV/ImageStore/Control_Test1_img%s.png" % i
	#cv2.imwrite(IMGfname, img)
	return

def main():
	global dT, t1, ii
	# ------ Loop to identify object and send delta command --------
	for ii in range(0,np.shape(Data)[0]):  # Run for amount of data desired
#		try:
		if True:
			dT = time.clock() - t1
			t1 = time.clock()
			Image_Read() # Read Image From Capture Object
			Image_Processing() # Thresholding and Morphology
			ContourDetection_findContours() # Find center and Tip
			Position_Calc()
			Command_Calc()
			# Optional Informational Displays
			Flame_Projection()
			Position_Overlay()
			
			Save_Data()
#		except:
#			print("Something Broke!")
		# if the `q` key was pressed, break from the loop
		if cv2.waitKey(1) & 0xFF == ord('q'):
			break
	return

if __name__ == "__main__":
	setup()
	Target_Detection()
	t0 = time.time()
	main()
	print('Avg FPS: ', 1/np.mean(Data[3:,3],0))	
	# Close Everything once main loop terminates
	cv2.destroyAllWindows()  # Close the display windows
	cap.release()
	#ser.close()  # Close the serial port
	# Save Data
	np.savetxt(DataFname, Data, delimiter=",")
