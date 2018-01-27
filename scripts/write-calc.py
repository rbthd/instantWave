#!/usr/bin/env python3

from xlutils.copy import copy
from xlrd import open_workbook
from xlwt import easyxf

import sys
import os
import datetime

## Each time this python script is executed, the excel results file 
## is overwritten. Be careful not to accidentaly delete your saved
## results. 

rb = open_workbook('/home/robin/instantWave/minimu9/results.xls',formatting_info=True)
r_sheet = rb.sheet_by_index(0) #read-only copy to check the file
wb = copy(rb) #writable copy; will just write into this file
w_sheet = wb.get_sheet(0) #the sheet to write to within the writable copy

## Write new data in file

row=5 #to get nice display in the excel file
i=0 #just some counter

## Get correct data from bash script : argvs are passed as strings
## so we need to use the split function

# COSINE MATRIX
northX=[float(i) for i in sys.argv[1].split(' ')]
northY=[float(i) for i in sys.argv[2].split(' ')]
northZ=[float(i) for i in sys.argv[3].split(' ')]

eastX=[float(i) for i in sys.argv[4].split(' ')]
eastY=[float(i) for i in sys.argv[5].split(' ')]
eastZ=[float(i) for i in sys.argv[6].split(' ')]

downX=[float(i) for i in sys.argv[7].split(' ')]
downY=[float(i) for i in sys.argv[8].split(' ')]
downZ=[float(i) for i in sys.argv[9].split(' ')]

# QUATERNION
quatA=[float(i) for i in sys.argv[10].split(' ')]
quatB=[float(i) for i in sys.argv[11].split(' ')]
quatC=[float(i) for i in sys.argv[12].split(' ')]
quatD=[float(i) for i in sys.argv[13].split(' ')]

# EULER ANGLES
yaw=[float(i) for i in sys.argv[14].split(' ')]
pitch=[float(i) for i in sys.argv[15].split(' ')]
roll=[float(i) for i in sys.argv[16].split(' ')]

# ACCELERATION & MAGNETIC FIELD VECTORS
vectAccX=[float(i) for i in sys.argv[17].split(' ')]
vectAccY=[float(i) for i in sys.argv[18].split(' ')]
vectAccZ=[float(i) for i in sys.argv[19].split(' ')]

vectMagX=[float(i) for i in sys.argv[20].split(' ')]
vectMagY=[float(i) for i in sys.argv[21].split(' ')]
vectMagZ=[float(i) for i in sys.argv[22].split(' ')]

MagMagnitude=[float(i) for i in sys.argv[24].split(' ')]

line_max=int(sys.argv[23])


while i < (line_max) : ##represents the length of data

	w_sheet.write(row,0,datetime.datetime.now())
	w_sheet.write(row,1,northX[i]) #north.x
	w_sheet.write(row,2,northY[i]) #north.y
	w_sheet.write(row,3,northZ[i]) #north.z
	w_sheet.write(row,4,eastX[i]) #east.x
	w_sheet.write(row,5,eastY[i]) #east.y
	w_sheet.write(row,6,eastZ[i]) #east.z
	w_sheet.write(row,7,downX[i]) #down.x
	w_sheet.write(row,8,downY[i]) #down.y
	w_sheet.write(row,9,downZ[i]) #down.z

	w_sheet.write(row,10,quatA[i]) #quaternion A
	w_sheet.write(row,11,quatB[i]) #quaternion B
	w_sheet.write(row,12,quatC[i]) #quaternion C
	w_sheet.write(row,13,quatD[i]) #quaternion D

	w_sheet.write(row,14,yaw[i]) #YAW
	w_sheet.write(row,15,pitch[i]) #PITCH
	w_sheet.write(row,16,roll[i]) #ROLL

	w_sheet.write(row,17,vectAccX[i]) #scaled acc vect (x)
	w_sheet.write(row,18,vectAccY[i]) #scaled acc vect (y)
	w_sheet.write(row,19,vectAccZ[i]) #scaled acc vect (z)
	w_sheet.write(row,20,vectMagX[i]) #scaled mag field vect (x)
	w_sheet.write(row,21,vectMagY[i]) #scaled mag field vect (y)
	w_sheet.write(row,22,vectMagZ[i]) #scaled mag field vect (z)
	
	w_sheet.write(row,23,MagMagnitude[i]) #scaled mag field vect (z)


	row+=1
	i+=1
wb.save("/home/robin/instantWave/minimu9/results.xls")
