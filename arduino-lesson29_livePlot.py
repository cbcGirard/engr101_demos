#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#----------------------------------------------------------------------------
# Created By  : Trudi Qi (dqi@chapman.edu)
# Created Date: Fri Nov 5 2021 2:30 PM
# Modified by cbcgirard (cgirard@chapman.edu)
# version ='1.1'
# ---------------------------------------------------------------------------
#%%
# Import libraries
import time
import serial
import pandas as pd 
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# TODO: argparse for command-line override?
# portname = '/dev/cu.usbmodem64389201'
portname = '/dev/cu.usbserial-2120'

# Find the arduino serial port
ser = serial.Serial(portname, 9600)
#time.sleep(2)
print(ser.name)


# Create a new figure with 7 rows of sub-plots (axes)
fig, axes = plt.subplots(7, 1)
ax0, ax1, ax2, ax3, ax4, ax5, ax6 =axes

labels = [r'$A_X [g]$',
        r'$A_Y [g]$',
        r'$A_Z [g]$',
          'Tmp [C]',
        r'$A_X $',
        r'$A_Y$',
        r'$A_Z$']
           

#init data
lines = []
for ax,lab in zip(axes, labels):
    lines.append(ax.plot(0,0)[0])
    ax.set_ylabel(lab)
# array = np.zeros((7,100))
array = [[] for n in range(7)]

# animation function
#   - ser is the serial port of arduino
# #   - df is pandas dataframe
def animation(i, ser, array, axes,df=None): 
    # read data from serial port
    serString = ser.readline()
    
    # convert the data to string
    serString = str(serString, encoding = 'utf-8') 
    print(serString)
    # split string to an array of data elements
    dataArray = serString.split(' | ')

    dataVec=np.array([float(d.split('=')[-1]) for d in dataArray],
                     )
    
    for ii in range(3):
        dataVec[ii]=dataVec[ii]/16384
   
    for lst, d, ax, ii in zip(array,dataVec, axes, range(7)):
        # lst.append(float(df.loc[index, label]))
        lst.append(d)
        lst = lst[-100:]
        
        ax.clear()
        ax.plot(lst,"C"+str(ii))

# unsuccessful attempt to speed up matplotlib plotting routines
def fastAnimate(*args):

    # read data from serial port
    serString = ser.readline()
    
    # convert the data to string
    serString = str(serString, encoding = 'utf-8') 
    print(serString)

    # split string to an array of data elements
    dataArray = serString.split(' | ')
    dataVec=np.array([float(d.split('=')[-1]) for d in dataArray],
                     )

    for vec,d in zip(array,dataVec):
        if len(vec)==100:
            vec = vec[1:]
        vec.append(d)


    for d,l in zip(array,lines):
        l.set_data(range(len(d)),d)

    return lines



# Create a new CSV file linked to a dataFrame 
df = pd.DataFrame(columns = ["AcX", "AcY", "AcZ", "Tmp", "GyX", "GyY", "GyZ"]) # header of the CSV
df.to_csv('test.csv', index=False) # give the file name



time.sleep(1)
ser.write('a'.encode()) 
time.sleep(1)
# Makes an plotting animation by repeatedly calling 'animation' function
#   - Reading data from 'ser', plotting live data, saving data to CSV
ani = FuncAnimation(fig, animation, frames=100, fargs=(ser, array, axes, None),
                                                        interval=100)
# ani = FuncAnimation(fig, fastAnimate, frames=100, #fargs=(ser,array, lines), 
#                     interval=100)# Adjust figure layout and display the figure 
plt.subplots_adjust(left=0.15)
plt.show()

# after the window is closed, close the serial line
ser.close()
print("Serial line closed")