# ENGR 101: Data Visualization 
## Python Script Setup
1. Download ZIP file "engr101-master.zip" from https://github.com/trudiQ/engr101/tree/master 
2. Install Anaconda: https://www.anaconda.com/products/individual 
3. Open Anaconda Prompt
4. Navigate to the "engr101-master", for example:
> cd "./engr101-master"
5. Create and activate a new virtual environment named "engr101" (this will install all the required python libraries):
> conda env create -f environment.yml \
> conda activate engr101
6. Open Lesson 29 using the Arduino IDE, check port number, say 'COMP7' (WINDOWS) or '' (MAC) 
7. Open the python file and find the line "ser = serial.Serial('COM6', 9600)"
8. Replace 'COM6' with your port number, save, and close the python file
9. Run the python file:
> python arduino-lesson29_livePlot.py
