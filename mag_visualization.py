import csv
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
#matplotlib inline
import numpy as np


def read_MagData(source_table, columns=True):
    """
    Open the csv file where the magnetic data was logged via Rosbag
    Default assumes data is formatted in seperate columns. If not 
    the case, give argument "columns=False".
    """
    mag_data_file = open(source_table, "r")
    x_raw = []
    y_raw = []
    z_raw = []
    
    """
    If the data is not formatted in columns, we can simply read through each 
    line and identify x, y and z values based on the first character of each 
    line. We then simply append the part of the string that denotes the 
    component of the vector dimension and convert it to float. 
    Assumed format of the data:
        x: -6.7455
        y: -23.5343
        z: 65.5063
        ---
    The value always starts at index 3.         
    """
    
    if columns == False:
        for line in mag_data_file:
            print(line[3])
            if line[0] == "x":
                x_raw.append(float(line[3:]))
            if line[0] == "y":
                y_raw.append(float(line[3:]))
            if line[0] == "z":
                z_raw.append(float(line[3:]))
    
    """
    If the data is already seperated into different columns, we can simply use the 
    csv module of python to create an object of the reader class for the given csv
    file. "mag_data" then consists of a list of lists for row and column. First 
    index denotes row, second index column. We can then get the values by looping
    through each row. Index 0 contains time stamp, 1-3 the x,y,z components. 
    """
    
    if columns == True:
        dataReader = csv.reader(mag_data_file)
        mag_data = list(dataReader)
        #mag_data saves values in list of lists: first index denotes row, second column
        
        for index in range(1, len(mag_data)):
            x_raw.append(float(mag_data[index][1]))
            y_raw.append(float(mag_data[index][2]))
            z_raw.append(float(mag_data[index][3]))
            
    mag_vector_raw = [x_raw, y_raw, z_raw]
    
    return mag_vector_raw
      
def smooth_vector(vector, x_mean = 5, round_to = 2):
    """
    This function smoothes the vector by taking the mean over the last x_mean values
    and rounding to the decimals given by round_to. 
    """
    
    x_raw = vector[0]
    y_raw = vector[1]
    z_raw = vector[2]
    
    x_smooth = []
    y_smooth = []
    z_smooth = []
    
    for index in range(len(vector[0])):
        if index > x_mean - 1:
            x_smooth.append(np.mean(x_raw[index-x_mean:index+1]))
            y_smooth.append(np.mean(y_raw[index-x_mean:index+1]))
            z_smooth.append(np.mean(z_raw[index-x_mean:index+1]))
        else:
            x_smooth.append(x_raw[index])
            y_smooth.append(y_raw[index])
            z_smooth.append(z_raw[index])

    x_smooth = np.around(x_smooth, decimals = round_to)
    y_smooth = np.around(y_smooth, decimals = round_to)
    z_smooth = np.around(z_smooth, decimals = round_to)
    
    mag_vector_smooth = [x_smooth, y_smooth, z_smooth]
    return mag_vector_smooth

#raw_vector = read_MagData("drive1m_mag.csv")
#mag_vector = smooth_vector(read_MagData("drive1m_mag.csv"))

raw_vector = read_MagData("drivingmaybe_mag.csv")
mag_vector = smooth_vector(raw_vector)

mag_vector_length = mag_vector[0] + mag_vector[1] + mag_vector[2]    

#plt.plot(mag_vector_length)
    
#Plot the dimensions seperately:
#plt.plot(vector)
#Legend does not work yet, shows mostly random colours for the dimensions. 
#Needs to be connected to the plotted vecotr! 
#plt.legend(["x", "y", "z"])

"""
Now we can plot the vector in a 3D graph to see how the values behaved 
over time. 
"""

fig = plt.figure()
ax = plt.axes(projection='3d')

ax.plot3D(mag_vector[0], mag_vector[1], mag_vector[2], 'red')
ax.set_xlabel('x')
ax.set_ylabel('y')
ax.set_zlabel('z')
#Scale - uncomment for automatic scaling
"""
ax.set_xlim3d(-50, 0)
ax.set_ylim3d(-50,0)
ax.set_zlim3d(0,100)
"""
#For scatter plot instead of or in addition to line
ax.scatter3D(mag_vector[0], mag_vector[1], mag_vector[2]) 
#To add colout accentuation in one axis, append above with:             
#,c=z_smooth, cmap='Greens')







