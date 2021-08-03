# Madeline Coleman, 400251330, colemm6
# Libraries: Open3D v0.12.0, Numpy v1.19.2, PySerial v3.5
# Python v3.8.5

#%% Import Libraries

import serial
import math
import open3d as o3d
import numpy as np

#%% Initialize Variables

angleCount, distCount, count = 0, 0, 0
xyz = []
angleDeg, x, y, z = 0, 0, 0, 0
num_slices = 10
points_per_slice = 72
dist = np.empty([num_slices, points_per_slice])
angle = np.empty([num_slices, points_per_slice])
y_dis = 100 #stores y displacement in millimeters

#%% Recieve Data 

s = serial.Serial("COM8", 115200)
print("Opening: " + s.name)

s.write(b'1')           #Send 0x31 so micro know to start+ measurments

while (count<num_slices):
    x = s.readline()    ##read byte from micro
    c = x.decode()      ##decode byte
    c = int(c)          ##cast string to int
    angleCount = 0
    distCount = 0

    if (c == 9999):                             ##9999 is sent when the button is pressed
        for i in range(points_per_slice*2):
            x = s.readline()                    # read one byte
            c = x.decode()                      # convert byte type to str
            if(i%2 == 0):
                angle[count][angleCount] = c    #add angle to list
                print("Angle steps: ", c)
                angleCount += 1
            else:
                dist[count][distCount] = c      #add distance to list
                print("Distance: ", c)
                distCount += 1
        count = count + 1
        print("Complete scan number ", count)
    
print("Closing: " + s.name)
s.close();

#%% Format Data and create xyz file

for i in range(num_slices):
    for j in range(points_per_slice):
        angleDeg = angle[i][j]*360/512                  #convert angle steps to degrees
        x = math.sin(math.radians(angleDeg))*dist[i][j] #calculate x coordinate
        z = math.cos(math.radians(angleDeg))*dist[i][j] #calculate z coordinate
        y = i*y_dis                                     #calculate y coordinate
        xyz.append([x, y, z])                           #add coordinates to list

file = "tof.xyz"
with open(file, 'w') as f:                              #create xyz file
    f.writelines('{0} {1} {2}\n'.format(xyz[i][0], xyz[i][1], xyz[i][2]) for i in range(len(xyz)))


#%% Create Visualization 

pcd = o3d.io.read_point_cloud(file, format='xyz')        #create point cloud from xyz file

plt = o3d.visualization.Visualizer()
plt.create_window()
plt.add_geometry(o3d.geometry.TriangleMesh().create_coordinate_frame(size=100.0))

lines = []
line_set = o3d.geometry.LineSet(points=o3d.utility.Vector3dVector(np.asarray(pcd.points)),
                              lines=o3d.utility.Vector2iVector(lines))
#create lines within each scan 
for i in range(num_slices):
    for j in range(points_per_slice):
        if j == points_per_slice - 1:
            lines.append([j + i * points_per_slice, 0 + i * points_per_slice])
        else:
            lines.append([j + i * points_per_slice, j + i * points_per_slice + 1])

    #Update plot
    plt.remove_geometry(line_set)
    line_set = o3d.geometry.LineSet(points=o3d.utility.Vector3dVector(np.asarray(pcd.points)),
                                    lines=o3d.utility.Vector2iVector(lines))
    plt.add_geometry(line_set)
    plt.poll_events()
    plt.update_renderer()

    # Join planes together
    if i != num_slices - 1:
        for j in range(0, points_per_slice):
            lines.append([j + i * points_per_slice, j + (i + 1) * points_per_slice])
plt.run()
