import csv
import time
import os

period = 0.1
t=time.time()

cmd_vel = []
with open('test.csv', mode ='r')as file:
  csvFile = csv.reader(file)

  for lines in csvFile:

    if len(lines) == 2:
      cmd_vel.append([lines[0],lines[1]])


for level,avel in cmd_vel:
  t+=period
  tmpstr = 'ros2 topic pub /husky_velocity_controller/cmd_vel geometry_msgs/Twist "{linear: {x: '+level+', y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: '+avel+'}}" -r 10'
  os.system(tmpstr)
  time.sleep(max(0,t-time.time()))

#   ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.2}}"