import os 
import math 
import smbus 
import math 
import serial 
import requests 
import subprocess 
import struct
import time 
import numpy as np 
from itertools import count 
import cv2 
from pyzbar.pyzbar import decode
import base64 
#from mpu6050 import mpu6050 
def distance_RSSI(Pt,Pl,rssi):
        A = Pt-Pl 
        c_dat = (A-rssi)*0.014336239
        d = math.pow(10,c_dat) 
        return d
def data_projection_post(email,payload_projection):
        try:
           post_projection = requests.post("http://0.0.0.0:5340/Multiplenode_logic",json={email:payload_projection}) # Get the data payload projection to run in real-time operating in mutlitasking function of sensor and actuator processing 
           return post_projection.json() # return the projection data from the post request of the data projection 
        except: 
              print("Error communicating with the data projection server ")           
              
def Lidar_info_report(email,lidar_name,unit_converter,serial_data):
     for i in count(0):     
            #print("Activating lidar system ",email+","+lidar_name) #Get the lidar system name 
            data = serial_data.read(1)  # read the start byte
            if data != b'\xFA':
                continue  # start byte not found, skip this packet
            data += serial_data.read(1)  # read the index byte
            data += serial_data.read(2)  # read the speed bytes
            data += serial_data.read(4)  # read the timestamp bytes
            data += serial_data.read(360*3)  # read the data bytes (360 values of 3 bytes each)
            data += serial_data.read(2)  # read the checksum bytes
            # decode the data
            data = data[2:-2]  # remove the index, speed, timestamp, and checksum bytes
            ranges = []
            angles = []
            for i in range(0, len(data), 3):
                     byte3, byte2, byte1 = struct.unpack('BBB', data[i:i+3])
                     dist_mm = ((byte1 << 16) | (byte2 << 8) | byte3)
                     range_m = dist_mm/unit_converter # convert mm to cm
                     angle_deg = i / 3  # calculate the angle from the index
                     angles.append(angle_deg)
                     ranges.append(range_m)
            #print the ranges and angles
            #print("Ranges:", ranges,len(ranges))
            #print("Angles:", angles,len(angles))
            pack_payload_lidar = {lidar_name:{"Ranges":ranges,"Angles":angles}} # Get the range and angle from the lidar to post back into the server 
            lidar_device = {email:pack_payload_lidar}
            lidar_post = requests.post("https://roboreactor.com/lidar_post_data",json=lidar_device) 
            return lidar_device  

def rssi_distance_report(email,wifi_name):  
  #Generate the navigation system 
  try:
       wifi_strange = subprocess.check_output("iwconfig",shell=True)
       get_rssi = wifi_strange.decode()
       data_local = get_rssi.split("\n")[5]     
       A_diff = data_local.split(" ")[11].split("=")[1].split("/")
       rssi_value = data_local.split(" ")[14].split("=")[1]
       #wifi_name = get_rssi.split("\n")[6].split("    ")[5].split(":")[1]
       distance_data =   distance_RSSI(int(A_diff[1]),int(A_diff[0]),float(rssi_value))
       #print("Data_local_wifi",A_diff,rssi_value,distance_data,get_rssi.split("\n")[1].split(" ")[12].split(":")[1])
       wifi_list_rssi = {wifi_name:{'wifi_freq':get_rssi.split("\n")[1].split(" ")[12].split(":")[1],'rssi_signal':rssi_value,"A":int(A_diff[1]) - int(A_diff[0]),'distance':distance_data}}
       pack_payload = {email:wifi_list_rssi}
       payload_data = pack_payload
       rssi_post_robot = requests.post("https://roboreactor.com/rssi_robot_client",json=payload_data)
       return payload_data   # Return the json value report to the back end data 
  except:
        print("Current wifi not found you are not connected")
def point_cloud_pcd_generator(camera_name,payload_command):
             #Post the image from the camera input to the front end of the server   
             while True:
                      # Capture a frame from the video stream
                      ret, frame = camera_name.read()
                      # Check if the video has ended
                      if not ret:
                            break
                      # Encode the frame in JPEG format
                      ret, jpeg = cv2.imencode('.jpg', frame)
                      # Encode the JPEG data in base64
                      jpeg_b64 = base64.b64encode(jpeg).decode("utf-8")
                      print(jpeg_b64)
                      # Send the frame to the server
                      requests.post("https://roboreactor.com/update_image", json=payload_command) # Post request to the website  for the update poit cloud data into the server  

def Camera_image_sensor(email,project_name,camera_name,cam_index,device_name):
             #Post the image from the camera input to the front end of the server   
             pos_mem_local = {}
             while True:
                      # Capture a frame from the video stream
                      
                      ret, frame = camera_name.read()
                      # Check if the video has ended
                      if not ret:
                            break
                      # Encode the frame in JPEG format
                      ret, jpeg = cv2.imencode('.jpg', frame)
                      gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                      qr_codes = decode(gray)
                      for qr_code in qr_codes:
                             data = qr_code.data.decode('utf-8')
        
                             # Draw a rectangle around the QR code
                             x, y, w, h = qr_code.rect
                             cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
                             pos_mem_local[project_name] = {'cam_index':cam_index,'message':data,'X':x,'Y':y}
                             # Draw the data on the frame
                             cv2.putText(frame, data, (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                             #return {email:{project_name:pos_mem_local}}
                      # Encode the JPEG data in base64
                      jpeg_b64 = base64.b64encode(jpeg).decode("utf-8")
                      #print(jpeg_b64)
                      print({email:{device_name:pos_mem_local}})
                      data_projection_post(email,{device_name:pos_mem_local})
                      # Send the frame to the server
                      requests.post('https://roboreactor.com/update_image', json={email:{project_name:{"video":jpeg_b64,"position":pos_mem_local}}}) # Post request to the website                 
                      # Return the data of the camera system positioning and project 
def wifi_localization_accesspoint(email,ap_dat1,ap_dat2,ap_dat3,R1,R2,R3,A,N): #  
                        # Access point coordinates
                        AP1 = np.array(ap_dat1)
                        AP2 = np.array(ap_dat2)
                        AP3 = np.array(ap_dat3)
                        # RSSI values (in dB) 
                        r1 = R1
                        r2 = R2
                        r3 = R3
                        # Distance calculation based on RSSI value (you need to calibrate this based on your own setup)
                        def calculate_distance(rssi, a=A, n=N):
                                       return 10**((a - rssi) / (10 * n))

                        # Function to update the coordinates of an access point
                        def update_ap_coordinates(ap_index, new_coordinates):
                             global AP1, AP2, AP3
                             if ap_index == 1:
                                 AP1 = np.array(new_coordinates)
                             elif ap_index == 2:
                                 AP2 = np.array(new_coordinates)
                             elif ap_index == 3:
                                 AP3 = np.array(new_coordinates)
                             else:
                                print("Invalid access point index")

                        # Calculate the distances from the access points to the device
                        d1 = calculate_distance(r1)
                        d2 = calculate_distance(r2) 
                        d3 = calculate_distance(r3)
                        # Trilateration calculation
                        A = 2 * np.array([AP2 - AP1, AP3 - AP1])
                        b = np.array([d1**2 - d2**2 - AP1[0]**2 + AP2[0]**2 - AP1[1]**2 + AP2[1]**2,
                        d1**2 - d3**2 - AP1[0]**2 + AP3[0]**2 - AP1[1]**2 + AP3[1]**2])
                        x, y = np.linalg.solve(A, b)
                        # Print the calculated position
                        print("Position: ({}, {})".format(x, y))
                        # Update the coordinates of AP2
                        new_coordinates = [x, y]
                        update_ap_coordinates(2, new_coordinates)
                        # Recalculate the distances and position
                        d2 = calculate_distance(r2)
                        A = 2 * np.array([AP2 - AP1, AP3 - AP1])
                        b = np.array([d1**2 - d2**2 - AP1[0]**2 + AP2[0]**2 - AP1[1]**2 + AP2[1]**2,
                        d1**2 - d3**2 - AP1[0]**2 + AP3[0]**2 - AP1[1]**2 + AP3[1]**2])
                        x, y = np.linalg.solve(A, b)
                        # Print the updated position
                        print("Updated position: ({}, {})".format(x, y))
                        res_wifi_locz = requests.post("https://roboreactor.com/wifi_locz_record",json={email:{'position':{'x':x,'y':y}}})
                        print(res_wifi_locz.json())


'''
def odometry_sensor(email,sensor_type,i2c_addr):
                print("Activate the odometry sensor") # Choose the odometry sensor type to send the odometry data into the server 
                #Input the sensor data to classify odometry sensor 
                if sensor_type == "MPU6050":
                             sensor = mpu6050(i2c_addr) 
                             x_angle = y_angle = z_angle = 0
                             last_time = time.time()
                             # Register
                             power_mgmt_1 = 0x6b
                             power_mgmt_2 = 0x6c
                             AngleDegX  = 0 
                             AngleDegY = 0 
                             AngleDegZ = 0 
                             def read_byte(reg):
                                   return bus.read_byte_data(address, reg)
 
                             def read_word(reg):
                                 h = bus.read_byte_data(address, reg)
                                 l = bus.read_byte_data(address, reg+1)
                                 value = (h << 8) + l
                                 return value
 
                             def read_word_2c(reg):
                                    val = read_word(reg)
                                    if (val >= 0x8000):
                                        return -((65535 - val) + 1)
                                    else:
                                        return val
 
                             def dist(a,b):
                                  return math.sqrt((a*a)+(b*b))
 
                             def get_y_rotation(x,y,z):
                                    radians = math.atan2(x, dist(y,z))
                                    return -math.degrees(radians)
 
                             def get_x_rotation(x,y,z):
                                   radians = math.atan2(y, dist(x,z))
                                   return math.degrees(radians) 
                             
                             os.system("sudo chmod -R 777 /dev/i2c-1") # give the permission to the i2c  device tree 
                             bus = smbus.SMBus(1) # bus = smbus.SMBus(0) fuer Revision 1
                             address = 0x68       # via i2cdetect
                             # Aktivieren, um das Modul ansprechen zu koennen
                             bus.write_byte_data(address, power_mgmt_1, 0)
                             print("Gyroscope")
                             print("--------")
                             gyroskop_xout = read_word_2c(0x43)
                             gyroskop_yout = read_word_2c(0x45)
                             gyroskop_zout = read_word_2c(0x47) 
                             # Accelleration 
                             accel_x = []
                             accel_y = [] 
                             accel_z = [] 
                             # Velocity_calculation 
                             velocity_x = [0]
                             velocity_y = [0]
                             velocity_z = [0]
                             # Distance_calculation 
                             position_x = [0] 
                             position_y = [0]
                             position_z = [0]
                             def Distance_calculation(dt):
     
                                def x_accel_data(dt): 
                                      if len(accel_x) >=2:
                                           for i in range(0,len(accel_x)-1):
                                                   vx = velocity_x[i-1] + (accel_x[i] + accel_x[i-1])/2*dt                       
                                                   velocity_x.append(vx)
                             
                                           for i in range(0,len(accel_x)-1):
                                                   px = position_x[i-1] + (velocity_x[i] + velocity_x[i-1])/2*dt
                                                   position_x.append(px)
                                def y_accel_data(dt):
                                           if len(accel_y) >=2:
                                                for i in range(0,len(accel_y)-1):
                                                    vy = velocity_y[i-1] + (accel_y[i] + accel_y[i-1])/2*dt
                                                    velocity_y.append(vy) 
                          
                                                for i in range(0,len(accel_y)-1):
                                                          py = position_y[i-1] + (velocity_y[i] + velocity_y[i-1])/2*dt
                                                          position_y.append(py) 
                                def z_accel_data(dt):
                                           if len(accel_z) >=2: 
                                                for i in range(0,len(accel_y)-1):
                                                           vz = velocity_z[i-1] + (accel_z[i] + accel_z[i-1])/2*dt
                                                           velocity_z.append(vz) 
                                                for i in range(0,len(accel_z)-1):
                                                          pz = position_z[i-1] + (velocity_z[i] + velocity_z[i-1])/2*dt
                                                          position_z.append(pz)   
                                x_accel_data(dt)
                                y_accel_data(dt)
                                z_accel_data(dt) 
                for i in count(0):
                        start_time = time.time() # get the start time
                        print("Robot Gyroscope")
                        print("---------------------")
                        beschleunigung_xout = read_word_2c(0x3b)
                        beschleunigung_yout = read_word_2c(0x3d)
                        beschleunigung_zout = read_word_2c(0x3f)
                        accel_data = sensor.get_accel_data()
                        gyro_data = sensor.get_gyro_data()
                        # Calculate the change in time since the last reading
                        current_time = time.time()
                        dt = current_time - last_time
                        last_time = current_time
                        # Calculate the change in rotation along all three axes based on the gyroscope data
                        dx = gyro_data['x'] * dt
                        dy = gyro_data['y'] * dt
                        dz = gyro_data['z'] * dt
                        Distance_calculation(dt)
                        # Add the change in rotation to the current rotation along all three axes
                        x_angle += dx
                        y_angle += dy
                        z_angle += dz
                        beschleunigung_xout_skaliert = beschleunigung_xout / 16384.0
                        beschleunigung_yout_skaliert = beschleunigung_yout / 16384.0
                        beschleunigung_zout_skaliert = beschleunigung_zout / 16384.0
                        AngleDegX = math.degrees(beschleunigung_xout_skaliert)
                        AngleDegY = math.degrees(beschleunigung_yout_skaliert)
                        AngleDegZ = math.degrees(beschleunigung_zout_skaliert)
                        print("AngleDegX",(AngleDegX))
                        print("AngleDegY",(AngleDegY))
                        print("AngleDegZ",(AngleDegZ))
                        print('Acceleration: x={:.2f}g, y={:.2f}g, z={:.2f}g'.format(
                        accel_data['x'], accel_data['y'], accel_data['z'])) 
                        print('Angles: x={:.2f}deg, y={:.2f}deg, z={:.2f}deg'.format(
                        x_angle, y_angle, z_angle))
                        data_trans = {"AngleDegX":AngleDegX,"AngleDegY":AngleDegY,"AngleDegZ":z_angle,"Accel_X":accel_data['x'],"Accel_Y":accel_data['y'],"Accel_Z":accel_data['z'],"dx":position_x[len(position_x)-1]*10,"dy":position_x[len(position_x)-1]*10,"dz":position_z[len(position_z)-1]/5}
                        print(data_trans)
                        accel_x.append(data_trans.get("Accel_X")) 
                        accel_y.append(data_trans.get("Accel_Y")) 
                        accel_z.append(data_trans.get("Accel_Z"))
                        if len(accel_x) > 10:
                              accel_x.remove(accel_x[0])
                              accel_y.remove(accel_y[0]) 
                              accel_z.remove(accel_z[0]) 
                        #Distance_calculation(dt)
                        print("Accel_x",accel_x[len(accel_x)-1],velocity_x[len(velocity_x)-1],position_x[len(position_x)-1],dt)
                        print("Accel_y",accel_y[len(accel_y)-1],velocity_y[len(velocity_z)-1],position_y[len(position_y)-1],dt)
                        print("Accel_z",accel_z[len(accel_z)-1],velocity_z[len(velocity_z)-1],position_z[len(position_z)-1],dt)   
                        #print(position_x[len(position_x)-1],position_y[len(position_y)-1],position_z[len(position_z)-1])
                        #Calculate the data of distance and velocity from the acceleration of each axis of the gyroscope 
                        package_data_iot = {email:data_trans} 
                        try:
                           res_imu_data = requests.post("https://roboreactor.com/iot_data",json=package_data_iot)
                        except:
                           print("Error post request data to the server") 
'''                           