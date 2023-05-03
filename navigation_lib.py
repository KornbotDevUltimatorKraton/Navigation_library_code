import math 
import serial 
import requests 
import subprocess 
import struct
import time 
from itertools import count 
import cv2 
from pyzbar.pyzbar import decode
import base64 
 
def distance_RSSI(Pt,Pl,rssi):
        A = Pt-Pl 
        c_dat = (A-rssi)*0.014336239
        d = math.pow(10,c_dat) 
        return d
         
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
       wifi_strange = subprocess.check_output("iwlist scanning",shell=True)
       get_rssi = wifi_strange.decode()
       data_local = get_rssi.split("\n")[4].split("    ")[5].split(" ")       
       A_diff = data_local[0].split("=")[1].split("/")
       rssi_value = data_local[3].split("=")[1]
       #wifi_name = get_rssi.split("\n")[6].split("    ")[5].split(":")[1]
       distance_data =   distance_RSSI(int(A_diff[1]),int(A_diff[0]),float(rssi_value))
       wifi_list_rssi = {wifi_name:{'wifi_freq':get_rssi.split("\n")[3].split("    ")[5].split(":")[1],'rssi_signal':data_local[3].split("=")[1],"A":int(A_diff[1]) - int(A_diff[0]),'distance':distance_data}}
       pack_payload = {email:wifi_list_rssi}
       payload_data = pack_payload
       rssi_post_robot = requests.post("https://roboreactor.com/rssi_robot_client",json=payload_data)
       return payload_data   # Return the json value report to the back end data 
  except:
        print("Current wifi not found you are not connected")
def point_cloud_pcd_generator(camera_name,server_url,payload_command):
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
                      requests.post(server_url, json=payload_command) # Post request to the website 

def Camera_image_sensor(email,project_name,camera_name,server_url,cam_index):
             #Post the image from the camera input to the front end of the server   
             
             while True:
                      # Capture a frame from the video stream
                      pos_mem_local = {}
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
                      # Encode the JPEG data in base64
                      jpeg_b64 = base64.b64encode(jpeg).decode("utf-8")
                      print(jpeg_b64)
                      # Send the frame to the server
                      requests.post(server_url, json={email:{project_name:{"video":jpeg_b64,"position":pos_mem_local}}}) # Post request to the website 

def data_projection_post(email,payload_projection):
        try:
           post_projection = requests.post("http://0.0.0.0:5340/Multiplenode_logic",json={email:payload_projection}) # Get the data payload projection to run in real-time operating in mutlitasking function of sensor and actuator processing 
           return post_projection.json() # return the projection data from the post request of the data projection 
        except: 
              print("Error communicating with the data projection server ")                  
   

