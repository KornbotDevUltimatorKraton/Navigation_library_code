import math 
import serial 
import requests 
import subprocess 
import struct
import time 
from itertools import count 

 
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
       wifi_name = get_rssi.split("\n")[6].split("    ")[5].split(":")[1]
       distance_data =   distance_RSSI(int(A_diff[1]),int(A_diff[0]),float(rssi_value))
       wifi_list_rssi = {wifi_name:{'wifi_freq':get_rssi.split("\n")[3].split("    ")[5].split(":")[1],'rssi_signal':data_local[3].split("=")[1],"A":int(A_diff[1]) - int(A_diff[0]),'distance':distance_data}}
       pack_payload = {email:wifi_list_rssi}
       payload_data = pack_payload
       rssi_post_robot = requests.post("https://roboreactor.com/rssi_robot_client",json=payload_data)
       return payload_data   # Return the json value report to the back end data 
  except:
        print("Current wifi not found you are not connected")
def point_cloud_pcd_generator(email,camera_name):
             print("Point cloud camera devices ",email,camera_name)  #Get the point cloud data to send into the server 
                               

   

