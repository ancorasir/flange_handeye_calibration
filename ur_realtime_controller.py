#!/usr/bin/python3
import socket
import time
import struct
import math
import numpy as np

robot_ip = "192.168.1.10" # The remote robot_ip
port = 30003

print("Starting Program")

def get_pos():
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(10)
        s.connect((robot_ip, port))
        s.send("get_state()"+"\n")
        # time.sleep(1)
        packet_1 = s.recv(444)
        packet_12 = s.recv(8)
        packet_12 = packet_12.encode("hex") #convert the data from \x hex notation to plain hex
        x = str(packet_12)
        x = struct.unpack('!d', packet_12.decode('hex'))[0]
        # print("X = ", x * 1000)
        packet_13 = s.recv(8)
        packet_13 = packet_13.encode("hex") #convert the data from \x hex notation to plain hex
        y = str(packet_13)
        y = struct.unpack('!d', packet_13.decode('hex'))[0]
        # print("Y = ", y * 1000)
        packet_14 = s.recv(8)
        packet_14 = packet_14.encode("hex") #convert the data from \x hex notation to plain hex
        z = str(packet_14)
        z = struct.unpack('!d', packet_14.decode('hex'))[0]
        # print("Z = ", z * 1000)
        packet_15 = s.recv(8)
        packet_15 = packet_15.encode("hex") #convert the data from \x hex notation to plain hex
        Rx = str(packet_15)
        Rx = struct.unpack('!d', packet_15.decode('hex'))[0]
        # print "Rx = ", Rx
        packet_16 = s.recv(8)
        packet_16 = packet_16.encode("hex") #convert the data from \x hex notation to plain hex
        Ry = str(packet_16)
        Ry = struct.unpack('!d', packet_16.decode('hex'))[0]
        # print "Ry = ", Ry
        packet_17 = s.recv(8)
        packet_17 = packet_17.encode("hex") #convert the data from \x hex notation to plain hex
        Rz = str(packet_17)
        Rz = struct.unpack('!d', packet_17.decode('hex'))[0]
        # print "Rz = ", Rz
        beta = (1-2*3.14/math.sqrt(Rx*Rx+Ry*Ry+Rz*Rz))
        Rx = -Rx * beta
        Ry = -Ry * beta
        Rz = -Rz * beta
        # print("Rx = ", Rx)
        # print("Ry = ", Ry)
        # print("Rz = ", Rz)
        return np.array([x * 1000, y * 1000, z * 1000, Rx, Ry, Rz])
        s.close()
    except socket.error as socketerror:
        print("Error: ", socketerror)

def movej(p, a=0.5, v=0.5):
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.settimeout(10)
    s.connect((robot_ip, port))
    time.sleep(0.05)
    # s.send ("set_digital_out(2,True)" + "\n")  # Set digital output 2 high True
    if len(p)==6:
        s.send ("movej(p[ %f, %f, %f, %f, %f, %f], a = %f, v = %f)\n" %(p[0]/1000.0,p[1]/1000.0,p[2]/1000.0,p[3],p[4],p[5],a,v))
        time.sleep(2)
    if len(p)==7:
        s.send ("movej([%f, %f, %f, %f, %f, %f], a = %f, v = %f)\n" %(p[0],p[1],p[2],p[3],p[4],p[5],a,v))
        time.sleep(2)

    s.close()

def rg6_close(switch):
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.settimeout(10)
    s.connect((robot_ip, port))
    s.send(str.encode('set_digital_out(8,%s)\n'%switch))
    s.close()

def verifyPostion(targetPosition):
    delay_time = True
    cnt = 0
    timeGap = 0.1
    while(delay_time and cnt < 100):
        currentPose = get_pos()
        dpose = np.zeros(6)
        inv_dpose = np.zeros(6)
        dpose[0] = abs(currentPose[0]-targetPosition[0])
        dpose[1] = abs(currentPose[1]-targetPosition[1])
        dpose[2] = abs(currentPose[2]-targetPosition[2])
        dpose[3] = abs(currentPose[3]-targetPosition[3])
        dpose[4] = abs(currentPose[4]-targetPosition[4])
        dpose[5] = abs(currentPose[5]-targetPosition[5])

        inv_dpose[0] = abs(currentPose[0]-targetPosition[0])
        inv_dpose[1] = abs(currentPose[1]-targetPosition[1])
        inv_dpose[2] = abs(currentPose[2]-targetPosition[2])
        inv_dpose[3] = abs(-currentPose[3]-targetPosition[3])
        inv_dpose[4] = abs(-currentPose[4]-targetPosition[4])
        inv_dpose[5] = abs(-currentPose[5]-targetPosition[5])

        # print("maxPose: ",max(dpose))
        # print("currentPose", currentPose)
        # print("targetPose", targetPosition)
        # print("========================================================")
        if (max(dpose) < 0.2 or max(inv_dpose) < 0.2):
            delay_time = False
            return True
        else:
            time.sleep(timeGap)
            cnt = cnt + 1
        if(cnt*timeGap >= 20):
            print("Time Out!")
            return False


if __name__=="__main__":
    get_pos()
    raw_input('Press enter to continue: ')
    movej(446.0, 286.0, 329.0, 3.07, -0.38, -0.03, 0.1, 0.1)
    get_pos()
    print("Program finish")
