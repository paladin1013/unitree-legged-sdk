#!/usr/bin/python

import sys
import time
import math

sys.path.append('../lib/python/amd64')
import robot_interface as sdk

# high cmd
TARGET_PORT = 8082
LOCAL_PORT = 8081
TARGET_IP = "192.168.123.220"   # target IP address

HIGH_CMD_LENGTH = 113
HIGH_STATE_LENGTH = 244


if __name__ == '__main__':

    HIGHLEVEL = 0x00
    LOWLEVEL  = 0xff
    
    # udp = sdk.UDP(8080, "192.168.123.161", 8082, 129, 1087, False, sdk.RecvEnum.nonBlock)
    # udp = sdk.UDP(HIGHLEVEL, 8080, "192.168.123.161", 8082)

    cmd = sdk.HighCmd()
    state = sdk.HighState()

    

    
    udp = sdk.UDP(LOCAL_PORT, TARGET_IP, TARGET_PORT, HIGH_CMD_LENGTH, HIGH_STATE_LENGTH, -1)

    udp.InitCmdData(cmd)

    motiontime = 0
   
    cmd.velocity[0] = 0.0
    cmd.velocity[1] = 0.0
    cmd.position[0] = 0.0
    cmd.position[1] = 0.0
    cmd.yawSpeed = 0.0

    cmd.mode = 0
    cmd.rpy[0]  = 0
    cmd.rpy[1] = 0
    cmd.rpy[2] = 0
    cmd.gaitType = 0
    cmd.dBodyHeight = 0
    cmd.dFootRaiseHeight = 0

    
    motiontime = 0
    while 1:
        motiontime += 2
        if motiontime == 2:
            print("begin sending commands.")
            
        elif motiontime > 10 and motiontime < 100:
            cmd.levelFlag = 0xf0
            
        elif motiontime == 100:
            print("Aliengo sport mode trigger sent !")
            
        elif motiontime > 100 and motiontime % 100 == 0:
            udp.Recv()
            udp.GetRecv(state)
            print(f"rpy: {state.imu.rpy} yawSpeed: {state.yawSpeed}")
            time.sleep(0.1)
            
        udp.SetSend(cmd)
        udp.Send()
        time.sleep(0.002)
        

