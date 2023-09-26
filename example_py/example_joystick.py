import sys
import time
import math


sys.path.append('../lib/python/amd64')
import robot_interface as sdk


TARGET_PORT = 8007
LOCAL_PORT = 8082
TARGET_IP = "192.168.123.10"   # target IP address

LOW_CMD_LENGTH = 610
LOW_STATE_LENGTH = 771

HIGHLEVEL = 0x00
LOWLEVEL  = 0xff

if __name__ == '__main__':
    udp = sdk.UDP(LOCAL_PORT, TARGET_IP, TARGET_PORT, LOW_CMD_LENGTH, LOW_STATE_LENGTH, -1)
    safe = sdk.Safety(sdk.LeggedType.Aliengo)

    cmd = sdk.LowCmd()
    state = sdk.LowState()
    
    udp.InitCmdData(cmd)
    cmd.levelFlag = LOWLEVEL
    
    while True:
        time.sleep(0.1)
        
        udp.Recv()
        udp.GetRecv(state)
        
        print(f"Remote controller: {state.wirelessRemote}")
        
        udp.SetSend(cmd)
        udp.Send()