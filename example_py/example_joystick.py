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
        
        key_data = sdk.parse_joystick_data(state.wirelessRemote)
        print(f"R1: {key_data.btn.components.R1}, \
L1: {key_data.btn.components.L1}, \
start: {key_data.btn.components.start}, \
select: {key_data.btn.components.select}, \
R2: {key_data.btn.components.R2}, \
L2: {key_data.btn.components.L2}, \
F1: {key_data.btn.components.F1}, \
F2: {key_data.btn.components.F2}, \
A: {key_data.btn.components.A}, \
B: {key_data.btn.components.B}, \
X: {key_data.btn.components.X}, \
Y: {key_data.btn.components.Y}, \
up: {key_data.btn.components.up}, \
right: {key_data.btn.components.right}, \
down: {key_data.btn.components.down}, \
left: {key_data.btn.components.left}, \
lx: {key_data.lx:.2f}, \
ly: {key_data.ly:.2f}, \
rx: {key_data.rx:.2f}, \
ry: {key_data.ry:.2f}, \
")
        
        udp.SetSend(cmd)
        udp.Send()