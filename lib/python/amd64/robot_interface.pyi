from typing import List, overload
from enum import Enum, IntEnum

class LeggedType(Enum):
    Aliengo: ...
    A1: ...
    Go1: ...
    B1: ...

class RecvEnum(IntEnum):
    nonBlock = 0x00
    block = 0x01
    blockTimeout = 0x02

class UDP:
    """
    UDP critical configuration:

    1. initiativeDisconnect: if need disconnection after connected, another ip/port can access after disconnection\n

                            /--- block             will block till data come\n
    2. recvType  ---- block + timeout   will block till data come or timeout\n
                            \\\--- non block         if no data will return immediately\n\n

                            /--- Y  ip/port will be set later\n
    3. setIpPort:\n
                            \\\--- N  ip/port not specified, as a server wait for connect\n
    """
    udpState: UDPState
    targetIP: str
    targetPort: int
    localIP: str
    localPort: int
    accessible: bool
    @overload
    def __init__(self, level: int, localPort: int, targetIP: str, targetPort: int) -> None: 
        """udp use dafault length according to level"""
        ...
        
    @overload
    def __init__(self, localPort: int, targetIP: str, targetPort: int, sendLength: int, recvLength: int, initiativeDisconnect: bool = ..., recvType: RecvEnum = ...) -> None: ...
    @overload
    def __init__(self, localPort: int, sendLength: int, recvLength: int, initiativeDisconnect: bool = ..., recvType: RecvEnum = ..., setIpPort: bool = ...) -> None: ...

    def SetIpPort(self, targetIP: str, targetPort: int) -> None: 
        """if not indicated at constructor function, use in RecvEnum::blockTimeout  (unit: ms)"""
        ...
    def SetRecvTimeout(self, time: int) -> None: 
        """if not indicated at constructor function, use in RecvEnum::blockTimeout  (unit: ms)"""
        ...
    def SetDisconnectTime(self, callback_dt: float, disconnectTime: float) -> None: 
        """initiativeDisconnect = true, disconnect for another IP to connect"""
        ...
    def SetAccessibleTime(self, callback_dt: float, accessibleTime: float) -> None: 
        """check if can access data"""
        ...

    def Send(self) -> int: ...
    def Recv(self) -> int: 
        """directly save in buffer"""
        ...
        
    @overload
    def InitCmdData(self, cmd: HighCmd) -> None: ...
    @overload
    def InitCmdData(self, cmd: LowCmd) -> None: ...
    @overload
    def SetSend(self, data: str) -> int: ...
    @overload
    def SetSend(self, cmd: HighCmd) -> int: ...
    @overload
    def SetSend(self, cmd: LowCmd) -> int: ...
    @overload
    def GetRecv(self, buffer: str) -> None: ...
    @overload
    def GetRecv(self, state: HighState) -> None: ...
    @overload
    def GetRecv(self, state: LowState) -> None: ...

 
    
class Safety:
    def __init__(self, type: LeggedType) -> None: ...
    def PositionLimit(self, cmd: LowCmd) -> None: 
        """only effect under Low Level control in Position mode"""
        ...
    def PowerProtect(self, cmd: LowCmd, state: LowState, factor: int) -> int: 
        """
        only effect under Low Level control, input factor: 1~10,
        means 10%~100% power limit. If you are new, then use 1; if you are familiar,
        then can try bigger number or even comment this function.
        """
        ...
    def PositionProtect(self, cmd: LowCmd, state: LowState, limit: float = 0.087) -> int: 
        """default limit is 5 degree"""
        ...

HIGHLEVEL: int = 0xee
LOWLEVEL: int = 0xff
TRIGERLEVEL: int = 0xf0
PosStopF: float = 2.146E+9
VelStopF: float = 16000.0
HIGH_CMD_LENGTH: int
HIGH_STATE_LENGTH: int
LOW_CMD_LENGTH: int
LOW_STATE_LENGTH: int

    
class BmsCmd:
    off: int  
    """set 0xA5 to turn off the battery, please try it under the premise of ensuring safety"""
    reserve: List[int]

class BmsState:
    version_h: int
    version_l: int
    bms_status: int  
    """0x00 : wakeup, 0X01 :  discharge, 0x02 : charge, 0x03 : charger, 0x04 : precharge, 0x05 : charge_err, 0x06 : waterfall_light, 0x07 : self_discharge, 0x08 : junk."""
    SOC: int  
    """SOC 0-100%"""
    current: int  
    """(unit: mA)"""
    cycle: int  
    """The current number of cycles of the battery"""
    BQ_NTC: List[int]  
    """2 elements, x1 degrees centigrade"""
    MCU_NTC: List[int]  
    """2 elements, x1 degrees centigrade"""
    cell_vol: List[int]  
    """10 elements, cell voltage mV"""

class Cartesian:
    x: float
    y: float
    z: float

class IMU:
    quaternion: List[float]  
    """4 elements, quaternion, normalized, (w,x,y,z)"""
    gyroscope: List[float]  
    """3 elements, angular velocity (unit: rad/s) (raw data)"""
    accelerometer: List[float]  
    """3 elements, acceleration (unit: m/(s2)) (raw data)"""
    rpy: List[float]  
    """3 elements, euler angle (unit: rad)"""
    temperature: int  
    """the temperature of imu (unit: °C)"""

class LED:
    r: int
    g: int
    b: int

class MotorState:
    mode: int  
    """motor working mode. Servo : 0x0A, Damping : 0x00, Overheat : 0x08."""
    q: float  
    """current angle (unit: radian)"""
    dq: float  
    """current velocity (unit: radian/second)"""
    ddq: float  
    """current acc (unit: radian/second*second)"""
    tauEst: float  
    """current estimated output torque (unit: N.m)"""
    q_raw: float  
    """reserve"""
    dq_raw: float  
    """reserve"""
    ddq_raw: float  
    """reserve"""
    temperature: int  
    """current temperature (temperature conduction is slow that leads to lag)"""
    reserve: List[int]  
    """2 elements"""
    

class MotorCmd:
    """motor control"""
    mode: int  
    """desired working mode. Servo : 0x0A, Damping : 0x00."""
    q: float  
    """desired angle (unit: radian)"""
    dq: float  
    """desired velocity (unit: radian/second)"""
    tau: float  
    """desired output torque (unit: N.m)"""
    Kp: float  
    """desired position stiffness (unit: N.m/rad )"""
    Kd: float  
    """desired velocity stiffness (unit: N.m/(rad/s) )"""
    reserve: List[int]  
    """3 elements"""
    
class LowState:
    """low level feedback"""
    head: List[int]  
    """2 elements, reserve"""
    levelFlag: int  
    """reserve"""
    frameReserve: int  
    """reserve"""
    SN: List[int]  
    """2 elements, reserve"""
    version: List[int]  
    """2 elements, reserve"""
    bandWidth: int  
    """reserve"""
    imu: IMU
    motorState: List[MotorState]  
    """20 elements"""
    bms: BmsState
    footForce: List[int]  
    """4 elements, Data from foot airbag sensor"""
    footForceEst: List[int]  
    """4 elements, reserve, typically zero"""
    tick: int  
    """reference real-time from motion controller (unit: ms)"""
    wirelessRemote: List[int]  
    """40 elements, Data from Unitree Joystick."""
    reserve: int
    crc: int
    
class LowCmd:
    """low level control"""
    head: List[int]  
    """2 elements, reserve"""
    levelFlag: int  
    """reserve"""
    frameReserve: int  
    """reserve"""
    SN: List[int]  
    """2 elements, reserve"""
    version: List[int]  
    """2 elements, reserve"""
    bandWidth: int
    motorCmd: List[MotorCmd]  
    """20 elements"""
    bms: BmsCmd
    wirelessRemote: List[int]  
    """40 elements, reserve"""
    reserve: int
    crc: int
    
class HighState:
    """high level feedback"""
    head: List[int]  
    """2 elements, reserve"""
    levelFlag: int  
    """reserve"""
    frameReserve: int  
    """reserve"""
    SN: List[int]  
    """2 elements, reserve"""
    version: List[int]  
    """2 elements, reserve"""
    bandWidth: int
    imu: IMU
    motorState: List[MotorState]  
    """20 elements"""
    bms: BmsState
    footForce: List[int]  
    """4 elements, Data from foot airbag sensor"""
    footForceEst: List[int]  
    """4 elements, reserve, typically zero"""
    mode: int  
    """The current mode of the robot"""
    progress: float  
    """reserve"""
    gaitType: int  
    """0.idle 1.trot 2.trot running 3.climb stair 4.trot obstacle"""
    footRaiseHeight: float  
    """(unit: m, default: 0.08m), foot up height while walking"""
    position: List[float]  
    """3 elements, (unit: m), from own odometry in inertial frame, usually drift"""
    bodyHeight: float  
    """(unit: m, default: 0.28m)"""
    velocity: List[float]  
    """3 elements, (unit: m/s), forwardSpeed, sideSpeed, rotateSpeed in body frame"""
    yawSpeed: float  
    """(unit: rad/s), rotateSpeed in body frame"""
    rangeObstacle: List[float]  
    """4 elements, Distance to nearest obstacle"""
    footPosition2Body: List[Cartesian]  
    """4 elements, foot position relative to body"""
    footSpeed2Body: List[Cartesian]  
    """4 elements, foot speed relative to body"""
    wirelessRemote: List[int]  
    """40 elements, Data from Unitree Joystick."""
    reserve: int
    crc: int

class HighCmd:
    """high level control"""
    head: List[int]  
    """2 elements, reserve, no need to set."""
    levelFlag: int  
    """reserve. No need to set, only need to set UDP class."""
    frameReserve: int  
    """reserve"""
    SN: List[int]  
    """2 elements, reserve"""
    version: List[int]  
    """2 elements, reserve"""
    bandWidth: int  
    """reserve"""
    mode: int
    """
    0. idle, default stand\n
    1. force stand (controlled by dBodyHeight + ypr)\n
    2. target velocity walking (controlled by velocity + yawSpeed)\n
    3. target position walking (controlled by position + ypr[0]), reserve\n
    4. path mode walking (reserve for future release), reserve\n
    5. position stand down.\n
    6. position stand up\n
    7. damping mode\n
    8. recovery stand\n
    9. backflip, reserve\n
    10. jumpYaw, only left direction. Note, to use this mode, you need to set mode = 1 first.\n
    11. straightHand. Note, to use this mode, you need to set mode = 1 first.\n
    """
    gaitType: int
    """
    0.idle\n
    1.trot\n
    2.trot running\n
    3.climb stair\n
    4.trot obstacle\n
    """
    speedLevel: int  
    """reserve"""
    footRaiseHeight: float  
    """(unit: m, range: -0.06~0.03m, default: 0.09m), foot up height while walking, delta value"""
    bodyHeight: float  
    """(unit: m, range: -0.13~0.03m, default: 0.31m), delta value"""
    position: List[float]  
    """2 elements, (unit: m), desired position in inertial frame, reserve"""
    euler: List[float] 
    """
    3 elements,
    (unit: rad), roll pitch yaw in stand mode\n
    (range: roll : -0.75~0.75rad)\n
    (range: pitch: -0.75~0.75rad)\n
    (range: yaw  : -0.6~0.6rad)\n
    """
    velocity: List[float]
    """
    2 elements,
    (unit: m/s), forwardSpeed, sideSpeed in body frame\n
    (range: trot : vx:-1.1~1.5m/s,  vy:-1.0~1.0m/s)\n
    (range: run  : vx:-2.5~3.5m/s,  vy:-1.0~1.0m/s)\n
    (range: stair: vx:-0.2~0.25m/s, vy:-0.15~0.15m/s)\n
    """
    yawSpeed: float  
    """
    (unit: rad/s), rotateSpeed in body frame\n
    (range: trot : -4.0~4.0rad/s)\n
    (range: run  : -4.0~4.0rad/s)\n
    (range: stair: -0.7~0.7rad/s)\n
    """
    bms: BmsCmd
    led: List[LED]  
    """4 elements, reserve"""
    wirelessRemote: List[int]  
    """40 elements, reserve"""
    reserve: int
    crc: int
    
    
class UDPState:
    """UDP communication state"""
    TotalCount: int  
    """total loop count"""
    SendCount: int  
    """total send count"""
    RecvCount: int  
    """total receive count"""
    SendError: int  
    """total send error"""
    FlagError: int  
    """total flag error"""
    RecvCRCError: int  
    """total reveive CRC error"""
    RecvLoseError: int  
    """total lose package count"""