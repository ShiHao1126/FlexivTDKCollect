"""Software test doubles only; these modules never import a hardware SDK."""
from types import SimpleNamespace as NS
import numpy as np


class Robot:
    def __init__(self, sn, value, log):
        self.sn, self.value, self.log = sn, value, log
        self.valid = True
        self.bad = False
        self.enabled = False
        self.tick = 100
        self.freeze = False
        self.q = np.zeros(7)
        self.control_mode = None
        self.follow = True

    def info(self):
        return NS(DoF=7, serial_num=self.sn, model_name="FAKE", software_ver="test",
                  has_FT_sensor=True, K_x_nom=[1000]*6, q_min=np.full(7, -3.2),
                  q_max=np.full(7, 3.2), dq_max=np.ones(7))

    def connected(self): return self.valid
    def fault(self): return self.bad
    def recovery(self): return False
    def estop_released(self): return True
    def operational(self): return self.enabled
    def mode(self): return self.control_mode
    def Enable(self): self.enabled = True
    def Stop(self): self.log.append((self.sn, "robot_stop"))
    def SwitchMode(self, mode): self.control_mode = mode

    def SendJointPosition(self, q, dq, vel, acc):
        self.log.append((self.sn, "move", q, vel))
        if self.follow:
            self.q = np.array(q)

    def states(self):
        if not self.valid:
            raise RuntimeError("TEST: access after TDK Stop")
        if not self.freeze:
            self.tick += 1
        return NS(timestamp=[1, self.tick * 1000000], tcp_pose=[0, 0, 0, 1, 0, 0, 0],
                  tcp_vel=np.zeros(6), q=self.q, dq=np.zeros(7),
                  ext_wrench_in_tcp=np.arange(6) + self.value,
                  ext_wrench_in_tcp_raw=np.arange(6) + self.value + 10,
                  ft_sensor_raw=np.arange(6) + self.value + 20)


class Teleop:
    def __init__(self, sdk, pairs):
        self.sdk = sdk
        self.robots = (Robot(pairs[0][0], 1, sdk.log), Robot(pairs[0][1], 101, sdk.log))
        self.log = sdk.log

    def instances(self, index): return self.robots
    def Init(self, limit, zero):
        self.log.append(("init", zero))
        for robot in self.robots:
            robot.Enable()
        if self.sdk.fail_init:
            raise RuntimeError("injected Init failure")
    def Start(self): self.log.append(("start",))
    def Stop(self):
        self.log.append(("tdk_stop",))
        for robot in self.robots:
            robot.valid = False
    def Engage(self, index, active): self.log.append(("engage", active))
    def any_fault(self): return False
    def stopped(self, index): return False
    def SetAxisLockCmd(self, index, value): self.log.append(("lock", value))
    def SetWrenchFeedbackScalingFactor(self, index, value): self.log.append(("feedback", value))
    def SetFollowerCartStiff(self, index, value): self.log.append(("stiffness", value))
    def SetFollowerMaxContactWrench(self, index, value): self.log.append(("wrench_limit", value))
    def SetLeaderNullSpacePosture(self, index, value): pass
    def SetFollowerNullSpacePosture(self, index, value): pass


class SDK:
    ZeroFTSensor = NS(Enable="zero", Disable="keep")
    CoordType = NS(WORLD="world", TCP="tcp")
    AxisLock = NS

    def __init__(self):
        self.log = []
        self.connections = []
        self.fail_init = False

    def TransparentCartesianTeleopLAN(self, pairs, interfaces):
        teleop = Teleop(self, pairs)
        self.connections.append(teleop)
        return teleop


RDK = NS(Mode=NS(NRT_JOINT_POSITION="joint", NRT_CARTESIAN_MOTION_FORCE="cartesian"))
