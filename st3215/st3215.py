import serial
import threading
import time

from .port_handler import *
from .protocol_packet_handler import *
from .group_sync_write import *
from .group_sync_read import *
from .values import *


__all__ = ['ST3215']


class ST3215(protocol_packet_handler):

    def __init__(self, device):

        self.portHandler = PortHandler(device)
        
        if not self.portHandler.openPort():
            raise ValueError(f"Could not open port: {device}")

        protocol_packet_handler.__init__(self, self.portHandler)

        self.groupSyncWrite = GroupSyncWrite(self, STS_ACC, 7)
        self.lock = threading.Lock()


    def ServoPing(self, sts_id):
        model, comm, error = self.ping(sts_id)
        if comm != COMM_SUCCESS or model == 0 or error != 0:
            return False
        return True

    def ListServos(self):
        servos=[]
        for id in range(0, 254):
            if self.ServoPing(id):
                servos.append(id)

        return servos


    def ReadLoad(self, sts_id):
        return self.read1ByteTxRx(sts_id, STS_PRESENT_LOAD_L)

    def ReadVoltage(self, sts_id):
        return self.read1ByteTxRx(sts_id, STS_PRESENT_VOLTAGE)

    def ReadTemperature(self, sts_id):
        return self.read1ByteTxRx(sts_id, STS_PRESENT_TEMPERATURE)




    def WritePosEx(self, sts_id, position, speed, acc):
        txpacket = [acc, self.sts_lobyte(position), self.sts_hibyte(position), 0, 0, self.sts_lobyte(speed), self.sts_hibyte(speed)]
        return self.writeTxRx(sts_id, STS_ACC, len(txpacket), txpacket)

    def ReadPos(self, sts_id):
        sts_present_position, sts_comm_result, sts_error = self.read2ByteTxRx(sts_id, STS_PRESENT_POSITION_L)
        return self.sts_tohost(sts_present_position, 15), sts_comm_result, sts_error

    def ReadSpeed(self, sts_id):
        sts_present_speed, sts_comm_result, sts_error = self.read2ByteTxRx(sts_id, STS_PRESENT_SPEED_L)
        return self.sts_tohost(sts_present_speed, 15), sts_comm_result, sts_error

    def ReadPosSpeed(self, sts_id):
        sts_present_position_speed, sts_comm_result, sts_error = self.read4ByteTxRx(sts_id, STS_PRESENT_POSITION_L)
        sts_present_position = self.sts_loword(sts_present_position_speed)
        sts_present_speed = self.sts_hiword(sts_present_position_speed)
        return self.sts_tohost(sts_present_position, 15), self.sts_tohost(sts_present_speed, 15), sts_comm_result, sts_error

    def ReadMoving(self, sts_id):
        return self.read1ByteTxRx(sts_id, STS_MOVING)

    def SyncWritePosEx(self, sts_id, position, speed, acc):
        txpacket = [acc, self.sts_lobyte(position), self.sts_hibyte(position), 0, 0, self.sts_lobyte(speed), self.sts_hibyte(speed)]
        return self.groupSyncWrite.addParam(sts_id, txpacket)

    def RegWritePosEx(self, sts_id, position, speed, acc):
        txpacket = [acc, self.sts_lobyte(position), self.sts_hibyte(position), 0, 0, self.sts_lobyte(speed), self.sts_hibyte(speed)]
        return self.regWriteTxRx(sts_id, STS_ACC, len(txpacket), txpacket)

    def RegAction(self):
        return self.action(BROADCAST_ID)

    def WheelMode(self, sts_id):
        return self.write1ByteTxRx(sts_id, STS_MODE, 1)

    def WriteSpec(self, sts_id, speed, acc):
        speed = self.sts_toscs(speed, 15)
        txpacket = [acc, 0, 0, 0, 0, self.sts_lobyte(speed), self.sts_hibyte(speed)]
        return self.writeTxRx(sts_id, STS_ACC, len(txpacket), txpacket)

    def LockEprom(self, sts_id):
        return self.write1ByteTxOnly(sts_id, STS_LOCK, 1)

    def UnLockEprom(self, sts_id):
        return self.write1ByteTxOnly(sts_id, STS_LOCK, 0)

    def ChangeId(self, sts_id, new_id):
        if isinstance(new_id, int) and 0 <= new_id <= 253:
            if not self.ServoPing(sts_id):
                return None, f"Could not find servo: {sts_id}" 

            if self.UnLockEprom(sts_id) != COMM_SUCCESS:
                return None, "Could not unlock Eprom" 

            if self.write1ByteTxOnly(sts_id, STS_ID, new_id) != COMM_SUCCESS:
                return None, "Could not change Servo ID" 

            self.LockEprom(sts_id)
        else:
            return None, "new_id is not between 0 and 254" 



