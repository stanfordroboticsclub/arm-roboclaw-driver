
import roboclaw_driver


class RoboClaw:

    def __init__(self, port, addresses=None, names = None, auto_recover=False):

        if addresses == None:
            addresses = [128]
        if names == None:
            names =[1,2]

        assert(2*len(addresses) == len(names))

        self.port = port
        self.names = names
        self.auto_recover = auto_recover
        self.driver = roboclaw_driver

        self.address = {}
        self.motor_num = {}
        for i in range(len(addresses)):
            self.address[names[2*i  ]] = addresses[i]
            self.address[names[2*i+1]] = addresses[i]

            self.motor_num[names[2*i  ]] = 1
            self.motor_num[names[2*i+1]] = 2

        roboclaw_driver.Open(port, 115200)
        # How does this work>
        # self.serial_lock = Lock()
        # try:
        #     self.port.close()
        #     self.port.open()
        # except serial.serialutil.SerialException:
        #     if auto_recover:
        #         self.recover_serial()
        #     else:
        #         raise

    ##### GENERAL ######

    def read_version(self, motor):
        return roboclaw_driver.ReadVersion(self.address[motor])

    def read_error(self,motor):
        return roboclaw_driver.ReadError(self.address[motor])

    #tenths of a volte
    def read_main_battery_voltage(self, motor):
        return roboclaw_driver.ReadMainBatteryVoltage(self.address[motor])
        
    # doesn't work, gives 0
    def read_logic_battery_voltage(self, motor):
        return roboclaw_driver.ReadLogicBatteryVoltage(self.address[motor])

    #tenths of a celcious
    def read_temp(self,motor):
        return roboclaw_driver.ReadTemp(self.address[motor])

    ##### CURRENTS ######

    def read_current(self,motor):
        return roboclaw_driver.ReadCurrents(self.address[motor])[self.motor_num[motor]]

    def set_max_current(self,motor,current):
        if self.motor_num[motor] == 1:
            out = roboclaw_driver.SetM1MaxCurrent(self.address[motor],current)
        else:
            out = roboclaw_driver.SetM2MaxCurrent(self.address[motor],current)
        return out

    def read_max_current(self,motor):
        if self.motor_num[motor] == 1:
            out = roboclaw_driver.ReadM1MaxCurrent(self.address[motor])
        else:
            out = roboclaw_driver.ReadM2MaxCurrent(self.address[motor])
        return out

    ##### ENCODERS ######

    # The status byte tracks counter under ow, direction and over ow. The byte value represents:
# Bit0 - Counter Under ow (1= Under ow Occurred, Clear After Reading) Bit1 - Direction (0 = Forward, 1 = Backwards)
# Bit2 - Counter Over ow (1= Under ow Occurred, Clear After Reading) Bit3 - Reserved
# Bit4 - Reserved Bit5 - Reserved Bit6 - Reserved Bit7 - Reserved
# 1, val, status
    def read_encoder(self, motor):
        if self.motor_num[motor] == 1:
            out = roboclaw_driver.ReadEncM1(self.address[motor])
        else:
            out = roboclaw_driver.ReadEncM2(self.address[motor])
        return out

    #Receive: [Speed(4 bytes), Status, CRC(2 bytes)]
    # Status indicates the direction (0 - forward, 1 - backward)
    def read_encoder_speed(self, motor):
        if self.motor_num[motor] == 1:
            out = roboclaw_driver.ReadSpeedM1(self.address[motor])
        else:
            out = roboclaw_driver.ReadSpeedM2(self.address[motor])
        return out

    def set_encoder(self, motor, val):
        if self.motor_num[motor] == 1:
            out = roboclaw_driver.SetEncM1(self.address[motor], val)
        else:
            out = roboclaw_driver.SetEncM2(self.address[motor], val)
        return out

    ##### CONTROL ######

    def drive_position(self,motor, accel, speed, deccel, position, buffer):
        if self.motor_num[motor] == 1:
            out = roboclaw_driver.SpeedAccelDeccelPositionM1(self.address[motor], 
                                                             accel, speed, deccel, position, buffer)
        else:
            out = roboclaw_driver.SpeedAccelDeccelPositionM2(self.address[motor], 
                                                             accel, speed, deccel, position, buffer)
        return out

    # The duty value is signed and the range is -32768 to +32767 
    def drive_duty(self,motor,duty):
        if self.motor_num[motor] == 1:
            out = roboclaw_driver.DutyM1(self.address[motor], duty)
        else:
            out = roboclaw_driver.DutyM2(self.address[motor], duty)
        return out

    def drive_speed(self,motor,speed):
        if self.motor_num[motor] == 1:
            out = roboclaw_driver.SpeedM1(self.address[motor], speed)
        else:
            out = roboclaw_driver.SpeedM2(self.address[motor], speed)
        return out

