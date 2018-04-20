
import roboclaw_driver


class RoboClaw:

    def __init__(self, port, addresses=None, names = None, auto_recover=False):

        if address == None:
            address = [128]
        if names == None:
            names =[1,2]

        assert(2*len(address) == len(names))

        self.port = port
        self.names = names
        self.auto_recover = auto_recover

        self.address = {}
        self.motor_num = {}
        for i in range(len(address)):
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


    def read_version(self, motor):
        return roboclaw_driver.ReadVersion(self.address[motor])

    def read_error(self,motor)
        return roboclaw_driver.ReadError(self.address[motor])

    def read_main_battery_voltage(self, motor):
        return roboclaw_driver.ReadMainBatteryVoltage(self.address[motor]):
        
    def read_logic_battery_voltage(self, motor):
        return roboclaw_driver.ReadLogicBatteryVoltage(self.address[motor]):

    def read_temp(self,motor):
        return roboclaw_driver.ReadTemp(self.address[motor])


    def read_current(self,motor):
        return roboclaw_driver.ReadCurrents(self.address[motor])[self.motor_num[motor]]

    def set_max_current(self,motor,current):
        if self.motor_num[motor] == 1:
            out = roboclaw_driver.SetM1MaxCurrent(self.address[motor],current)
        else:
            out = roboclaw_driver.SetM2MaxCurrent(self.address[motor],current)
        return out


    def read_encoder(self, motor):
        if self.motor_num[motor] == 1:
            out = roboclaw_driver.ReadEncM1(self.address[motor])
        else:
            out = roboclaw_driver.ReadEncM2(self.address[motor])
        return out

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

