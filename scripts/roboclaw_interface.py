
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


    def ReadVersion(self, motor):
        return roboclaw_driver.ReadVersion(self.address[motor])

    def ReadEncoder(self, motor):
        if self.motor_num[motor] == 1:
            out = roboclaw_driver.ReadEncM1(self.address[motor])
        else:
            out = roboclaw_driver.ReadEncM2(self.address[motor])
        return out

    def ReadEncoderSpeed(self, motor):
        if self.motor_num[motor] == 1:
            out = roboclaw_driver.ReadSpeedM1(self.address[motor])
        else:
            out = roboclaw_driver.ReadSpeedM2(self.address[motor])
        return out


    def SetEncoder(self, motor, val):
        if self.motor_num[motor] == 1:
            out = roboclaw_driver.SetEncM1(self.address[motor], val)
        else:
            out = roboclaw_driver.SetEncM2(self.address[motor], val)
        return out
