import serial


class Serial_arduino:
    def __init__(self, address = '/dev/ttyACM0', bundrate = 115200):

        
        self.com = serial.Serial(address,bundrate, timeout = None)
        self.com.flushInput()

    def get_button(self):
        self.com.flushInput()
        self.com.write(b'b')
        num = int(self.com.readline().strip())
        return num

    def init_imu(self):
        self.com.flushInput()
        self.com.write(b'i')
        num = self.com.read(0)

    def euler(self):
        self.com.flushInput()
        self.com.write(b'r')
        x = float(self.com.readline().strip())
        y = float(self.com.readline().strip())
        z = float(self.com.readline().strip())
        return x, y, z

if __name__ == "__main__":
    com = Serial_arduino()
    print(com.get_button())
    com.init_imu()
    for i in range(10):
        print(com.euler())
    com.init_imu()
    for i in range(10):
        print(com.euler())
     
    