import serial
import time 
import numpy


class Serial_arduino:
    def __init__(self, address = '/dev/ttyACM0', bundrate = 115200):

        
        self.com = serial.Serial(address,bundrate, timeout = None)
        self.com.flushInput()
        self.x = 0
        self.y = 0
        self.z = 0

    def get_button(self):
        self.com.flushInput()
        self.com.write([1])
        number = int(self.com.readline().strip())
        return number

    def init_imu(self):
        self.com.flushInput()
        self.com.write([2])
        num = self.com.read(0)
        

    def euler(self):
        self.com.write([3])
        #self.com.read(self.com.in_waiting)
        #self.x = self.com.in_waiting
        #x = self.com.read(2)
        
        #self.x = int(self.com.read(2))
        #self.x = int.from_bytes(x, "little")
        
        #x = self.com.read(4)
        #print(x)
        #self.x = int.from_bytes(self.com.read(4), "little", signed=True)
        #self.y = int.from_bytes(self.com.read(4), "big", signed=True)
        #self.z = int.from_bytes(self.com.read(4), "big", signed=True)

        self.x = float(self.com.readline().strip())
        self.y = float(self.com.readline().strip())
        self.z = float(self.com.readline().strip())
        
        #self.x = float(self.com.read(4))
        #self.y = float(self.com.read(4))
        #self.z = float(self.com.read(4))
        
        return self.x, self.y, self.z


#if __name__ == "__main__":
    #com = Serial_arduino()
    #print(com.get_button())
    
    #com.com.flushInput()

    #com.init_imu()
    #for i in range(100):
        #print(com.euler())