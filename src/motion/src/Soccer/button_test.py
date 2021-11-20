# Untitled - By: a - Вс июн 21 2020

import sensor
import pyb
from pyb import Pin, ExtInt, LED

class Button_Test:
    def __init__(self):
        from pyb import Pin, ExtInt, LED
        self.red_led = LED(1)
        self.green_led = LED(2)
        self.blue_led = LED(3)
        PinMapperDict = { 'TopRight' : Pin.board.P2, 'BottomRight': Pin.board.P3,
                      'BottomLeft': Pin.board.P9 }
        Pin.dict(PinMapperDict)

        self.TopRightPin = Pin('TopRight', Pin.IN, Pin.PULL_UP)
        self.BottomRightPin = Pin('BottomRight', Pin.IN, Pin.PULL_UP)
        self.BottomLeftPin = Pin('BottomLeft', Pin.IN, Pin.PULL_UP)

        self.TopRightButtonON = False
        self.BottomRightButtonON = False
        self.BottomLeftButtonON = False

        self.extint1 = ExtInt(self.TopRightPin, ExtInt.IRQ_RISING_FALLING, Pin.PULL_UP, self.callback00)
        self.extint2 = ExtInt(self.BottomRightPin, ExtInt.IRQ_RISING_FALLING, Pin.PULL_UP, self.callback01)
        self.extint3 = ExtInt(self.BottomLeftPin, ExtInt.IRQ_RISING_FALLING, Pin.PULL_UP, self.callback02)

    def callback00(self, line):
        if self.TopRightPin.value() == 0:
            self.TopRightButtonON = True
        else: self.TopRightButtonON = False

    def callback01(self, line):
        if self.BottomRightPin.value() == 0:
            self.BottomRightButtonON = True
        else: self.BottomRightButtonON = False

    def callback02(self, line):
        if self.BottomLeftPin.value() == 0:
            self.BottomLeftButtonON = True
        else: self.BottomLeftButtonON = False

    def white_led_blink(self):
        pyb.delay(50)
        self.red_led.toggle()
        self.green_led.toggle()
        self.blue_led.toggle()

    def white_led_blink_long(self):
        pyb.delay(200)
        self.red_led.toggle()
        self.green_led.toggle()
        self.blue_led.toggle()

    def white_led_off(self):
        self.red_led.off()
        self.green_led.off()
        self.blue_led.off()

    def wait_for_button_pressing(self):
        returnCode = 0

        TR_short_pressed = False
        TR_long_pressed = False
        BR_short_pressed = False
        BR_long_pressed = False
        BL_short_pressed = False
        BL_long_pressed = False

        TR_pressed = False
        BR_pressed = False
        BL_pressed = False

        buttons_pressed = 0

        TopRightTimerStart = pyb.millis()
        BottomRightTimerStart = pyb.millis()
        BottomLeftTimerStart = pyb.millis()

        while True:
            if TR_pressed == False and BR_pressed == False and BL_pressed == False:
                self.white_led_blink_long()                          # ready
            if self.TopRightButtonON == True and TR_pressed == False:
                TopRightTimerStart = pyb.millis()
                TR_pressed = True
                buttons_pressed += 1
            TR_time_elapsed = pyb.elapsed_millis(TopRightTimerStart)
            if TR_time_elapsed > 1000 and TR_pressed == True:
                self.white_led_blink()
            if TR_time_elapsed > 50:        # button bounce cancellation
                if self.TopRightButtonON == False and TR_pressed == True:
                    if TR_time_elapsed > 1000: TR_long_pressed = True
                    else: TR_short_pressed = True
                    self.white_led_off()
                    break
            if self.BottomRightButtonON == True and BR_pressed == False:
                BottomRightTimerStart = pyb.millis()
                BR_pressed = True
                buttons_pressed += 1
            BR_time_elapsed = pyb.elapsed_millis(BottomRightTimerStart)
            if BR_time_elapsed > 1000 and BR_pressed == True:
                self.white_led_blink()
            if BR_time_elapsed > 50:        # button bounce cancellation
                if self.BottomRightButtonON == False and BR_pressed == True:
                    if BR_time_elapsed > 1000: BR_long_pressed = True
                    else: BR_short_pressed = True
                    self.white_led_off()
                    break
            if self.BottomLeftButtonON == True and BL_pressed == False:
                BottomLeftTimerStart = pyb.millis()
                BL_pressed = True
                buttons_pressed += 1
            BL_time_elapsed = pyb.elapsed_millis(BottomLeftTimerStart)
            if BL_time_elapsed > 1000 and BL_pressed == True:
                self.white_led_blink()
            if BL_time_elapsed > 50:        # button bounce cancellation
                if self.BottomLeftButtonON == False and BL_pressed == True:
                    if BL_time_elapsed > 1000: BL_long_pressed = True
                    else: BL_short_pressed = True
                    self.white_led_off()
                    break

        if buttons_pressed > 1:
            if TR_pressed and BR_pressed: returnCode = 7
            if TR_pressed and BL_pressed: returnCode = 8
            if BL_pressed and BR_pressed: returnCode = 9
        else:
            if TR_short_pressed: returnCode = 1
            if BR_short_pressed: returnCode = 2
            if BL_short_pressed: returnCode = 3
            if TR_long_pressed: returnCode = 4
            if BR_long_pressed: returnCode = 5
            if BL_long_pressed: returnCode = 6

        return returnCode

if __name__ == "__main__":
    b = Button_Test()
    returnCode = b.wait_for_button_pressing()
    print(returnCode)




