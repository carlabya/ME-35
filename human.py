import asyncio
import neopixel
from machine import Pin, PWM



class Human:
    def __init__(self, infected = 0, team = 0): 

        self.neo = neopixel.NeoPixel(Pin(28),1)
        self.buzzer = PWM(Pin('GPIO18', Pin.OUT))
        self.buzzer.freq(500)
        self.current_time = [0,0,0,0,0,0,0,0,0,0,0,0,0]
        self.infected = infected
        self.team = team
        self.tagged = [0,0,0,0,0,0,0,0,0,0,0,0,0]
        self.current_time = [0,0,0,0,0,0,0,0,0,0,0,0,0]
        self.tag = [0,0,0,0,0,0,0,0,0,0,0,0,0]

    def status(self):

        for i, count in enumerate(self.tagged):
            if count >= 3:
                self.infected = 1
                self.team = i + 1
                break

    async def visual(self):
        while True:
            for i in range(255):
                self.neo[0] = (i, 0, 0) if self.infected else (0, i, 0)
                self.neo.write()
                await asyncio.sleep(0.01)

            for i in range(255, 0, -1):
                self.neo[0] = (i, 0, 0) if self.infected else (0, i, 0)
                self.neo.write()
                await asyncio.sleep(0.01)

    async def buzz(self):
        while True:
            while self.infected:
                self.buzzer.freq(500)
                self.buzzer.duty_u16(10000)
                await asyncio.sleep(0)
            await asyncio.sleep(0)
