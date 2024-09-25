import time
from Tufts_ble import Sniff, Yell

def zombie():    
    p = Yell()
    while True:
        p.advertise(f'!{4}') # advertising our team number
        print(".")
        time.sleep(0.1)
        p.stop_advertising()

while True:
    zombie()
    
