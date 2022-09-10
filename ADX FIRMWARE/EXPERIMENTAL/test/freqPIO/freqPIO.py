#from machine import Pin
#from time import sleep_ms, ticks_us
from rp2 import asm_pio, StateMachine, PIO
#from ST7789 import TFT114
import sys 

t1 = 0
t2 = 0
t_diff = 0


@asm_pio()
def IO_ops():
    irq(clear, 0)              # clear irq 0
    wait(1, pin, 0)            # wait for high logic on pin index 0, i.e sense pin
    irq(rel(0))                # raise IRQ 0
    wait(0, pin, 0)            # wait for logic low on pin index 0, i.e sense pin   
    
    
def irq_0_handler(sm):
    global t1, t2, t_diff
    t2 = ticks_us()
    t_diff = (t2 - t1)
    t1 = t2
    
    
sense_pin = Pin(0, Pin.IN)
    
sm = StateMachine(0, IO_ops, in_base = sense_pin)
sm.irq(irq_0_handler)
sm.active(1)



while(True):
    period = t_diff
    
    if(period > 999):
        f = 1000000 / (period + 1)
        sys.stdout.write("freq %9.2f\n\r" % (f))
    else:
        f = 1000 / (period + 1)
        sys.stdout.write("freq %9.2f\n\r" % (f))
    sleep_ms(600)
    

