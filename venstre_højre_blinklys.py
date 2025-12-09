from machine import Pin
import time

venstre_LED = Pin(26, Pin.OUT) #husk at ændre vores Pin 
venstre_knap = Pin(4, Pin.IN)

højre_LED = Pin(12, Pin.OUT) #husk at ændre vores Pin
højre_knap = Pin(0, Pin.IN)

#vi har defineret en start_tid for venstre knap
start_time_left = time.ticks_ms()

#vi har defineret en start_tid for højre knap
start_time_right = time.ticks_ms()

interval = 300 #ms - burde være hvor hurtigt vores LED blinker
blink_tid = 10000 #hvor lang tid den skal blinke (10sec)


# stop_blink = False

def blink_left():
    start_time_left = time.ticks_ms()
    while time.ticks_ms() - start_time_left <= blink_tid: #blink_tid er defineret som 10,000 mili seconds
        if højre_knap.value() ==0:
             break
        venstre_LED.value(not venstre_LED.value())
        time.sleep_ms(interval)
    venstre_LED.value(0) # LED slukket
        
def blink_right():
    start_time_right = time.ticks_ms()
    while time.ticks_ms() - start_time_right <= blink_tid:
        if venstre_knap.value() ==0: #hvis vi afbryder knappen
            break
        højre_LED.value(not højre_LED.value())
        time.sleep_ms(interval)
    højre_LED.value(0)
    

def venstre_stop_irq(pin):
    global stop_blink
    stop_blink = True
    
def højre_stop_irq(pin):
    global stop_blink
    stop_blink = True

# interrupt for venstre_knap

# interrrupt():
venstre_knap.irq(trigger = Pin.IRQ_FALLING, handler = venstre_stop_irq) 
# #interrupt for højre_knap
højre_knap.irq(trigger = Pin.IRQ_FALLING, handler = højre_stop_irq)

while True:
    if venstre_knap.value() == 0:
        blink_left()
           
    if højre_knap.value() ==0:
        blink_right()