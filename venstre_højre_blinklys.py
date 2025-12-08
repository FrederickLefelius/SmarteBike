from machine import Pin
import time

venstre_LED = Pin(26, Pin.OUT)
venstre_knap = Pin(4, Pin.IN)

højre_LED = Pin(12, Pin.OUT)
højre_knap = Pin(0, Pin.IN)

#vi har defineret en start_tid for venstre knap
start_time_left = time.ticks_ms()

#vi har defineret en start_tid for højre knap
start_time_right = time.ticks_ms()

interval = 500 #ms - burde være hvor hurtigt vores LED blinker
blink_tid = 10000 #hvor lang tid den skal blinke (10sec)

højre_LED_state = 0 # dette er når vores LED == 0
venstre_LED_state = 0 # dette er når vores LED ==0 dvs. burde ikke blinke


while True:    # - venstre_knap
    
    if venstre_knap.value() == 0:
        print("venstre_knap_value: ", venstre_knap.value())
        if time.ticks_ms() - start_time_left >= interval:
            start_time_left = time.ticks_ms()
            if (venstre_LED_state == 1 and venstre_knap ==0):
                venstre_LED_state = 0
                   
            else:
                venstre_LED_state = 1
            venstre_LED.value(venstre_LED_state)
            højre_LED.off()

    if højre_knap.value() == 0: #højre_knap
        print("højre_knap_value: ", højre_knap.value())
        if time.ticks_ms() - start_time_right >= interval:
            start_time_right = time.ticks_ms()
            if (højre_LED_state == 1 and højre_knap ==0):
                højre_LED_state = 0
            else:
                højre_LED_state = 1
            højre_LED.value(højre_LED_state)
            venstre_LED.off()
            


#interrupt for venstre_knap
venstre_knap.irq(trigger=machine.Pin.IRQ_FALLING, handler = handle_interrupt) 
#interrupt for højre_knap
højre_knap.irq(triggermachine.Pin.IRQ_FALLING, handler = handle_interrupt)
