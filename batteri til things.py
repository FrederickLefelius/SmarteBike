#     for i in range (256):
#         adcVal += adc.read()
#         #sleep(0.05)
#     adcVal = adcVal >> 8
#
#     print("ADC: %4d, %.4f V" % (adcVal, adcVal * 3.3 / 4095))
#     sleep(1)
#

from adc_sub import ADC_substitute
from time import sleep
from machine import Pin
from gpio_lcd import GpioLcd
import secrets
import gc
from uthingsboard.client import TBDeviceMqttClient

#thingsboard setup
client = TBDeviceMqttClient(secrets.SERVER_IP_ADRESS, access_token=secrets_ACCESS_TOKEN)
client.connect()
print("connected to thingsboard")


x1=1659
#3v
y1=0
x2=2380
#4,2
y2=100

a= (y2-y1)/(x2-x1)
b = y2 - a*x2

lcd = GpioLcd(rs_pin=Pin(27), enable_pin=Pin(25),
              d4_pin=Pin(33), d5_pin=Pin(32), d6_pin=Pin(21), d7_pin=Pin(22),
              num_lines=4, num_columns=20)

def formel(x):
    y= a*x+b
    return int(y) #laver vores batteristatus om til integer(hel tal)

adc = ADC_substitute(13)

def send_telemetry(batt_volt):
    client.send.telemetry({
        "batteryLevel": float(batt_volt)
        })

while True:
    adc_val = adc.read_adc()
    v = adc.read_voltage()
    batt_volt = formel(adc_val)
    print("ADC: %1d, %.1f V, %.0f %%" % (adc_val, v, batt_volt))
    lcd.clear()
    lcd.move_to(0,0)
    lcd.putstr(f'batteri: {batt_volt}%')
    client.send.telemetry({
        "batteryLevel":batt_volt
        })
#     v = 0.000838616 * a + 0--------------------------------------------------------.079035
#     print("ADC: %4d, %.4f V" % (a, v))
   
   
    sleep(1)
                  

    
            