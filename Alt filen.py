import urequests as requests
from time import sleep, ticks_ms, ticks_diff, time
from machine import Pin, UART, PWM
import gc
import secrets
import math
from gps_simple import GPS_SIMPLE
from gpio_lcd import GpioLcd
from uthingsboard.client import TBDeviceMqttClient
from adc_sub import ADC_substitute

# =====================================
# HARDWARE SETUP
# =====================================
lcd = GpioLcd(rs_pin=Pin(27), enable_pin=Pin(25),
              d4_pin=Pin(33), d5_pin=Pin(32),
              d6_pin=Pin(21), d7_pin=Pin(22),
              num_lines=4, num_columns=20)

# GPS
uart = UART(2, 9600)
gps = GPS_SIMPLE(uart)

# Batteri
adc = ADC_substitute(13)

# Alarm
buzzer = PWM(Pin(14, Pin.OUT), duty=0)
alarm_LED = Pin(26, Pin.OUT)

# ThingsBoard
client = TBDeviceMqttClient(secrets.SERVER_IP_ADDRESS, access_token=secrets.ACCESS_TOKEN)
client.connect()

# =====================================
# SETTINGS
# =====================================
alarm_armed = False
alarm_triggered = False
last_lat = None
last_lon = None
fence_radius = 0.0003  # 3 meter
last_movement_time = time()
AUTO_ARM_DELAY = 180  # auto-arm efter 3 minutter

# Timing
last_display = 0
DISPLAY_INTERVAL = 4000
last_telemetry = 0
TELEMETRY_INTERVAL = 5000
last_weather = 0
WEATHER_INTERVAL = 60000

# Vejr
weather_desc = "Loading..."
temperature = 0
wind_speed = 0
frost_status = "Unknown"

toggle = 0  # til display rotation

# =====================================
# FUNKTIONER
# =====================================

def get_gps():
    if gps.receive_nmea_data():
        lat = gps.get_latitude()
        lon = gps.get_longitude()
        speed = gps.get_speed()
        if lat != -999.0 and lon != -999.0:
            return lat, lon, speed
    return None, None, 0

def get_battery():
    val = adc.read_adc()
    batt = int((val - 1659)/(2380-1659)*100)
    return max(0, min(100, batt))

def update_weather():
    global weather_desc, temperature, wind_speed, frost_status
    try:
        url = "https://api.openweathermap.org/data/2.5/weather?q=Copenhagen,dk&APPID=a472846cd47c408475fded82cadf6336"
        r = requests.get(url, timeout=5)
        data = r.json()
        weather_desc = data["weather"][0]["description"]
        temperature = data["main"]["feels_like"] - 273.15
        wind_speed = data["wind"]["speed"]
        frost_status = "Frost!" if wind_speed < 2 and temperature < 2 else "No frost"
        r.close()
    except:
        weather_desc = "No connection"
        temperature = 0
        wind_speed = 0
        frost_status = "Unknown"

def check_movement(lat, lon):
    global last_lat, last_lon, last_movement_time, alarm_triggered
    if lat is None or lon is None:
        return
    if last_lat is None:
        last_lat, last_lon = lat, lon
        last_movement_time = time()
        return
    distance = math.sqrt((lat-last_lat)**2 + (lon-last_lon)**2)
    if distance >= fence_radius:
        last_lat, last_lon = lat, lon
        last_movement_time = time()
        if alarm_armed and not alarm_triggered:
            alarm_triggered = True
            sound_alarm()

def sound_alarm():
    for _ in range(3):
        buzzer.freq(440)
        buzzer.duty(512)
        alarm_LED.on()
        sleep(0.3)
        buzzer.duty(0)
        alarm_LED.off()
        sleep(0.3)

def auto_arm():
    global alarm_armed, alarm_triggered
    if not alarm_armed and time() - last_movement_time >= AUTO_ARM_DELAY:
        alarm_armed = True
        alarm_triggered = False

def update_display(lat, lon, speed, batt):
    global toggle
    lcd.clear()
    screen = toggle % 2
    toggle += 1
    if screen == 0:
        # GPS + hastighed + batteri + alarm
        lcd.move_to(0,0)
        lcd.putstr("Lat:"+str(lat)[:12] if lat else "Lat:N/A")
        lcd.move_to(0,1)
        lcd.putstr("Lon:"+str(lon)[:12] if lon else "Lon:N/A")
        lcd.move_to(0,2)
        lcd.putstr("Spd:"+str(speed)[:5])
        lcd.move_to(0,3)
        lcd.putstr("Bat:"+str(batt)+"% "+("A" if alarm_armed else "O"))
    else:
        # Vejr
        lcd.move_to(0,0)
        lcd.putstr("Weather Copenhagen")
        lcd.move_to(0,1)
        lcd.putstr(weather_desc[:20])
        lcd.move_to(0,2)
        lcd.putstr("Temp:"+str(int(temperature))+"C Wind:"+str(wind_speed)[:3])
        lcd.move_to(0,3)
        lcd.putstr(frost_status)

def rpc_handler(req_id, method, params):
    global alarm_armed, alarm_triggered, last_movement_time
    if method == "Toggle_knap":
        alarm_armed = bool(params)
        alarm_triggered = False
        last_movement_time = time()
        print("Alarm:", "ON" if alarm_armed else "OFF")

client.set_server_side_rpc_request_handler(rpc_handler)

# =====================================
# HOVEDLOOP
# =====================================

lcd.clear()
lcd.putstr("Smart Bike System")
sleep(2)
update_weather()

while True:
    now = ticks_ms()
    
    lat, lon, speed = get_gps()
    batt = get_battery()
    
    auto_arm()
    check_movement(lat, lon)
    
    if ticks_diff(now, last_display) > 4000:
        update_display(lat, lon, speed, batt)
        last_display = now
    
    if ticks_diff(now, last_weather) > 60000:
        update_weather()
        last_weather = now
    
    if ticks_diff(now, last_telemetry) > 5000:
        client.send_telemetry({
            "latitude": lat or 0,
            "longitude": lon or 0,
            "speed": speed,
            "battery": batt,
            "alarm_armed": alarm_armed,
            "alarm_triggered": alarm_triggered,
            "weather": weather_desc,
            "temperature": temperature,
            "wind_speed": wind_speed,
            "frost_status": frost_status
        })
        last_telemetry = now
    
    client.check_msg()
    
    if gc.mem_free() < 2000:
        gc.collect()
    
    sleep(0.1)

