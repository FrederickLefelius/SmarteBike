# -------------------------------
# IMPORT AF BIBLIOTEKER
# -------------------------------

# HTTP-forespørgsler (bruges til vejr-API)
import urequests as requests

# Tidsfunktioner
from time import sleep, ticks_ms, ticks_diff, time

# Hardwarestyring
from machine import Pin, UART, PWM, I2C

# Garbage Collection (hukommelsesstyring)
import gc

# Hemmelige nøgler (WiFi, ThingsBoard)
import secrets

# Matematiske funktioner
import math

# GPS-modul
from gps_simple import GPS_SIMPLE

# LCD-display
from gpio_lcd import GpioLcd

# ThingsBoard MQTT-klient
from uthingsboard.client import TBDeviceMqttClient

# ADC (batterimåling)
from adc_sub import ADC_substitute

# Accelerometer / Gyro
from mpu6050 import MPU6050


# -------------------------------
# HARDWARE OPSÆTNING
# -------------------------------

# LCD 20x4 display
lcd = GpioLcd(
    rs_pin=Pin(27),
    enable_pin=Pin(25),
    d4_pin=Pin(33),
    d5_pin=Pin(32),
    d6_pin=Pin(21),
    d7_pin=Pin(22),
    num_lines=4,
    num_columns=20
)

# GPS via UART
uart = UART(2, 9600)
gps = GPS_SIMPLE(uart)

# Batteri ADC
adc = ADC_substitute(34)

# Buzzer til alarm
buzzer = PWM(Pin(14, Pin.OUT), duty=0)

# Alarm LED
alarm_LED = Pin(26, Pin.OUT)

# Blinklys (venstre / højre)
venstre_LED = Pin(15, Pin.OUT)
venstre_knap = Pin(4, Pin.IN)

højre_LED = Pin(2, Pin.OUT)
højre_knap = Pin(0, Pin.IN)

# Bremselys
bremse_LED = Pin(13, Pin.OUT)

# IMU (accelerometer)
i2c = I2C(0)
imu = MPU6050(i2c)


# -------------------------------
# THINGSBOARD FORBINDELSE
# -------------------------------

client = TBDeviceMqttClient(
    secrets.SERVER_IP_ADDRESS,
    access_token=secrets.ACCESS_TOKEN
)
client.connect()


# -------------------------------
# ALARM VARIABLER
# -------------------------------

alarm_armed = False           # Om alarmen er slået til
alarm_triggered = False      # Om alarmen er udløst

last_lat = None              # Sidste GPS-position
last_lon = None

fence_radius = 0.0003        # Bevægelsesradius (geo-fence)

last_movement_time = time()  # Sidste registrerede bevægelse
AUTO_ARM_DELAY = 180         # Automatisk alarm efter 3 min


# -------------------------------
# BLINKLYS VARIABLER
# -------------------------------

turn_active = False
turn_side = None
turn_start = 0
last_blink = 0
blink_state = False

BLINK_INTERVAL = 300         # Blink-hastighed
BLINK_DURATION = 10000       # Automatisk sluk efter 10 sek


# -------------------------------
# BREMS VARIABLER
# -------------------------------

braking = False
last_brake_check = 0


# -------------------------------
# TIMING VARIABLER
# -------------------------------

last_display = 0
DISPLAY_INTERVAL = 4000

last_telemetry = 0
TELEMETRY_INTERVAL = 5000

last_weather = 0
WEATHER_INTERVAL = 60000


# -------------------------------
# VEJR VARIABLER
# -------------------------------

weather_desc = "Loading..."
temperature = 0
wind_speed = 0
frost_status = "Unknown"


toggle = 0  # Skifter mellem LCD-skærme


# -------------------------------
# GPS FUNKTION
# -------------------------------

def get_gps():
    """
    Læser GPS-data og returnerer latitude, longitude og hastighed
    """
    if gps.receive_nmea_data():
        lat = gps.get_latitude()
        lon = gps.get_longitude()
        speed = gps.get_speed()

        if lat != -999.0 and lon != -999.0:
            return lat, lon, speed

    return None, None, 0


# -------------------------------
# BATTERI FUNKTION
# -------------------------------

def get_battery():
    """
    Læser batterispænding og omregner til procent
    """
    val = adc.read_adc()
    batt = int((val - 1659) / (2380 - 1659) * 100)
    return max(0, min(100, batt))


# -------------------------------
# VEJR API
# -------------------------------

def update_weather():
    """
    Henter vejrdata fra OpenWeatherMap API
    """
    global weather_desc, temperature, wind_speed, frost_status

    try:
        url = "http://api.openweathermap.org/data/2.5/weather?q=Copenhagen,dk&APPID=a472846cd47c408475fded82cadf6336"
        r = requests.get(url, timeout=10)

        data = r.json()

        weather_desc = data["weather"][0]["description"]
        temperature = data["main"]["feels_like"] - 273.15
        wind_speed = data["wind"]["speed"]

        frost_status = "Frost!"
        if not (wind_speed < 2 and temperature < 2):
            frost_status = "No frost"

        r.close()

    except:
        weather_desc = "No connection"
        temperature = 0
        wind_speed = 0
        frost_status = "Unknown"


# -------------------------------
# BEVÆGELSES- & ALARMLOGIK
# -------------------------------

def check_movement(lat, lon):
    """
    Tjekker om cyklen har flyttet sig uden tilladelse
    """
    global last_lat, last_lon, last_movement_time, alarm_triggered

    if lat is None or lon is None:
        return

    if last_lat is None:
        last_lat, last_lon = lat, lon
        last_movement_time = time()
        return

    distance = math.sqrt((lat - last_lat)**2 + (lon - last_lon)**2)

    if distance >= fence_radius:
        last_lat, last_lon = lat, lon
        last_movement_time = time()

        if alarm_armed and not alarm_triggered:
            alarm_triggered = True
            sound_alarm()


def sound_alarm():
    """
    Aktiverer buzzer og LED
    """
    for _ in range(3):
        buzzer.freq(440)
        buzzer.duty(512)
        alarm_LED.on()
        sleep(0.3)
        buzzer.duty(0)
        alarm_LED.off()
        sleep(0.3)


def auto_arm():
    """
    Aktiverer alarm automatisk efter inaktivitet
    """
    global alarm_armed, alarm_triggered

    if not alarm_armed and time() - last_movement_time >= AUTO_ARM_DELAY:
        alarm_armed = True
        alarm_triggered = False


# -------------------------------
# RPC FRA THINGSBOARD
# -------------------------------

def rpc_handler(req_id, method, params):
    """
    Fjernstyring af alarm fra ThingsBoard
    """
    global alarm_armed, alarm_triggered, last_movement_time

    if method == "Toggle_knap":
        alarm_armed = bool(params)
        alarm_triggered = False
        last_movement_time = time()


client.set_server_side_rpc_request_handler(rpc_handler)


# -------------------------------
# BLINKLYS FUNKTIONER
# -------------------------------

def check_turn_buttons():
    """
    Starter blinklys når knap trykkes
    """
    global turn_active, turn_side, turn_start, last_blink, blink_state

    if not turn_active:
        if venstre_knap.value() == 0:
            turn_active = True
            turn_side = "left"
        elif højre_knap.value() == 0:
            turn_active = True
            turn_side = "right"

        if turn_active:
            turn_start = ticks_ms()
            last_blink = ticks_ms()
            blink_state = False


def update_turn_signals():
    """
    Opdaterer blinklys non-blocking
    """
    global turn_active, last_blink, blink_state

    if not turn_active:
        return

    now = ticks_ms()

    if ticks_diff(now, turn_start) >= BLINK_DURATION:
        turn_active = False
        venstre_LED.off()
        højre_LED.off()
        return

    if ticks_diff(now, last_blink) >= BLINK_INTERVAL:
        blink_state = not blink_state

        if turn_side == "left":
            venstre_LED.value(blink_state)
        else:
            højre_LED.value(blink_state)

        last_blink = now


# -------------------------------
# BREMS FUNKTION
# -------------------------------

def check_braking():
    """
    Tænder bremselys ved negativ acceleration
    """
    global braking, last_brake_check

    now = ticks_ms()
    if ticks_diff(now, last_brake_check) < 100:
        return

    last_brake_check = now

    try:
        values = imu.get_values()
        x_acc = values.get("acc x")

        if x_acc is not None and x_acc < -2000:
            braking = True
            bremse_LED.on()
        else:
            braking = False
            bremse_LED.off()
    except:
        pass


# -------------------------------
# LCD DISPLAY
# -------------------------------

def update_display(lat, lon, speed, batt):
    """
    Skifter mellem GPS-skærm og vejr-skærm
    """
    global toggle

    lcd.clear()
    screen = toggle % 2
    toggle += 1

    if screen == 0:
        lcd.putstr("Lat:" + (str(lat)[:12] if lat else "N/A"))
        lcd.move_to(0,1)
        lcd.putstr("Lon:" + (str(lon)[:12] if lon else "N/A"))
        lcd.move_to(0,2)
        lcd.putstr("Spd:" + str(speed) + " Bat:" + str(batt) + "%")
        lcd.move_to(0,3)
        lcd.putstr("Alarm:" + ("ON" if alarm_armed else "OFF"))
    else:
        lcd.putstr("Weather Copenhagen")
        lcd.move_to(0,1)
        lcd.putstr(weather_desc[:20])
        lcd.move_to(0,2)
        lcd.putstr("Temp:" + str(int(temperature)) + "C")
        lcd.move_to(0,3)
        lcd.putstr(frost_status)


# -------------------------------
# OPSTART
# -------------------------------

lcd.clear()
lcd.putstr("Smart Bike System")
lcd.move_to(0,1)
lcd.putstr("Starter op")
sleep(2)

update_weather()


# -------------------------------
# HOVEDLOOP
# -------------------------------

try:
    while True:
        now = ticks_ms()

        lat, lon, speed = get_gps()
        batt = get_battery()

        auto_arm()
        check_movement(lat, lon)

        check_turn_buttons()
        update_turn_signals()

        check_braking()

        if ticks_diff(now, last_display) > DISPLAY_INTERVAL:
            update_display(lat, lon, speed, batt)
            last_display = now

        if ticks_diff(now, last_weather) > WEATHER_INTERVAL:
            update_weather()
            last_weather = now

        if ticks_diff(now, last_telemetry) > TELEMETRY_INTERVAL:
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
                "frost_status": frost_status,
            })
            last_telemetry = now

        client.check_msg()

        if gc.mem_free() < 2000:
            gc.collect()

        sleep(0.05)

except KeyboardInterrupt:
    print("Program afsluttet")
