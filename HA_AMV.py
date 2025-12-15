import network
import time
from umqtt.simple import MQTTClient
from uthingsboard.client import TBDeviceMqttClient
import dht
from machine import Pin
import gc

# ---- WiFi ----
ssid = "Gruppe4"
password = "gruppe4b"

# ---- Hardware Pins ----
sensor = dht.DHT11(Pin(19))
AMV = Pin(16, Pin.OUT)

# ---- ThingsBoard ----
tb_server = "demo.thingsboard.io"
tb_token = "BxGt1zd4kwJgCWqQNsBr"

# ---- Home Assistant MQTT ----
ha_server = "172.16.2.13"
ha_port = 1883
ha_user = "G4"
ha_password = "gruppe4b"
ha_sensor_topic = "homeassistant/sensor/esp32/state"
ha_buzzer_cmd_topic = "homeassistant/switch/esp32_buzzer/set"
ha_buzzer_state_topic = "homeassistant/switch/esp32_buzzer/state"

# ---- Buzzer Function ----
def Blink():
    print("Buzzer ON")
    AMV.on()
    time.sleep(5)
    AMV.off()
    print("Buzzer OFF")
    # Send status til Home Assistant
    try:
        ha_client.publish(ha_buzzer_state_topic, b"OFF")
    except:
        pass

# ---- ThingsBoard RPC Handler ----
def tb_handler(req_id, method, params):
    """Handler for ThingsBoard RPC commands"""
    print(f'TB RPC: {method}, params: {params}')
    try:
        if method == "toggle_Blink":
            if params == True:
                print("Buzzer activated from ThingsBoard")
                Blink()
            else:
                print("Buzzer deactivated from ThingsBoard")
                AMV.off()
        
        if method == "sendCommand":
            print(params.get("command"))
    except Exception as e:
        print(f"TB handler error: {e}")

# ---- Home Assistant MQTT Callback ----
def ha_callback(topic, msg):
    """Handler for Home Assistant MQTT commands"""
    print(f"HA Command: {topic} = {msg}")
    try:
        if topic == ha_buzzer_cmd_topic.encode():
            if msg == b'ON':
                print("Buzzer activated from Home Assistant")
                Blink()
            elif msg == b'OFF':
                print("Buzzer stopped from Home Assistant")
                AMV.off()
                ha_client.publish(ha_buzzer_state_topic, b"OFF")
    except Exception as e:
        print(f"HA callback error: {e}")

# ---- WiFi Connect ----
print("Connecting to WiFi...")
wifi = network.WLAN(network.STA_IF)
wifi.active(True)
wifi.connect(ssid, password)

while not wifi.isconnected():
    time.sleep(0.5)
print(" WiFi Connected:", wifi.ifconfig())

# ---- Connect to ThingsBoard ----
print("Connecting to ThingsBoard...")
tb_client = TBDeviceMqttClient(tb_server, access_token=tb_token)
tb_client.connect()
print(" Connected to ThingsBoard")

# ---- Connect to Home Assistant MQTT ----
print("Connecting to Home Assistant...")
ha_client = MQTTClient(
    client_id="ESP32_Combined",
    server=ha_server,
    port=ha_port,
    user=ha_user,
    password=ha_password
)
ha_client.set_callback(ha_callback)
ha_client.connect()
print(" Connected to Home Assistant")

# Subscribe to Home Assistant buzzer commands
ha_client.subscribe(ha_buzzer_cmd_topic)
print(f" Subscribed to: {ha_buzzer_cmd_topic}")

# Publish Home Assistant MQTT Discovery for buzzer (så den dukker automatisk op)
ha_buzzer_config_topic = "homeassistant/switch/esp32_buzzer/config"
ha_buzzer_config = '''{
    "name": "ESP32 Buzzer",
    "unique_id": "esp32_buzzer_001",
    "command_topic": "homeassistant/switch/esp32_buzzer/set",
    "state_topic": "homeassistant/switch/esp32_buzzer/state",
    "payload_on": "ON",
    "payload_off": "OFF",
    "icon": "mdi:alarm-bell",
    "optimistic": false,
    "retain": true
}'''
ha_client.publish(ha_buzzer_config_topic, ha_buzzer_config, retain=True)
ha_client.publish(ha_buzzer_state_topic, b"OFF", retain=True)
print(" Buzzer switch published to Home Assistant")


print("System Ready!")
print("- DHT11 sensor sending data every 5 sec")
print("- Buzzer controllable from both platforms")
