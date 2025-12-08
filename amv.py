from uthingsboard.client import TBDeviceMqttClient
from time import sleep
from sys import exit
import gc
import secrets
from machine import Pin
AMV = Pin(16, Pin.OUT)

# Tænd buzzer
def Blink():
    AMV.on()

# Vent i 10 sekunder
    sleep(5)

# Sluk buzzer
    AMV.off()

# the handler callback that gets called when there is a RPC request from the server
def handler(req_id, method, params):
    """handler callback to recieve RPC from server """
     # handler signature is callback(req_id, method, params)
    print(f'Response {req_id}: {method}, params {params}')
    print(params, "params type:", type(params))
    try:
        # check if the method is "toggle_led1" (needs to be configured on thingsboard dashboard)
        if method == "toggle_Blink":
            # check if the value is is "led1 on"
            if params == True:
                print("buzzer on")
                Blink()
            else:
                print("buzzer off")
        # check if command is send from RPC remote shell widget   
        if method == "sendCommand":
            print(params.get("command"))

    except TypeError as e:
        print(e)

# see more about ThingsBoard RPC at the documentation:
# https://thingsboard.io/docs/user-guide/rpc/
        
# See examples for more authentication options
# https://github.com/thingsboard/thingsboard-micropython-client-sdk/
client = TBDeviceMqttClient(secrets.SERVER_IP_ADDRESS, access_token = secrets.ACCESS_TOKEN)


# Connecting to ThingsBoard
client.connect()
print("connected to thingsboard, starting to send and receive data")
while True:
    try:
        print(f"free memory: {gc.mem_free()}")
        # monitor and free memory
        if gc.mem_free() < 2000:
            print("Garbage collected!")
            gc.collect()
        
        # uncomment for sending telemetry from device to server       
        
        #telemetry = {}
        #client.send_telemetry(telemetry)
        
        #callback to get server RPC requests
        client.set_server_side_rpc_request_handler(handler) 
        
        # Checking for incoming subscriptions or RPC call requests (non-blocking)
        client.check_msg()
        sleep(3) # blocking delay
    except KeyboardInterrupt:
        print("Disconnected!")
        # Disconnecting from ThingsBoard
        client.disconnect()
        exit()

    