import requests# bruges til at hente data fra internettet via HTTP (API-kald).
from machine import Pin#styrer GPIO-ben. fx LED’ og relæ.
from time import sleep# Pauser programmet
import esp32#Giver adgang til ESP32's specifikke funktioner


try:# Programmet prøver, og hvis det ikke kan så crasher programmet ikke.
    import ujson as json# Importerer ujson som json( ujson er micropythons version af json)
except ImportError:#Hvis den ikke kan finde ujson så importerer den bare normale json.
    import json 
RL = Pin(22, Pin.OUT) #Rød LED får Pin 22 som output
GL = Pin(23, Pin.OUT) #Grøn LED får Pin 23 som output
relay = Pin(16, Pin.OUT) #Relæet får Pin 16 som output.


while True:# Koden kører for evigt.
    CO2_response = requests.get(url='https://api.energidataservice.dk/dataset/CO2Emis?limit=2')# Henter data fra Energidataservice's API.
    CO2_result = CO2_response.json()#Laver dataen om til Pythonobjekt via json.
    CO2_records = CO2_result.get('records', []) #Finder listen "records" i json data fra url.
    
    for record in CO2_records: #For hvert record (Vi bruger ikke record) printer du altid element nr. 1 i listen.
        print(CO2_records[1])  #printer det første element. 
        co2_emission = CO2_result["records"][1]["CO2Emission"] # Får co2 emission fra index 1 i anden record.
        sleep(1)# Programmet venter i 1 sekund før det begynder igen.
        if co2_emission <= 100: # Hvis co2 værdien er 100 eller lavere.
            
            print(co2_emission) #Printer DK2 co2 emission fra listen.
            GL.on() #Grøn LED tænder.
            relay.on() #Relæ tænder.
            RL.off() #Rød LED slukker.
            print('grøn') #Printer "grøn".
        else:# Hvis co2 værdien er over 100.
            print(co2_emission) #Printer DK2 co2 emission fra listen.
            RL.on() #Rød LED tænder.
            relay.off() #Relæ slukker.
            GL.off() # Grøn LED slukker.
            print('ikke grøn')#Printer "Ikke grøn".
