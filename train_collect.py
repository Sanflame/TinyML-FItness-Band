import json
import time, hmac, hashlib
import requests
import re, uuid
import math
import sys
import time
import binascii
import struct
from bluepy import btle
from bluepy.btle import UUID, Peripheral



if len(sys.argv) != 4:
	print("Fatal, must pass device address:", sys.argv[0], "<device address="">")
	quit()
accelServiceUuid = "2BEEF31A-B10D-271C-C9EA-35D865C1F48A"
accCharUuid = "4664E7A1-5A13-BFFF-4636-7D0A4B16496C"
peripheralObject = btle.Peripheral(sys.argv[1])


# Your API & HMAC keys can be found here (go to your project > Dashboard > Keys to find this)
HMAC_KEY = "Your HMAC Key"
API_KEY = "Your API Key"


mySensor = btle.UUID(accelServiceUuid)
sensorService = peripheralObject.getServiceByUUID(mySensor)


accValue = sensorService.getCharacteristics(accCharUuid)[0]

#print(cur_time,"\t", acc_x,"\t",acc_y,"\t",acc_z,"\t")
		


# empty signature (all zeros). HS256 gives 32 byte signature, and we encode in hex, so we need 64 characters here
emptySignature = ''.join(['0'] * 64)

# use MAC address of network interface as deviceId
device_name ="Temp_BLe"
NAME=sys.argv[2]
TIME=int(sys.argv[3])
# here we have new data every 16 ms
INTERVAL_MS = 90

if INTERVAL_MS <= 0:
    raise Exception("Interval in miliseconds cannot be equal or lower than 0.")

# here we'll collect 2 seconds of data at a frequency defined by interval_ms
freq =1000/INTERVAL_MS
values_list=[]
for i in range (TIME*int(round(freq,0))):
    accVal=accValue.read()
    accV=[accVal[i:i+4] for i in range(0, len(accVal), 4)]
    acc_x=struct.unpack('f',accV[0])[0]
    acc_y=struct.unpack('f',accV[1])[0]
    acc_z=struct.unpack('f',accV[2])[0]
    values_list.append([acc_x*9.865,acc_y*9.865,acc_z*9.865])

data = {
    "protected": {
        "ver": "v1",
        "alg": "HS256",
        "iat": time.time() # epoch time, seconds since 1970
    },
    "signature": emptySignature,
    "payload": {
        "device_name":  device_name,
        "device_type": "BLE_TEST_DEVICE",
        "interval_ms": INTERVAL_MS,
        "sensors": [
            { "name": "accX", "units": "m/s2" },
            { "name": "accY", "units": "m/s2" },
            { "name": "accZ", "units": "m/s2" }
        ],
        "values": values_list
    }
}



# encode in JSON
encoded = json.dumps(data)

# sign message
signature = hmac.new(bytes(HMAC_KEY, 'utf-8'), msg = encoded.encode('utf-8'), digestmod = hashlib.sha256).hexdigest()

# set the signature again in the message, and encode again
data['signature'] = signature
encoded = json.dumps(data)

# and upload the file
res = requests.post(url='https://ingestion.edgeimpulse.com/api/training/data',
                    data=encoded,
                    headers={
                        'Content-Type': 'application/json',
                        'x-file-name': NAME,
                        'x-api-key': API_KEY
                    })
if (res.status_code == 200):
    print('Uploaded file to Edge Impulse', res.status_code, res.content)
else:
    print('Failed to upload file to Edge Impulse', res.status_code, res.content)
