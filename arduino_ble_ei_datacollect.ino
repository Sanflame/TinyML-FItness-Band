#include <ArduinoBLE.h>
#include <Arduino_LSM9DS1.h>

#define BLE_UUID_SENSOR_DATA_SERVICE              "2BEEF31A-B10D-271C-C9EA-35D865C1F48A"
#define BLE_UUID_MULTI_SENSOR_DATA                "4664E7A1-5A13-BFFF-4636-7D0A4B16496C"

#define NUMBER_OF_SENSORS 3
#define UPDATE_INTERVALL 20

const int ledPin = LED_BUILTIN; // set ledPin to on-board LED

union multi_sensor_data
{
  struct __attribute__( ( packed ) )
  {
    float values[NUMBER_OF_SENSORS];
  };
  uint8_t bytes[ NUMBER_OF_SENSORS * sizeof( float ) ];
};

union multi_sensor_data multiSensorData;

//Service for publish Accelerometer value...
BLEService sensorDataService( BLE_UUID_SENSOR_DATA_SERVICE );
BLECharacteristic
 multiSensorDataCharacteristic( BLE_UUID_MULTI_SENSOR_DATA, BLERead | 
BLENotify, sizeof multiSensorData.bytes );

float acc_x, acc_y, acc_z;

void setup() {
  Serial.begin(115200);
  //while (!Serial);

  pinMode(ledPin, OUTPUT); // use the LED as an output
 
  // begin initialization
  if (!BLE.begin()) {
    Serial.println("starting BLE failed!");

    while (1);
  }

  if (!IMU.begin())
  { 
    Serial.println("Failed to initialize IMU!");
    while (1);
  }



  BLE.setDeviceName("IMU");
  // set the local name peripheral advertises
  BLE.setLocalName("IMU");
  
  BLE.setAdvertisedService(sensorDataService);
  
  // BLE add characteristics
  sensorDataService.addCharacteristic( multiSensorDataCharacteristic );
  
  // add service
  BLE.addService( sensorDataService );
  
  // set the initial value for the characeristic:
  multiSensorDataCharacteristic.writeValue( multiSensorData.bytes, sizeof multiSensorData.bytes );
  
  BLE.setConnectable(true);
  // BLE.setAdvertisingInterval(100);// configure the interval time for BLE
  BLE.advertise();
  
  Serial.println("Bluetooth Device Active, Waiting for Connections...");

  Serial.println(" AX \t AY \t AZ");
}

void loop() {
  
  static long previousMillis = 0;
  
  BLEDevice central = BLE.central();

  if(central) {
   Serial.print("Connected to Central: ");
   Serial.println(central.address());
  while(central.connected()) {
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= UPDATE_INTERVALL) {
      previousMillis = currentMillis;
      
      IMU.readAcceleration(acc_x, acc_y, acc_z);
      multiSensorData.values[0] = acc_x;
      multiSensorData.values[1] = acc_y;
      multiSensorData.values[2] = acc_z;
      multiSensorDataCharacteristic.writeValue( multiSensorData.bytes, sizeof multiSensorData.bytes );
      

    }
  }
  }
  Serial.print("Disconnected from Central: ");
  Serial.println(BLE.address());

}
