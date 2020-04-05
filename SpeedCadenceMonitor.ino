#include <CurieBLE.h>

BLEPeripheral blePeripheral;            // BLE Peripheral Device (the board you're programming)

BLEService speedcadenceService("1816"); // BLE CSC
    
BLECharacteristic SpeedCadenceCharacteristic("2A5B", BLERead | BLENotify, 7);  

BLEUnsignedCharCharacteristic CSCFeature("2A5C", BLERead | BLENotify);
BLEUnsignedCharCharacteristic SensorLocation("2A5D", BLERead | BLENotify);

long previousMillis = 0;  // last time the heart rate was checked, in ms

long CumulativeWheel = 0, oldCumulativeWheel = 0;  // last Cumulative Wheel reading 
int LastWheelEvent = 0, CumulativeCrank = 0, oldCumulativeCrank = 0, LastCrankEvent = 0;
unsigned char CumWheel[4], LastWheelE[2], CumCrank[2], LastCrankE[2];

void setup() {
  Serial.begin(9600);    // initialize serial communication
  pinMode(13, OUTPUT);   // initialize the LED on pin 13 to indicate when a central is connected

  /* Set a local name for the BLE device
     This name will appear in advertising packets
     and can be used by remote devices to identify this BLE device
     The name can be changed but maybe be truncated based on space left in advertisement packet */
  blePeripheral.setLocalName("Wahoo BlueSC");
  blePeripheral.setAdvertisedServiceUuid(speedcadenceService.uuid());   // add the service UUID
  blePeripheral.addAttribute(speedcadenceService);                      // Add the BLE csc service
  
  speedcadenceService.addCharacteristic(SpeedCadenceCharacteristic); // add the speed Measurement characteristic
  speedcadenceService.addCharacteristic(CSCFeature);
  speedcadenceService.addCharacteristic(SensorLocation);
  
  /* Now activate the BLE device.  It will start continuously transmitting BLE
     advertising packets and will be visible to remote BLE central devices
     until it receives a new connection */
  blePeripheral.begin();
  Serial.println("Bluetooth device active, waiting for connections...");
}

void loop() 
{
  // listen for BLE peripherals to connect:
  BLECentral central = blePeripheral.central();

  // if a central is connected to peripheral:
  if (central) 
  {
    Serial.print("Connected to central: ");
    // print the central's MAC address:
    Serial.println(central.address());
    // turn on the LED to indicate the connection:
    digitalWrite(13, HIGH);

    // check the Speed and Cadence measurement every 200ms
    // as long as the central is still connected:
    while (central.connected()) {
      long currentMillis = millis();
      // if 200ms have passed, check the heart rate measurement:
      if (currentMillis - previousMillis >= 200) {
        previousMillis = currentMillis;
        updateSpeedCadence();
        CumulativeWheel += 1;
        LastWheelEvent += 350;
        CumulativeCrank += 1;
        LastCrankEvent += 850;
      }
    }
    // when the central disconnects, turn off the LED:
    digitalWrite(13, LOW);
    Serial.print("Disconnected from central: ");
    Serial.println(central.address());
  }
}

void updateSpeedCadence() 
{
  if (CumulativeWheel != oldCumulativeWheel) 
  {      
    CumWheel[0] = CumulativeWheel & 0x000000FF;
    CumWheel[1] = (CumulativeWheel & 0x0000FF00) >> 8;
    CumWheel[2] = (CumulativeWheel & 0x00FF0000) >> 16;
    CumWheel[3] = (CumulativeWheel & 0xFF000000) >> 24;
    LastWheelE[0] = LastWheelEvent & 0x00FF;
    LastWheelE[1] = (LastWheelEvent & 0xFF00) >> 8;
 
    unsigned char SpeedCadenceCharacteristicArray[7] = {0x01, CumWheel[0], CumWheel[1], CumWheel[2], CumWheel[3], LastWheelE[0], LastWheelE[1] };

    SpeedCadenceCharacteristic.setValue(SpeedCadenceCharacteristicArray, 7);      // update the speed and cadence measurement characteristic
    CSCFeature.setValue(0x00);
    SensorLocation.setValue(0x0B);

    // save the level for next comparison
    oldCumulativeWheel = CumulativeWheel;
  }
  
  if (CumulativeCrank != oldCumulativeCrank) 
  {      
    CumCrank[0] = CumulativeCrank & 0x000000FF;
    CumCrank[1] = (CumulativeCrank & 0x0000FF00) >> 8;
    LastCrankE[0] = LastCrankEvent & 0x00FF;
    LastCrankE[1] = (LastCrankEvent & 0xFF00) >> 8;
    
    unsigned char cadenceCharacteristicArray[5] = {0x02, CumCrank[0], CumCrank[1], LastCrankE[0], LastCrankE[1] };
    
    SpeedCadenceCharacteristic.setValue(cadenceCharacteristicArray, 5);      // update the speed and cadence measurement characteristic

    oldCumulativeCrank = CumulativeCrank;                           // save the level for next comparison
  }
  
}
