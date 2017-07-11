/* MFSC PAXC Rocket Telemetry Reciever
 * License: MIT
 */

#include <SoftwareSerial.h>

// Tx/Rx pins for HC12
const byte hc12RxPin = 4;
const byte hc12TxPin = 5;
const byte hc12SetPin = 6;

SoftwareSerial hc12(hc12TxPin, hc12RxPin);

void setup() {
  Serial.begin(9600);
  hc12.begin(9600);
  pinMode(hc12SetPin, OUTPUT);
  digitalWrite(hc12SetPin, LOW);
  delay(100);
  hc12.write("AT+DEFAULTS");
  delay(100);
  Serial.print("Reading: " + hc12.readString() + "\n"); 
  delay(50);
  hc12.write("AT+RX");
  delay(100);
  Serial.print("Parameters: " + hc12.readString() + "\n");
  
  digitalWrite(hc12SetPin, HIGH);
  delay(100);
}

void loop() {
  if (hc12.available() > 0){
    Serial.write(hc12.read());
  }
}
