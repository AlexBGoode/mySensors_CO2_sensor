/**
 * Pins order for MH-Z19_CO2: left down - up, right down - up
 * Pin 6 Vin (input voltage)
 * Pin 7 GND
 * Pin 1 Vout (output voltage 3.3V, output current lower than 10mA)
 * Pin 9 PWM
 * Pin 5 HD (zero calibration, low level above 7 seconds)
 * Pin 2 UART (RXD) 0~3.3V digital input
 * Pin 3 UART (TXD) 0~3.3V digital output
 * Pin 4 SR (Reserved)
 * Pin 8 AOT (Reserved)
 * 
 * http://www.micropik.com/PDF/HCSR04.pdf
 * http://2150692.ru/images/mh-z19_co2_manual.pdf
 **/

// Enable debug prints
#define MY_DEBUG

// Enable and select radio type attached
#define MY_RADIO_NRF24

#include <SPI.h>
#include <MySensors.h>
#define CHILD_ID_AIQ      1

#define SLEEP_TIME        11*1000 // Sleep time between reports (in milliseconds)

#define CO2_RX            A0  // RX сенсора CO2
#define CO2_TX            A1  // TX сенсора CO2
#define pinPWM            A3
#include <SoftwareSerial.h>
SoftwareSerial co2Serial( CO2_RX, CO2_TX ); // RX, TX сенсора CO2

byte command[9] = {0xFF, 0x01, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00, 0x79};
byte response[9];

// Initialize Air Quality message
MyMessage msgPPM( CHILD_ID_AIQ, V_LEVEL );


void presentation()
//void setup()
{
//  Serial.begin(9600);
  // Send the sketch version information to the gateway and Controller
  sendSketchInfo( "AIQ Sensor CO2 MH-Z19", "1.0" );

  // Register all sensors to gw (they will be created as child devices)
  present( CHILD_ID_AIQ, S_AIR_QUALITY );

  pinMode(pinPWM, INPUT);
  co2Serial.begin(9600);
//  pinMode(2, OUTPUT);
//  digitalWrite(2, HIGH); 
//  pinMode(3, OUTPUT);
//  digitalWrite(3, HIGH); 
}


void loop()
{
  static int lastReading = -1;  // The first reading always differs

  int co2ppm = readCO2_UART();
  Serial.println( co2ppm );

  // Report changes more than 10 ppm
  if (( co2ppm != lastReading ) && ( abs( co2ppm - lastReading ) >= 10 )) {
    send( msgPPM.set( co2ppm ));
    lastReading = co2ppm;
  }
//
//  int co2ppm2 = readCO2_PWM( 2000 );
//  Serial.print( "PPM: ");
//  Serial.println( co2ppm2 );

//  sleep( SLEEP_TIME );
  delay( SLEEP_TIME );
}


// Reading CO2 via UART
int readCO2_UART()
{
  // Command to ask for data, first byte fixed to 0xFF
  // http://eleparts.co.kr/data/design/product_file/SENSOR/gas/MH-Z19_CO2%20Manual%20V2.pdf

  static byte command[ 9 ] = { 0xFF, 0x01, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00, 0x79 };
  static unsigned char response[ 9 ];

  co2Serial.write(command, 9);
  memset(response, 0, 9);
  co2Serial.readBytes(response, 9);
  int i;
  byte crc = 0;
  for (i = 1; i < 8; i++) crc+=response[i];
  crc = 255 - crc;
  crc++;

  if ( !(response[0] == 0xFF && response[1] == 0x86 && response[8] == crc) ) {
    Serial.println("CRC error: " + String(crc) + " / "+ String(response[8]));
    return -1;
  } else {
    unsigned int responseHigh = (unsigned int) response[2];
    unsigned int responseLow = (unsigned int) response[3];
    int ppm = (256*responseHigh) + responseLow;
    return ppm;
  }
}


// Reading CO2 via PWM
int readCO2_PWM(int ppmLimit)
{
  static unsigned long Th, Tl, ppm = 0;

  // Th is time for HIGH level during an output cycle
  // Tl is time for LOW level during an output cycle
  // Cycle: 1004ms ± 5%

  do
  {
    // pulsein reada a pulse within a timeout in microseconds
    // and returns the length of the pulse (in microseconds)
    // or 0 if no pulse is completed before the timeout 
    Th = pulseIn( pinPWM, HIGH, 1004000 ) / 1000; // in ms
//    Th = pulseIn( pinPWM, HIGH ); // in ms
    Serial.print( '.' );
    Serial.print( Th );

    Tl = 1004 - Th;
    ppm = ppmLimit * ( Th - 2 ) / ( Th + Tl - 4 );
  } while ( Th == 0 );
}

byte getCheckSum(byte *packet)
{
  // The checksum = ( invert ( byte 1 +...+ 7 )) + 1
  byte i, checksum;
  for( i = 1; i < 8; i++)
  {
    checksum += packet[i];    // add all except byte 0
  }
  checksum = 0xFF - checksum; // invert the sum
  checksum += 1;              // increase by one
  return checksum;
}


