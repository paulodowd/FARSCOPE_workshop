
#include <avr/io.h>
#include "ircomm.h"

// Used to control the IR communication.
IRComm_c ircomm;

float a1;
float a2;

void setup() {
  // Set random seed.
  initRandomSeed();

  // Start the IR communication board.
  ircomm.init();
  
  // Used later for LED fading example.
  a1  = 0;
  a2 = PI;

}


// Experiment to see if reading the LSB
// of an analog read can generate a
// useful random seed.  Nothing connected
// to A0.  I think the LSB fluctuates
// randomly(?)
void initRandomSeed() {
  pinMode(A0, INPUT);
  byte r = 0x00;
  for ( int i = 0; i < 8; i++ ) {
    byte b = (byte)analogRead(A0);
    
    b = b & 0x01;
    r |= (b << i);
    delayMicroseconds(2);
  }
  //Serial.println(r, DEC);
  randomSeed( r );
}

void loop() {

  // If you don't want your board to transmit,
  // you can use the following command.
  //ircomm.stopTx();
  
  // This line must be called to process new
  // received messages and transmit new messages
  ircomm.update();

  // Tell your board to transmit a floating point
  // value like this:
  // Transmission becomes enabled by default.
  ircomm.transmitFloat(99.2);

  // Check if a message has been received:
  if( ircomm.hasMsg() ) {


    // If you want to use Serial.print() to print your
    // debug statements, you need to tell the board to 
    // stop receiving temporarily. Othewise, you will
    // receive your own debug statements!
    ircomm.disableRx();

    // Let's look at what was received:
    Serial.print("Received: ");
    Serial.println( ircomm.getFloatValue() );
    Serial.flush();

    // If you don't delete this received message, it will
    // stay for 1 second and then be automatically deleted.
    ircomm.clearRxMsg();

    // Don't forget to re-enable receiving messages after
    // you are done printing your debug statements.
    ircomm.enableRx();

  } else {

    //Serial.println("No Message");

  }

  // Example of fading an LED.
  a1 += 0.01;
  a2 += 0.01;

  // Use cos to convert a into -1:+1
  // Shift -1:+1 to 0:2
  // Map 0:2 to 0:255
  float r = 0;
  float g = (cos( a1 ) + 1.0 ) * 127.5;
  float b = (cos( a2 ) + 1.0 ) * 127.5; 

  // Note, Red LED can't fade yet.
  ircomm.setRGB( r, g, b );


  delay(10);
}
