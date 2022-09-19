#include "ircomm.h"

IRComm_c::IRComm_c() {
}

void IRComm_c::init() {

  // Debug led
  pinMode(13, OUTPUT);

  // RGB LED
  pinMode(LED_R, OUTPUT);
  pinMode(LED_G, OUTPUT);
  pinMode(LED_B, OUTPUT);

  // Input pots
  pinMode(POT_A, INPUT);
  pinMode(POT_B, INPUT);
  pinMode(POT_C, INPUT);

  // Set pin for 38khz carrier as output
  pinMode(TX_CLK_OUT, OUTPUT);

  state = STATE_IR_TX_ON;


  // Set message buffers to invalid (empty) state
  memset(tx_buf, 0, sizeof(tx_buf));
  memset(rx_buf, 0, sizeof(rx_buf));
  memset(rx_msg, 0, sizeof(rx_msg));

  // Setup Timer 2 to help generate a 38khz signal.
  setupTimer2();

  // Start assuming we are not transmitting
  // a message, and that we have not received
  // a message to process.
  BROADCAST = false;
  PROCESS_MSG = false;
  rx_count = 0;


  // Start Serial.  Note, here we are using
  // serial to transmit strings over IR by
  // modulating it with the 38khz carrier.
  // The IR demodulator chip can support
  // baud up to 4800, but other variants
  // might not.
  Serial.begin(4800);
  Serial.setTimeout(250);


  // Set initial time-stamp values
  ts = millis();
  msg_ttl = millis();

  tx_ts = millis();
  setTXDelay();
  resetRxBuf();
}

// Buffer size is 32
// We have to add:
// - start token, 1 byte '*'
// - checksum token, 1 byte '@'
// - checksum itself, 1 byte
// So 3 bytes subtracted from the buffer
// So the maxmimum length string we can send is 29 bytes
void IRComm_c::transmitString(char* str_to_send, int len) {
  char buf[MAX_MSG];
  int count;

  if (len > 29) return;

  // Clear buffer
  memset(buf, 0, sizeof(buf));

  count = 0;

  // set first character as our message
  // start byte
  buf[count++] = '*';

  // Copy string in
  for (int i = 0; i < len; i++) {
    buf[count++] = str_to_send[i];
  }

  // Add checksum token
  buf[count++] = '@';

  // Add checksum
  buf[count++] = CRC(buf, strlen(buf));

  // Stop transmission whilst we update
  // the buffer.
  state = STATE_IR_TX_OFF;

  // Copy to tx buffer
  memset(tx_buf, 0, sizeof(tx_buf));
  for (int i = 0; i < count; i++) {
    tx_buf[i] = buf[i];
  }

  //Serial.println("Created:");
  //Serial.print( buf );
  //Serial.println("END");

  // Restart IR transmission
  state = STATE_IR_TX_ON;
}


// This takes a number, converts it to a float, then
// it is sent to transmitString()
void IRComm_c::transmitFloat(float f_to_send) {

  char buf[MAX_MSG];

  // Convert float to a string, store in the
  // message buffer.
  // I had a lot of trouble finding a solution for this.
  // This is an odd, non-standard function I think.
  // dtostrf(float_value, min_width, num_digits_after_decimal, where_to_store_string)
  // https://www.programmingelectronics.com/dtostrf/
  //  - Convert "heading"
  //  - a minimum of 6 character (e.g. 000.00)
  //  - 2 digits after decimal
  //  - store in buf
  dtostrf(f_to_send, 6, 2, buf);
  transmitString(buf, strlen(buf));
}


uint8_t IRComm_c::CRC(char* buf, int len) {
  uint8_t cs;
  int i;

  cs = 0;
  for (i = 0; i < len; i++) {
    cs = cs ^ buf[i];
  }
  cs &= 0xff;

  return cs;
}


// The arduino pro mini has a parallel serial
// interface,meaning with IR it can receive
// it's own tranmission. There, we access the
// UART register to disable RX functionality.
// When we re-enable it, this has the beneficial
// side-effect of clearing the input buffer.

void IRComm_c::disableRx() {
  cli();
  UCSR0B &= ~(1 << RXEN0);
  UCSR0B &= ~(1 << RXCIE0);
  sei();
}

// Re-enable the serial port RX hardware.
void IRComm_c::enableRx() {


  cli();
  UCSR0A &= ~(1 << FE0);
  UCSR0A &= ~(1 << DOR0);
  UCSR0A &= ~(1 << UPE0);
  UCSR0B |= (1 << RXEN0);
  UCSR0B |= (1 << RXCIE0);
  sei();
}


// Clears our received buffer
void IRComm_c::resetRxBuf() {
  rx_count = 0;
  PROCESS_MSG = false;
  memset(rx_buf, 0, sizeof(rx_buf));
}

// Attempting an asynchronous send
// and listen procedure because we
// can't do both at the same time.
// https://lucidar.me/en/serialib/most-used-baud-rates-table/
// We are using 4800 baud, which is
// 4800 bits per second.
// 1.042ms per byte.
// So we space transmission by 64ms
// to allow for receipt, and pad/vary
// by upto 50ms(?)
void IRComm_c::setTXDelay() {
  float t = (float)random(0, 125);
  t += 125;
  // Insert random delay to help
  // break up synchronous tranmission
  // between robots.
  tx_delay = (unsigned long)t;
}

void IRComm_c::setRGB(int r, int g, int b) {
  if (r == 0) {
    digitalWrite(LED_R, LOW);
  } else {
    digitalWrite(LED_R, HIGH);
  }

  if (g == 0) {
    analogWrite(LED_G, 0);
  } else {
    analogWrite(LED_G, g);
  }


  if (b == 0) {
    analogWrite(LED_B, 0);
  } else {
    analogWrite(LED_B, b);
  }
}

// We use Timer2 to generate a 38khz clock
// which is electronically OR'd with the
// serial TX.
void IRComm_c::setupTimer2() {

  // Termporarily stop interupts
  cli();

  // Setup Timer 2 to fire ISR every 38khz
  // Enable CTC mode
  TCCR2A = 0;
  TCCR2A |= (1 << WGM21);

  // Setup ctc prescaler
  TCCR2B = 0;

  // No prescale?
  //TCCR2B |= (1<<CS21);
  TCCR2B |= (1 << CS20);

  // match value
  OCR2A = 210;  // counts to 53, every 38,000hz
  //OCR2A = 53; // counts to 53, every 38,000hz


  // Set up which interupt flag is triggered
  TIMSK2 = 0;
  TIMSK2 |= (1 << OCIE2A);

  // enable interrupts
  sei();
}

void IRComm_c::stopTx() {
  state = STATE_IR_TX_OFF;
}

void IRComm_c::startTx() {

  state = STATE_IR_TX_ON;

}

void IRComm_c::clearRxMsg() {
  memset(rx_msg, 0, sizeof(rx_msg));
  msg_ttl = millis();
}

float IRComm_c::getFloatValue() {
  return atof( rx_msg );
}

void IRComm_c::update() {

  // Toggle whether we are broadcasting
  // or listening for IR messages
  if (millis() - tx_ts > tx_delay) {
    BROADCAST = !BROADCAST;
    tx_ts = millis();

    // Set a different delay for next
    // iteration
    setTXDelay();
  }

  // Clear status LED
  if ( millis() - ts > 100 ) {
    ts = millis();
    digitalWrite( 13, LOW );
  }

  // Clear old messages received via IR.
  if (millis() - msg_ttl > TTL) {
    memset(rx_msg, 0, sizeof(rx_msg));
    msg_ttl = millis();
  }

  // If we are in broadcast mode and the LEDS
  // are active.
  if (state == STATE_IR_TX_ON && BROADCAST) {
    if (strlen(tx_buf) == 0 || tx_buf[0] == '!' ) {
      // don't send, empty buffer.

    } else {  // Message from Master to send.

      // We have a problem where we pick up
      // our own reflected transmission.
      disableRx();


      // Using Serial.print transmits over
      // IR.  Serial TX is modulated with
      // the 38Khz carrier in hardware.
      Serial.println(tx_buf);
      Serial.flush();  // wait for send to complete
      //delay(2);

      // With TX finished, we can receive again
      // and not get our own transmission.
      enableRx();
    }
  }


  // Check for new message in serial buffer
  // Reads in currently available char's, should
  // not take long.
  while (Serial.available() && !PROCESS_MSG) {
    // If we find a newline, we flag we have
    // a message to attempt to process.
    rx_buf[rx_count] = Serial.read();
    if (rx_buf[rx_count] == '\n') PROCESS_MSG = true;


    // Note that, it might seem like rx_count will
    // exceed the buffer. The next check should
    // catch this, and cause the buffer to be
    // processed.  We don't zero rx_count on each
    // call of loop() because we may only have a
    // few characters in the Serial buffer.  So we
    // need to incrementally fill our rx_buf with
    // many calls to this while loop.
    // Processing the message will reset rx_count
    // to 0.
    rx_count++;

    // If we exceed the buffer size, we also
    // will try to process the message.
    if (rx_count >= MAX_MSG) PROCESS_MSG = true;
  }


  // Do we think we have a message to process?
  if (PROCESS_MSG) {

    // Debug
    //Serial.print("got "); Serial.print(rx_buf); Serial.print(":"); Serial.println(rx_count);

    // Invalid conditions:
    if (rx_count <= 0 || rx_buf[0] == 0) {
      // bad read.
      //Serial.println("bad serial");
      resetRxBuf();

    } else {
      //disableRx();
      processRxBuf();
      //Serial.flush();
      //enableRx();
    }
  }
}

// Needs to validate what is in the rx buffer
// by recalculating and comparing the checksum
int IRComm_c::processRxBuf() {


  //Serial.print("\n\nProcessing: ");
  //Serial.print( rx_buf );
  //Serial.print(" rx_count = " );
  //Serial.println( rx_count );

  // Check for start byte
  int start = findChar('*', rx_buf, rx_count);  // -1 = default error state

  //Serial.print("Start at: " );
  //Serial.println( start );

  // Message too short or bad
  // e.g. a good message would be:
  //      *abcdef@a  (start,message,CRC token, CRC value)
  //      a useless message would be:
  //      minimum (absurd) message would be *@_
  //      min message length = 3
  // if start < 0, then no * symbol was found
  if ( start < 0 ) {
    // No start token.
    // But a newline or buffer full must have brought
    // us here.
    //Serial.println("No start token");


  } else if ((rx_count - start) <= 3 ) {
    //Serial.println("message too short :(");


  } else {  // message valid, * index found

    // Check CRC, find @ token to separate
    // out CRC value.
    int last = findChar('@', rx_buf, rx_count);

    //Serial.print(" checksum after ");
    //Serial.println( last );

    // Start should be 0 or more
    // Note, we include the start byte *
    // in the checksum
    if (last > start && last < (rx_count - 1)) {
      char buf[rx_count];
      memset(buf, 0, sizeof( buf ));
      int b = 0;  // count how many bytes

      // we copy.
      for (int i = start; i <= last; i++) {
        buf[b] = rx_buf[i];
        b++;
      }

      // We need at least 1 byte of data
      // between start and checksum
      if (b >= 1) {
        //Serial.print("copied: " );
        //Serial.print( buf );
        //Serial.print( " b = ");
        //Serial.println(b);

        // Reconstruct checksum
        uint8_t cs;
        cs = CRC(buf, strlen(buf));

        //Serial.print( cs );
        //Serial.print( " vs ");
        //Serial.println( rx_buf[last + 1]);

        // Do the checksums match?
        if (cs == rx_buf[last + 1]) {

          // Debug
          //Serial.println("CRC GOOD!");

          // Flash so we can see when robots are
          // receiving messages correctly
          digitalWrite(13, HIGH);
          //analogWrite(LED_R, 0 );
          //analogWrite(LED_G, 255);
          //setRGB( 0, 1, 0 );

          // Make sure where we will store this
          // received message is clear.
          memset(rx_msg, 0, sizeof(rx_msg));

          // Copy message across, ignoring the *
          for (int i = 0; i < b - 2; i++) {
            rx_msg[i] = buf[i + 1];
          }

          // Add a ! to the message, which will be
          // used when later sending down I2C to
          // the Master to indicate the end of the
          // string.
          rx_msg[b - 2] = '!';

          // Reset TTL timestamp, so this message
          // will only exist temporarily.
          msg_ttl = millis();


        } else {

          // Checksums do not match, corrupt message.

          //Serial.println("bad checksum");
        }
      }
    } else {

      // checksum token missing or invalid

      //Serial.println("invalid message");
    }
  }
  resetRxBuf();
}

int IRComm_c::findChar(char c, char* str, int len) {
  for (int i = 0; i < len; i++) {
    if (str[i] == c) {
      return i;
    }
  }
  return -1;
}

int IRComm_c::hasMsg() {
  if (rx_msg[0] == '!') {
    return -1;
  }
  return strlen(rx_msg);
}

// This ISR simply toggles the state of
// pin D4 to generate a 38khz clock signal.
ISR(TIMER2_COMPA_vect) {
  PORTD ^= (1 << PD4);
}
