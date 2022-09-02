// FLASH Shutter Control Arduino Script
// 
// Code constantly checks for serial data
// Gets serial data in form of an integer
// When serial data received, set pin 9 to high for ms = integer recieved
//
// WARNING: REQUIRES INTEGER TO BE SENT TO ARDUINO... DO NOT ALLOW ANYTHNG ELSE

const byte numChars = 32;
char receivedChars[numChars];   // an array to store the received data
int pulse;

boolean newData = false;

void setup() {
    Serial.begin(9600);
    Serial.println("<Arduino is ready>");
    DDRB |= B00000010; // Set digital pin 9 as output
}

void loop() {
    recvWithEndMarker();
    showNewData();
}

void recvWithEndMarker() {
    static byte ndx = 0;
    char endMarker = '\n';
    char rc;
    
    while (Serial.available() > 0 && newData == false) {
        rc = Serial.read();

        if (rc != endMarker) {
            receivedChars[ndx] = rc;
            ndx++;
            if (ndx >= numChars) {
                ndx = numChars - 1;
            }
        }
        else {
            receivedChars[ndx] = '\0'; // terminate the string
            ndx = 0;
            newData = true;
        }
    }
}

void showNewData() {
    if (newData == true) {
        Serial.print("This just in ... ");
        Serial.println(receivedChars);
        pulse = atoi(receivedChars);
        PORTB |= B00000010; // Set digital pin 9 high
        delay(pulse);
        PORTB &= B00000000; // Set digital pin 9 low
        newData = false;
    }
}
