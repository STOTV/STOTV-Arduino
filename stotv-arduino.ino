#include <IridiumSBD.h>
#include <SoftwareSerial.h>
#include <TinyGPS++.h> // NMEA parsing: http://arduiniana.org
#include <PString.h> // String buffer formatting: http://arduiniana.org

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/common.h>
#include <avr/wdt.h>
#include <avr/sleep.h>

#define BEACON_INTERVAL 42200 // Time between transmissions,12 hours
#define BEACON_RETRY_INTERVAL 3600 // Time to wait if transmission was a failure, 1 Hour
#define ROCKBLOCK_RX_PIN 18
#define ROCKBLOCK_TX_PIN 19
#define ROCKBLOCK_SLEEP_PIN 10
#define ROCKBLOCK_BAUD 19200
#define GPS_RX_PIN 3
#define GPS_TX_PIN 4
#define GPS_BAUD 4800
#define CONSOLE_BAUD 115200

SoftwareSerial ssIridium(ROCKBLOCK_RX_PIN, ROCKBLOCK_TX_PIN);
SoftwareSerial ssGPS(GPS_RX_PIN, GPS_TX_PIN);
IridiumSBD isbd(ssIridium, ROCKBLOCK_SLEEP_PIN);
TinyGPSPlus tinygps;
int pingInterval=BEACON_INTERVAL; //master ping time
int nextPing=BEACON_INTERVAL; //when should it ping again
int retryCount=0;

void setup(){
  // Start the serial ports
  Serial.begin(CONSOLE_BAUD);

  // Setup the RockBLOCK
  isbd.attachConsole(Serial);
  isbd.attachDiags(Serial);
  isbd.setPowerProfile(1);
  isbd.useMSSTMWorkaround(false);
}

void loop(){
  bool fixFound = false;
  bool charsSeen = false;
  unsigned long loopStartTime = millis();

  // Step 0: Start the serial ports
  ssIridium.begin(ROCKBLOCK_BAUD);
  ssGPS.begin(GPS_BAUD);

  // Step 1: Reset TinyGPS++ and begin listening to the GPS
  Serial.println("Beginning to listen for GPS traffic...");
  tinygps = TinyGPSPlus();
  ssGPS.listen();

  // Step 2: Look for GPS signal for up to 7 minutes
  for (unsigned long now = millis(); !fixFound && millis() - now < 7UL * 60UL * 1000UL;)
    if (ssGPS.available()){
      charsSeen = true;
      tinygps.encode(ssGPS.read());
      fixFound = tinygps.location.isValid() && tinygps.date.isValid() &&
        tinygps.time.isValid() && tinygps.altitude.isValid();
    }

  Serial.println(charsSeen ? fixFound ? F("A GPS fix was found!") : F("No GPS fix was found.") : F("Wiring error: No GPS data seen."));

  // Step 3: Start talking to the RockBLOCK and power it up
  Serial.println("Beginning to talk to the RockBLOCK...");
  ssIridium.listen();
  if (isbd.begin() == ISBD_SUCCESS){
    char outBuffer[80]; // Always try to keep message short, 340 Max
    if (fixFound){
      sprintf(outBuffer, "%d%02d%02d%02d%02d%02d,",
        tinygps.date.year(), tinygps.date.month(), tinygps.date.day(),
        tinygps.time.hour(), tinygps.time.minute(), tinygps.time.second());
      int len = strlen(outBuffer);
      PString str(outBuffer, sizeof(outBuffer) - len);
      str.print(tinygps.location.lat(), 6);
      str.print(",");
      str.print(tinygps.location.lng(), 6);
      str.print(",");
      str.print(tinygps.altitude.meters());
      str.print(",");
      str.print(tinygps.speed.knots(), 1);
      str.print(",");
      str.print(tinygps.course.value() / 100);
      Serial.print("Transmitting message: ");
      Serial.println(outBuffer);
      uint8_t buffer[200]; // 270 Max
      size_t bufferSize=sizeof(buffer);
      int err=isbd.sendReceiveSBDText(outBuffer,buffer,bufferSize);
      if(err != 0){
        Serial.print("sendReceiveSBDText failed: error");
        Serial.println(err);
        nextPing=3600; // set ping interval to short time to retry but go back to normal interval after
        retryCount++;
      }else{
        if(pingInterval>nextPing){ // make sure nextPing needs to be restored in the first place
          if(nextPing == 3600){
            nextPing=pingInterval-(nextPing*retryCount); // return to normal timing
          }else{
            nextPing=pingInterval; // after first run on normal timing reset timer back
            retryCount=0;
          }
        }
        // parse incoming message to get new ping interval
        char incoming[sizeof(buffer)];
        PString incomingMsg(incoming, sizeof(incoming));
        for(int i=0; i<sizeof(buffer); i++){
          incomingMsg.print(buffer[i], HEX);
        }
        int newInterval=0;
        if(sscanf(incomingMsg,"SetInterval: %d",&newInterval) != EOF){
          pingInterval=newInterval;
          nextPing=newInterval;
        }
      }
    }else{
      Serial.print("No GPS fix found!");
      nextPing=3600; // set ping interval to short time to retry but go back to normal interval after
      retryCount++;
    }
  }

  // Sleep
  Serial.println("Going to sleep mode...");
  isbd.sleep();
  ssIridium.end();
  ssGPS.end();
  int elapsedSeconds = (int)((millis() - loopStartTime) / 1000);
  while (elapsedSeconds++ < nextPing)
    bigSleep(nextPing-elapsedSeconds);
}

// Sleep stuff
SIGNAL(WDT_vect) {
  wdt_disable();
  wdt_reset();
  WDTCSR &= ~_BV(WDIE);
}

void babySleep(uint8_t wdt_period){
  wdt_enable(wdt_period);
  wdt_reset();
  WDTCSR |= _BV(WDIE);
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_mode();
  wdt_disable();
  WDTCSR &= ~_BV(WDIE);
}

void smallSleep(int milliseconds) {
  while (milliseconds >= 8000) { babySleep(WDTO_8S); milliseconds -= 8000; }
  if (milliseconds >= 4000)    { babySleep(WDTO_4S); milliseconds -= 4000; }
  if (milliseconds >= 2000)    { babySleep(WDTO_2S); milliseconds -= 2000; }
  if (milliseconds >= 1000)    { babySleep(WDTO_1S); milliseconds -= 1000; }
  if (milliseconds >= 500)     { babySleep(WDTO_500MS); milliseconds -= 500; }
  if (milliseconds >= 250)     { babySleep(WDTO_250MS); milliseconds -= 250; }
  if (milliseconds >= 125)     { babySleep(WDTO_120MS); milliseconds -= 120; }
  if (milliseconds >= 64)      { babySleep(WDTO_60MS); milliseconds -= 60; }
  if (milliseconds >= 32)      { babySleep(WDTO_30MS); milliseconds -= 30; }
  if (milliseconds >= 16)      { babySleep(WDTO_15MS); milliseconds -= 15; }
}

void bigSleep(int seconds){ // before asked 8 seconds is the highest avr watchdog goes
   while (seconds > 8) { smallSleep(8000); seconds -= 8;  }
   smallSleep(1000 * seconds);
}
