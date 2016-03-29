#include <EEPROM.h>

#include <Adafruit_NeoPixel.h>

#include <SPI.h>

#include "ecc.h"
#include "U8glib.h"


#include <nRF24L01.h>
#include <RF24.h>
#include <RF24_config.h>

#include <dht11.h>

#include <TimeAlarms.h>

#include <Time.h>
#include <TimeLib.h>

#define TIME_HEADER  "T"   // Header tag for serial time sync message
#define TIME_REQUEST  7    // ASCII bell character requests a time sync message 
Adafruit_NeoPixel strip = Adafruit_NeoPixel(15, 4);

RF24 radio(9, 10);
const uint64_t pipes[2] = { 0xF0F0F0F0E1LL, 0xF0F0F0F0D2LL };

dht11 DHT11;
#include <OneWire.h>

#include <DallasTemperature.h>
static FILE uartout = {0} ;
// Data wire is plugged into port 2 on the Arduino
#define ONE_WIRE_BUS 3
#define DHTPIN 2
#define DRAINPUMP_RELAY 5
#define TOPUPVALVE_RELAY 6
#define RO_RUNOFFVALVE_RELAY 7
#define LIGHTS_RELAY 8
#define FLOATSWITCH_1 A0
#define FLOATSWITCH_2 A1
boolean float1_sinking = false;
boolean float2_sinking = false;
int currentFreeMem;
#define DISPLAYSTATE_CLOCK 0
#define DISPLAYSTATE_TEMP 1

#define SYSTEMSTATE_NORMAL 0
#define SYSTEMSTATE_WATERCHANGE_DRAINING 1
#define SYSTEMSTATE_WATERCHANGE_FILLING 2
#define SYSTEMSTATE_TOPPING_UP 3
int roRunOffLength;
int roRunOffAfter;
time_t timeNow;
time_t lastDrainStartedAt;
time_t lastFillStartedAt;
unsigned long roRunOffStartedAt;
float tankTemp;
int roomTemp;
int roomHumidity;
double roomDewPoint;
int currentDisplayState;
int currentSystemState;
boolean runOffOnGoing;
int iterationsLastSecond;
unsigned long lastMillis;
// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature.
DallasTemperature sensors(&oneWire);

// arrays to hold device address
DeviceAddress insideThermometer;

U8GLIB_SH1106_128X64 u8g(U8G_I2C_OPT_NONE);  // I2C / TWI
int programLoopCount;
void setup(void) {
  // start serial port
  currentDisplayState = DISPLAYSTATE_CLOCK;
  currentSystemState = SYSTEMSTATE_NORMAL;
  runOffOnGoing = false;
  roRunOffLength = 15;
  roRunOffAfter = 10;
  Serial.begin(115200);
  fdev_setup_stream (&uartout, uart_putchar, NULL, _FDEV_SETUP_WRITE);
  stdout = &uartout ;
  TWBR = 12;  // I2C -> 400khz


  strip.begin();
  strip.setBrightness(32);
  strip.show(); // Initialize all pixels to 'off'

  setTime(8, 29, 0, 1, 1, 11); // set time to Saturday 8:29:00am Jan 1 2011
  programLoopCount = 0;
  lastMillis = millis();
  Alarm.timerRepeat(1, tickTock);            // timer for every second
  radio.begin();
  radio.openWritingPipe(pipes[0]);
  radio.openReadingPipe(1, pipes[0]);
  radio.startListening();

  setSyncProvider( requestSync);  //set function to call when sync required
  pinMode(FLOATSWITCH_1, INPUT_PULLUP);
  pinMode(FLOATSWITCH_2, INPUT_PULLUP);
  pinMode(DRAINPUMP_RELAY, OUTPUT);
  pinMode(TOPUPVALVE_RELAY, OUTPUT);
  pinMode(RO_RUNOFFVALVE_RELAY, OUTPUT);
  pinMode(LIGHTS_RELAY, OUTPUT);
  digitalWrite(DRAINPUMP_RELAY, LOW);
  digitalWrite(TOPUPVALVE_RELAY, HIGH);
  digitalWrite(RO_RUNOFFVALVE_RELAY, HIGH);
  digitalWrite(LIGHTS_RELAY, LOW);

  Serial.println("Dallas Temperature IC Control Library Demo");
  DHT11.attach(2);
  // locate devices on the bus
  Serial.print("Locating devices...");
  sensors.begin();
  Serial.print("Found ");
  Serial.print(sensors.getDeviceCount(), DEC);
  Serial.println(" devices.");

  // report parasite power requirements
  Serial.print("Parasite power is: ");
  if (sensors.isParasitePowerMode()) Serial.println("ON");
  else Serial.println("OFF");

  // assign address manually.  the addresses below will beed to be changed
  // to valid device addresses on your bus.  device address can be retrieved
  // by using either oneWire.search(deviceAddress) or individually via
  // sensors.getAddress(deviceAddress, index)
  //insideThermometer = { 0x28, 0x1D, 0x39, 0x31, 0x2, 0x0, 0x0, 0xF0 };

  // Method 1:
  // search for devices on the bus and assign based on an index.  ideally,
  // you would do this to initially discover addresses on the bus and then
  // use those addresses and manually assign them (see above) once you know
  // the devices on your bus (and assuming they don't change).
  if (!sensors.getAddress(insideThermometer, 0)) Serial.println("Unable to find address for Device 0");

  // method 2: search()
  // search() looks for the next device. Returns 1 if a new address has been
  // returned. A zero might mean that the bus is shorted, there are no devices,
  // or you have already retrieved all of them.  It might be a good idea to
  // check the CRC to make sure you didn't get garbage.  The order is
  // deterministic. You will always get the same devices in the same order
  //
  // Must be called before search()
  //oneWire.reset_search();
  // assigns the first address found to insideThermometer
  //if (!oneWire.search(insideThermometer)) Serial.println("Unable to find address for insideThermometer");

  // show the addresses we found on the bus


  // set the resolution to 9 bit (Each Dallas/Maxim device is capable of several different resolutions)
  sensors.setResolution(insideThermometer, 9);

  Serial.print("Device 0 Resolution: ");
  Serial.print(sensors.getResolution(insideThermometer), DEC);
  Serial.println();
  u8g.setColorIndex(1);         // pixel on
  // u8g.setRot180();

}
void pixelate(unsigned long millisStep) {
  for (int i = 0; i < strip.numPixels(); i++) {
    strip.setPixelColor(i, Wheel((i + timeNow % 255) & 255));
  }
  strip.show();
}
void draw(unsigned long millisStep) {
  // graphic commands to redraw the complete screen should be placed here
  switch (currentSystemState) {
    case SYSTEMSTATE_NORMAL:
      switch (currentDisplayState) {
        case DISPLAYSTATE_CLOCK:
          displayClock(millisStep, 0);
          break;
        case DISPLAYSTATE_TEMP:
          displayTemp();
          break;
      }
      break;
    case SYSTEMSTATE_WATERCHANGE_DRAINING:
      displayDrainTimer();

      break;
    case SYSTEMSTATE_WATERCHANGE_FILLING:
      if (runOffOnGoing) {
        displayRunOffTimer();
      } else {
        displayFillTimer();
      }
      break;
    case  SYSTEMSTATE_TOPPING_UP:
      if (runOffOnGoing) {
        displayRunOffTimer();
      } else {
        displayTopUpTimer();
      }
      break;
  }
}
void displayRunOffTimer() {
  char rTimer[11];
  u8g.setFont(u8g_font_gdb12r);

  u8g.setFontPosTop();
  u8g.drawStr( 0, 0, F("RO Runoff"));
  drawTicker();
  u8g.setFont(u8g_font_osb21n);


  u8g.setFontPosTop();
  snprintf(rTimer, 11, "%d", round((float(lastMillis - roRunOffStartedAt) / float(roRunOffLength * 1000)) * 100));
  int width = u8g.getStrWidth(rTimer);
  u8g.drawStr( ((u8g.getWidth() - width) / 2), 32, rTimer);
  u8g.setFont(u8g_font_gdb12r);
  u8g.setFontPosTop();
  u8g.drawStr( ((u8g.getWidth() - width) / 2) + width + 2, 34, "%");
}
void displayDrainTimer() {
  u8g.setFont(u8g_font_gdb12r);
  //u8g.setFont(u8g_font_osb21);
  u8g.drawStr( 0, 22, "Draining");
  drawTicker();
}
void displayFillTimer() {

  u8g.setFont(u8g_font_gdb12r);
  //u8g.setFont(u8g_font_osb21);
  u8g.drawStr( 0, 22, "Refilling");
  drawTicker();

}
void displayTopUpTimer() {
  char fTimer[9];
   TimeElements tm;

  breakTime(lastFillStartedAt, tm);
  u8g.setFont(u8g_font_gdb12r);
  u8g.setFontPosTop();
  //u8g.setFont(u8g_font_osb21);
  u8g.drawStr( 0, 0, F("Topping up"));
  u8g.drawStr( 0, 23, F("Begun: "));

   snprintf(fTimer, 11, "%d:%02d:%02d", tm.Hour, tm.Minute, tm.Second);

  u8g.drawStr( u8g.getStrWidth(F("Begun: ")), 23, fTimer);
  u8g.drawStr( 0, 46, F("Timer: "));
  if (timeNow - lastFillStartedAt < 60) { 
    snprintf(fTimer, 11, "%ds", timeNow - lastFillStartedAt);
     u8g.drawStr( u8g.getStrWidth(F("Timer: ")), 46, fTimer); 
  } else {
    int secs = (timeNow - lastFillStartedAt) % 60;
    snprintf(fTimer, 11, "%dm", (timeNow - lastFillStartedAt) / 60);
    u8g.drawStr( u8g.getStrWidth(F("Timer: ")), 46, fTimer);
    int width = u8g.getStrWidth(fTimer);
    snprintf(fTimer, 11, "%ds", (timeNow - lastFillStartedAt) % 60);
    u8g.drawStr( u8g.getStrWidth(F("Timer: "))+width, 46, fTimer);
    // No idea why it's necessary to do this in two steps, sprintf always seems to use 0 for the second %d!?
  }
    
  drawTicker();
}
void drawTicker(void) {

  char ticker[2];
  if (lastMillis / 200 % 6 == 0) {
    ticker[0] = '|';
  } else if (lastMillis / 200 % 6 == 1) {
    ticker[0] = '/';
  } else if (lastMillis / 200 % 6 == 2) {
    ticker[0] = '-';
  } else if (lastMillis / 200 % 6 == 3) {
    ticker[0] = '\\';
  } else if (lastMillis / 200 % 6 == 4) {
    ticker[0] = '|';
  } else if (lastMillis / 200 % 6 == 5) {
    ticker[0] = '/';
  } else if (lastMillis / 200 % 6 == 6) {
    ticker[0] = '-';
  }
  ticker[1] = '\0';

  u8g.setFont(u8g_font_gdb12r);

  u8g.setFontPosTop();
  int width = u8g.getStrWidth(ticker);
  u8g.drawStr( (u8g.getWidth() - 15) + ((15 - width) / 2), 0, ticker);
}
void displayClock(unsigned long millisStep, int offset) {

  char ctimeNow[11];
  TimeElements tm;

  breakTime(timeNow, tm);
  if (lastMillis / 500 % 2 == 0) {

    snprintf(ctimeNow, 11, "%d.%02d.%02d", tm.Hour, tm.Minute, tm.Second);
  } else {
    snprintf(ctimeNow, 11, "%d:%02d:%02d", tm.Hour, tm.Minute, tm.Second);
  }
  //u8g.setFont(u8g_font_unifont);
  //u8g.setFont(u8g_font_gdb12r);
  u8g.setFont(u8g_font_osb21n);

  int width = u8g.getStrWidth(ctimeNow);

  u8g.setFontPosTop();
  u8g.drawStr( ((u8g.getWidth() - width) / 2) + offset, 0, ctimeNow);

  u8g.setFont(u8g_font_gdb12r);
  u8g.setFontPosTop();


  snprintf(ctimeNow, 11, "%s", dayStr(weekday()));

  width = u8g.getStrWidth(ctimeNow);
  u8g.drawStr( ((u8g.getWidth() - width) / 2) - offset, 26, ctimeNow);
  snprintf(ctimeNow, 11, "%d/%d/%d", tm.Day, tm.Month, year());

  width = u8g.getStrWidth(ctimeNow);
  u8g.drawStr( ((u8g.getWidth() - width) / 2) + offset, 48, ctimeNow);

}

void displayTemp(void) {
  u8g.setFont(u8g_font_gdb12r);
  //u8g.setFont(u8g_font_osb21);
  u8g.drawStr( 0, 22, "Hello World!");


}
void updateTankTemp(void) {
  sensors.requestTemperatures(); // Send the command to get temperatures
  tankTemp = sensors.getTempC(insideThermometer);

}
void updateDHTVals(boolean dodew = false) {

  int chk = DHT11.read();


  switch (chk)
  {
    case 0: break;
    case -1: Serial.println(F("Checksum error")); break;
    case -2: Serial.println(F("Time out error")); break;
    default: Serial.println(F("Unknown error")); break;
  }
  roomTemp = DHT11.temperature;
  roomHumidity = DHT11.humidity;
  if (dodew) {
    roomDewPoint = DHT11.dewPoint();
  }

}
void updateState(unsigned long millisStep) {

  timeNow = now();
  boolean float1_already_sinking = float1_sinking;
  boolean float2_already_sinking = float2_sinking;
  float1_sinking = digitalRead(FLOATSWITCH_1);
  float2_sinking = digitalRead(FLOATSWITCH_2);



  // trigger events after updating all state
  if (float1_already_sinking != float1_sinking && float1_sinking) {
    float1_dropped();
  } else if (float1_already_sinking != float1_sinking) {
    float1_risen();
  }
  if (float2_already_sinking != float2_sinking && float2_sinking) {
    float2_dropped();
  } else if (float2_already_sinking != float2_sinking) {
    float2_risen();
  }
}
void float1_dropped(void) {

  Serial.println(F("float 1 dropped"));
  if (currentSystemState == SYSTEMSTATE_NORMAL) {
    currentSystemState = SYSTEMSTATE_TOPPING_UP;
    beginFillingTank();
  } else if (currentSystemState == SYSTEMSTATE_WATERCHANGE_DRAINING) {
    stopDrainingTank();
    currentSystemState = SYSTEMSTATE_WATERCHANGE_FILLING;
    beginFillingTank();
  } else {
    errorState();
  }
}
void float1_risen(void) {
  Serial.println(F("float 1 risen"));
  if (currentSystemState == SYSTEMSTATE_TOPPING_UP || currentSystemState == SYSTEMSTATE_WATERCHANGE_FILLING) {
    stopFillingTank();
    currentSystemState = SYSTEMSTATE_NORMAL;
  } else {
    errorState();
  }

}
void float2_dropped(void) {

  Serial.println(F("float 2 dropped"));
}
void float2_risen(void) {
  Serial.println(F("float 2 risen"));

}
void stopDrainingTank() {
  Serial.println(F("tank drain stopped"));
}

void stopFillingTank() {
  Serial.println(F("tank fill stopped"));
  digitalWrite(TOPUPVALVE_RELAY, HIGH);
  digitalWrite(RO_RUNOFFVALVE_RELAY, HIGH);
}
void beginFillingTank() {
  Serial.println(F("begin filling tank"));
  digitalWrite(TOPUPVALVE_RELAY, LOW);
  digitalWrite(RO_RUNOFFVALVE_RELAY, HIGH);
  lastFillStartedAt = timeNow;
  Alarm.timerOnce(roRunOffAfter, beginRoRunOff);
}
void beginRoRunOff() {
  Serial.println(F("begin ro runoff"));
  if (!runOffOnGoing && currentSystemState == SYSTEMSTATE_TOPPING_UP || currentSystemState == SYSTEMSTATE_WATERCHANGE_FILLING) {
    digitalWrite(RO_RUNOFFVALVE_RELAY, LOW);
    roRunOffStartedAt = lastMillis;
    runOffOnGoing = true;
    Alarm.timerOnce(roRunOffLength, stopRoRunOff);
  } else {
    digitalWrite(RO_RUNOFFVALVE_RELAY, LOW);
    runOffOnGoing = false;
    errorState();
  }

}
void stopRoRunOff() {
  Serial.println(F("stop ro runoff"));
  digitalWrite(RO_RUNOFFVALVE_RELAY, HIGH);
  runOffOnGoing = false;
  if (currentSystemState == SYSTEMSTATE_TOPPING_UP || currentSystemState == SYSTEMSTATE_WATERCHANGE_FILLING) {
    Alarm.timerOnce(roRunOffAfter, beginRoRunOff);
  }
}
void errorState() {
  Serial.println(F("error state"));

}
void tickTock(void) {
  iterationsLastSecond = programLoopCount;
  programLoopCount = 0;
  updateTankTemp();
  currentFreeMem = freeRam();
  if (timeNow % 5 == 0) {
    updateDHTVals(timeNow % 15 == 0); // only do dew point calc every 15 secs
  }
  //radio.printDetails();
}

void loop(void) {


  unsigned long newMillis = millis();
  unsigned long millisStep = lastMillis - newMillis;
  lastMillis = newMillis;
  programLoopCount++;

  updateState(millisStep);
  if (Serial.available()) {
    processSyncMessage();
  }
  if ( radio.available() )
  {
    processRFMessage();

  }


  u8g.firstPage();
  do {
    draw(millisStep);
  } while ( u8g.nextPage() );

  pixelate(millisStep);
  // rebuild the picture after some delay
  Alarm.delay(1);

}
void processRFMessage() {
  // Dump the payloads until we've gotten everything
  unsigned int pbyte;
  boolean done = false;
  while (!done)
  {
    // Fetch the payload, and see if this was the last one.
    done = radio.read( &pbyte, sizeof(unsigned int) );

    // Spew it
    printf("Got payload %c...", pbyte);

    // Delay just a little bit to let the other unit
    // make the transition to receiver

  }

}
void processSyncMessage() {
  unsigned long pctime;
  const unsigned long DEFAULT_TIME = 1357041600; // Jan 1 2013

  if (Serial.find(TIME_HEADER)) {
    pctime = Serial.parseInt();
    if ( pctime >= DEFAULT_TIME) { // check the integer is a valid time (greater than Jan 1 2013)
      setTime(pctime); // Sync Arduino clock to the time received on the serial port
    }
  }
}

time_t requestSync()
{
  Serial.write(TIME_REQUEST);
  return 0; // the time will be sent later in response to serial mesg
}

int freeRam () {
  extern int __heap_start, *__brkval;
  int v;
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
}


// create a output function
// This works because Serial.write, although of
// type virtual, already exists.
static int uart_putchar (char c, FILE *stream)
{
  Serial.write(c) ;
  return 0 ;
}

int random_bytes(unsigned char *buffer, size_t len) {



}


// Fill the dots one after the other with a color
void colorWipe(uint32_t c, uint8_t wait) {
  for (uint16_t i = 0; i < strip.numPixels(); i++) {
    strip.setPixelColor(i, c);
    strip.show();
    delay(wait);
  }
}

void rainbow(uint8_t wait) {
  uint16_t i, j;

  for (j = 0; j < 256; j++) {
    for (i = 0; i < strip.numPixels(); i++) {
      strip.setPixelColor(i, Wheel((i + j) & 255));
    }
    strip.show();
    delay(wait);
  }
}

// Slightly different, this makes the rainbow equally distributed throughout
void rainbowCycle(uint8_t wait) {
  uint16_t i, j;

  for (j = 0; j < 256 * 5; j++) { // 5 cycles of all colors on wheel
    for (i = 0; i < strip.numPixels(); i++) {
      strip.setPixelColor(i, Wheel(((i * 256 / strip.numPixels()) + j) & 255));
    }
    strip.show();
    delay(wait);
  }
}

//Theatre-style crawling lights.
void theaterChase(uint32_t c, uint8_t wait) {
  for (int j = 0; j < 10; j++) { //do 10 cycles of chasing
    for (int q = 0; q < 3; q++) {
      for (int i = 0; i < strip.numPixels(); i = i + 3) {
        strip.setPixelColor(i + q, c);  //turn every third pixel on
      }
      strip.show();

      delay(wait);

      for (int i = 0; i < strip.numPixels(); i = i + 3) {
        strip.setPixelColor(i + q, 0);      //turn every third pixel off
      }
    }
  }
}

//Theatre-style crawling lights with rainbow effect
void theaterChaseRainbow(uint8_t wait) {
  for (int j = 0; j < 256; j++) {   // cycle all 256 colors in the wheel
    for (int q = 0; q < 3; q++) {
      for (int i = 0; i < strip.numPixels(); i = i + 3) {
        strip.setPixelColor(i + q, Wheel( (i + j) % 255)); //turn every third pixel on
      }
      strip.show();

      delay(wait);

      for (int i = 0; i < strip.numPixels(); i = i + 3) {
        strip.setPixelColor(i + q, 0);      //turn every third pixel off
      }
    }
  }
}

// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
uint32_t Wheel(byte WheelPos) {
  WheelPos = 255 - WheelPos;
  if (WheelPos < 85) {
    return strip.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  }
  if (WheelPos < 170) {
    WheelPos -= 85;
    return strip.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
  WheelPos -= 170;
  return strip.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
}
