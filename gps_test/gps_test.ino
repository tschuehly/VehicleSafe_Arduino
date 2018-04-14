#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
#include <SoftwareSerial.h>

//Serial config

#define pi_tx 8;
#define pi_rx 7;
byte number = 0;
int times;
int sectimes;

int longitude = 0;
int latitude = 0;
float lonmed = 0;
float latmed = 0;
float variance = -1;
float maxvariance = -1;
int fix;
SoftwareSerial piS(7,8);
// Connect the GPS Power pin to 5V
// Connect the GPS Ground pin to ground
//   Connect the GPS TX (transmit) pin to Digital 3
//   Connect the GPS RX (receive) pin to Digital 2
SoftwareSerial mySerial(3, 2);


Adafruit_GPS GPS(&mySerial);

#define GPSECHO  false

// this keeps track of whether we're using the interrupt
// off by default!
boolean usingInterrupt = false;
void useInterrupt(boolean); // Func prototype keeps Arduino 0023 happy

void setup()
{
  //connect to Raspberry
  piS.begin(115200);
  delay(500);
  // connect at 115200 so we can read the GPS fast enough and echo without dropping chars
  // also spit it out
  Serial.begin(115200);
  Serial.println("Adafruit GPS library basic test!");

  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPS.begin(9600);

  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);

  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate

  // Request updates on antenna status, comment out to keep quiet
  GPS.sendCommand(PGCMD_ANTENNA);

  // the nice thing about this code is you can have a timer0 interrupt go off
  // every 1 millisecond, and read data from the GPS for you. that makes the
  // loop code a heck of a lot easier!
  useInterrupt(true);

  delay(1000);
  // Ask for firmware version
  mySerial.println(PMTK_Q_RELEASE);


  times = 1;
}


// Interrupt is called once a millisecond, looks for any new GPS data, and stores it
SIGNAL(TIMER0_COMPA_vect) {
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
#ifdef UDR0
  if (GPSECHO)
    if (c) UDR0 = c;
    // writing direct to UDR0 is much much faster than Serial.print
    // but only one character can be written at a time.
#endif
}

void useInterrupt(boolean v) {
  if (v) {
    // Timer0 is already used for millis() - we'll just interrupt somewhere
    // in the middle and call the "Compare A" function above
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
    usingInterrupt = true;
  } else {
    // do not call the interrupt function COMPA anymore
    TIMSK0 &= ~_BV(OCIE0A);
    usingInterrupt = false;
  }
}

uint32_t timer = millis();
            //long,lati,speed
float gps[] = {0,0,0,0};

void lp()
{

  if (GPS.newNMEAreceived()) {
    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another
  }

  // if millis() or timer wraps around, we'll just reset it
  if (timer > millis())  timer = millis();

  // approximately every 2 seconds or so, print out the current stats
  if (millis() - timer > 2000) {
    timer = millis(); // reset the timer

    Serial.print("\nTime: ");
    Serial.print(GPS.hour, DEC); Serial.print(':');
    Serial.print(GPS.minute, DEC); Serial.print(':');
    Serial.print(GPS.seconds, DEC); Serial.print('.');
    Serial.println(GPS.milliseconds);
    Serial.print("Date: ");
    Serial.print(GPS.day, DEC); Serial.print('/');
    Serial.print(GPS.month, DEC); Serial.print("/20");
    Serial.println(GPS.year, DEC);
    Serial.print("Fix: "); Serial.print((int)GPS.fix);
    Serial.print(" quality: "); Serial.println((int)GPS.fixquality);
    if (GPS.fix) {
      Serial.print("Location: ");
      Serial.print(GPS.latitude, 4); Serial.print(GPS.lat);
      Serial.print(", ");
      Serial.print(GPS.longitude, 4); Serial.println(GPS.lon);
      Serial.print("Location (in degrees, works with Google Maps): ");
      Serial.print(GPS.latitudeDegrees, 10);
      Serial.print(", ");
      Serial.println(GPS.longitudeDegrees, 10);

      Serial.print("Speed (knots): "); Serial.println(GPS.speed);
      Serial.print("Angle: "); Serial.println(GPS.angle);
      Serial.print("Altitude: "); Serial.println(GPS.altitude);
      Serial.print("Satellites: "); Serial.println((int)GPS.satellites);

      // Da die string() funktion die float abschneidet muss man eine andere Funktion verwenden
      char buff[10];
      String latitude;
      String longitude;
      dtostrf(GPS.latitudeDegrees, 4, 6, buff);
      latitude = buff;
      dtostrf(GPS.longitudeDegrees, 4, 6, buff);
      longitude = buff;

      String csvdata = "100002," +String(GPS.hour)+","+ String(GPS.minute) + ","+String(GPS.seconds) + ","+ String(GPS.day) + ","+String(GPS.month) + ","+String(GPS.year)+","+String(GPS.fix)+ ","+String(GPS.fixquality)+ ","+ latitude +","+ longitude +","+String(GPS.speed)+","+String(GPS.angle)+"," +String(GPS.altitude)+","+ String(GPS.satellites);
      piS.print(csvdata);
      Serial.print("csv : ");Serial.println(csvdata);
    }else{
      //char csvdata = "100002 , " +GPS.hour + GPS.minute + GPS.seconds + ","+ GPS.day + ","+GPS.month + ","+GPS.year+","+GPS.fix+ ","+GPS.fixquality+ ",0,0,0,0,0,0";

    }
    }
}



int gps_moving(){

  if (GPS.newNMEAreceived()) {
    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      gps_moving();  // we can fail to parse a sentence in which case we should just wait for another
  }

  // if millis() or timer wraps around, we'll just reset it
  if (timer > millis())  timer = millis();

  // approximately every 2 seconds or so, print out the current stats
  if (millis() - timer > 2000) {
    timer = millis(); // reset the timer

    Serial.print("\nTime: ");
    Serial.print(GPS.hour, DEC); Serial.print(':');
    Serial.print(GPS.minute, DEC); Serial.print(':');
    Serial.print(GPS.seconds, DEC); Serial.print('.');
    Serial.println(GPS.milliseconds);
    Serial.print("Date: ");
    Serial.print(GPS.day, DEC); Serial.print('/');
    Serial.print(GPS.month, DEC); Serial.print("/20");
    Serial.println(GPS.year, DEC);
    Serial.print("Fix: "); Serial.print((int)GPS.fix);
    Serial.print(" quality: "); Serial.println((int)GPS.fixquality);
    if (GPS.fix) {;
      Serial.print(GPS.latitudeDegrees,10);
      Serial.print(", ");
      Serial.println(GPS.longitudeDegrees,10);

      Serial.print("Speed (knots): "); Serial.println(GPS.speed);
      times = times +1;
      if(times>2 && times <8){

        lonmed = lonmed + GPS.longitudeDegrees;
        Serial.println(lonmed,10);
        latmed = latmed + GPS.latitudeDegrees;
        Serial.println(latmed,10);
      }
      if(times>8){
        variance =  lonmed - GPS.longitudeDegrees;
        if(variance < 0){
                  variance = variance * -1;
                  Serial.println("negative Varianz");
        } 
        if(variance > maxvariance){
          maxvariance = variance;
        }
        Serial.print("Die Varianz ist: ");Serial.println(variance,10);
        if(variance > 0.01){
          Serial.println("Das Fahrzeug wurde bewegt");
        }
      }
    }else{
      Serial.println("NoFix");
    }

    }
  return times;
}

void loop(){
  gps_moving();
  if(times != sectimes){
    Serial.println(times);
    sectimes = times;
    if(times == 7){
    lonmed = lonmed / 5;
    latmed = latmed / 5;
    Serial.print("Average over 5 longitude: ");Serial.println(lonmed,10);
    Serial.print("Average over 5 latitude : ");Serial.println(latmed,10);
  }
    Serial.print("Die maximal Varianz ist :");Serial.println(maxvariance,10);
  }



  int longitude = 0;
  int latitude = 0;
  int fix;
}
