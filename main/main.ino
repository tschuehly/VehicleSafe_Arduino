
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
#include <SoftwareSerial.h>
#include <TimeLib.h>
#include <LowPower.h>
// Variablen
#define pi_tx 8;
#define pi_rx 7;
byte number = 0;
int times;
int sectimes;
int schluessel = 9;
int fix;
int longitude = 0;
int latitude = 0;
float lonmed = 0;
float latmed = 0;
float lonvariance = -1;
float latvariance = -1;
float maxvariance = -1;
int movcount = 0;

const int wakeUpPin = 2;
int source = 11;
volatile int value = 0;
int sleeper = 5;
int val = 0;
int schalter = 12;
int schalter_status = 0;
int gps_an = 13;
int rasp_an = 6;
int timer_speed = 2000;
//Raspberry Serial Connection
// Connect Pin 8 to GPIO 15
// Connect Pin 7 to GPIO 14
SoftwareSerial piS(7,8);
// Connect the GPS Power pin to 5V
// Connect the GPS Ground pin to ground
// Connect the GPS TX (transmit) pin to Digital 10
// Connect the GPS RX (receive) pin to Digital 9
SoftwareSerial mySerial(10,9);


Adafruit_GPS GPS(&mySerial);

#define GPSECHO  false

// this keeps track of whether we're using the interrupt
// off by default!
boolean usingInterrupt = false;
void useInterrupt(boolean); // Func prototype keeps Arduino 0023 happy
time_t t;

void setup()
{
  pinMode(wakeUpPin, INPUT); 
  pinMode(schalter,INPUT_PULLUP);  
  pinMode(source,OUTPUT);
  pinMode(rasp_an,OUTPUT);
  pinMode(gps_an,OUTPUT);
  pinMode(sleeper,INPUT);
  digitalWrite(source,HIGH);
  digitalWrite(rasp_an,HIGH );
  
  Serial.println("Not awake");
  //connect to Raspberry
  piS.begin(115200);
  delay(500);
  // connect at 115200 so we can read the GPS fast enough and echo without dropping chars
  Serial.begin(115200);
  Serial.println("VehicleSafe main test");

  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPS.begin(9600);

  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);

  // Set the update rate
  //GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
  GPS.sendCommand("$PMTK220,2000*1C");
  // Request updates on antenna status, comment out to keep quiet
  //GPS.sendCommand(PGCMD_ANTENNA);

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

// Hauptfunktion um das GPS abzufragen

void check_GPS(int mode)
{
  if (GPS.newNMEAreceived()) {
    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      check_GPS(1);  // we can fail to parse a sentence in which case we should just wait for another
  }

  // if millis() or timer wraps around, we'll just reset it
  if (timer > millis())  timer = millis();

  // approximately every 2 seconds or so, print out the current stats
  if (millis() - timer > timer_speed) {
    timer = millis(); // reset the timer
    if (GPS.fix) {
      // Da die string() funktion die float abschneidet muss man eine andere Funktion verwenden
      char buff[10];
      String latitude;
      String longitude;
      dtostrf(GPS.latitudeDegrees, 4, 6, buff);
      latitude = buff;
      dtostrf(GPS.longitudeDegrees, 4, 6, buff);
      longitude = buff;
      
      if(mode == 1){                                                                             // Der Modus 1 schickt die Daten direkt an den Raspberry
        if(times>2 && times <8){                                                                 // Die ersten  5 Gps Standorte werden zusammengezählt
          lonmed = lonmed + GPS.longitudeDegrees;
          Serial.println(lonmed,10);
          latmed = latmed + GPS.latitudeDegrees;
          Serial.println(latmed,10);
        }
        
        if(times == 7){                                                                            //Dann wird ein Durchschnitt errechnet
          lonmed = lonmed / 5;
          latmed = latmed / 5;
          Serial.print("Average over 5 longitude: ");Serial.println(lonmed,10);
          Serial.print("Average over 5 latitude : ");Serial.println(latmed,10);
        }
  
        if(times>8){                                                                      //Jetzt wird die Varianz von diesem Durchschnitt kontinuierlich gecheckt. 
          timer_speed = 5000;
          lonvariance =  lonmed - GPS.longitudeDegrees;
          latvariance = latmed - GPS.latitudeDegrees;
          
          if(lonvariance < 0){
                    lonvariance = lonvariance * -1;
          } 
          if(latvariance < 0){
                    latvariance = latvariance * -1;
          } 

          
          if(latvariance > 0.001 || lonvariance > 0.001){                                            //Wenn die Varianz größer als 0.001 ist was ~60m entspricht
            movcount = movcount + 1;
          }
          Serial.print("lonvariance: ");Serial.println(lonvariance,10);
          Serial.print("latvariance: ");Serial.println(latvariance,10); 
        }
        if(times>38){ 
          if(movcount >2){
            digitalWrite(rasp_an,HIGH); //Raspberry anschalten
        }
        
        }
      }

      times = times +1;
      int curhr = GPS.hour;
      int curmin = GPS.minute;
      int cursec = GPS.seconds;
      int curday = GPS.day;
      int curmonth = GPS.month;
      int curyear = GPS.year;
      setTime(curhr,curmin,cursec,curday,curmonth,curyear);
      t = now();
      String timenow = String(t);
      Serial.println(timenow);
      String serdata = "{'command':'sendbroadcast','acc_id':'1','device_id': '100001', 'timestamp':'" + timenow + "','lock_mode':'"+schalter_status+"','latitude':'"+ latitude + "','longitude':'" + longitude +"','altitude':'"+String(GPS.altitude)+"','speed':'"+String(GPS.speed)+"','fix':'"+String(GPS.fix)+"','fix_quality':'"+String(GPS.fixquality)+"'}";
      piS.print(serdata);
      Serial.print("Data : ");Serial.println(serdata);
      
    }else{
      Serial.println("NoFix");
      Serial.println(times);
      //char csvdata = "100002 , " +GPS.hour + GPS.minute + GPS.seconds + ","+ GPS.day + ","+GPS.month + ","+GPS.year+","+GPS.fix+ ","+GPS.fixquality+ ",0,0,0,0,0,0";
    }
    }
}

void sleepNow()       
{
    LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF); 
    // Disable external pin interrupt on wake up pin.
    detachInterrupt(wakeUpPin); 
}
void wakeUp()
{

}
void loop(){
  // Allow wake up pin to trigger interrupt on high.
  attachInterrupt(digitalPinToInterrupt(wakeUpPin), wakeUp, CHANGE);
  val = digitalRead(sleeper);
  if(val == HIGH){
    digitalWrite(gps_an,LOW);  
    sleepNow();
    times = 1; 
    digitalWrite(gps_an,HIGH);    
  } 
  
  if(digitalRead(schalter) == HIGH){

    schalter_status = 0; // zugeschlossen
  }else{

    schalter_status = 1; // aufgeschlossen
  }
  //Serial.println(digitalRead(schluessel));
  check_GPS(1);
  if(times != sectimes){ //Wird ein neuer Datensatz erhalten?
    Serial.println(times);
    sectimes = times;
}
}
