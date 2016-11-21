#include <LiquidCrystal.h>
#include <EEPROM.h>
#include <dht11.h>
#include <DS1302RTC.h>
#include <Streaming.h>
#include <Time.h>
#include <TimeLib.h>

LiquidCrystal lcd(8, 9, 4, 5, 6, 7);           // select the pins used on the LCD panel
dht11 DHT11;
// Set pins:  CE, IO,CLK
DS1302RTC RTC(53, 51, 49);

// Optional connection for RTC module
#define ramFirstAddress 192
#define FF 255

time_t t;
tmElements_t tm;
char time[20];
int i;
char buf [8];
unsigned long delayTime;
char ramArray[31];


//declare variables
int dispMode = 0;
int cTemp = 0;
int cHum = 0;
int sTemp = 75;
String aMode = ""; //0 = off, 2 = heat, 1 = cool
int mode = 0;
int fanMode = 0; //0 = auto, 1 = manual
String sfanMode = "";
int runMode = 0; //0 = Auto, 1 = Manual, 2 = Temporary Override
String sRunMode = "";
String sTime = "";
String hum = "100";
long timeSelectPress = 0;
long timeRightPress = 0;
long timeLeftPress = 0;
long timeUpPress = 0;
long timeDownPress = 0;
long debounce = 250; //milliseconds that the button must be held to cause an action
long lastTempPoleTime = 0; //milliseconds since the temperature was last polled
long tempPoleTime = 10000; //the amount of time to wait between temperature polls
int setDayTime = 0; //the time of the day from early morning to evening
String sSetDayTime = "";
String sSetDay = "";
int minTemp = 55; //min temp that the user can set
int maxTemp = 90; //max temp that the user can set
boolean heating = false; //is there currently a standing call for heat?
int tVariance = 2; //the amount of temperature deviation allowed before starting the cycle
int tVarienceCutOut = 3; //the amount of temperature deviation allowed when ending the cylcle
int timeHour = 0;
String sTimeHour = "";
int timeMinute = 0;
String sTimeMinute = "";
int dayOfWeek = 0;
String sDayOfWeek = "";
int timeOfDay = 0;
long EEPROMUpdatePeriod = 3600000; //update the EEPROM every hour if there have been changes to the data
long lastEEPROMUpdateTime = 0;
long resetPeriod = 2592000000; //the time in milliseconds to perform a soft reset 2592000000 is 30 days
long timeOverrideEnabled = 0; //a variable to hold the current time in millis that the override was enabled
long allowedOverrideTime = 3600000; //time in milliseconds that the system will stay in override mode
long cycleStartTime = 0; //the time in milliseconds that the cycle started
long cycleEndTime = 0; //the time in milliseconds that the cycle ended
long totalCycleTime = 0; //the total amount of cycle time since the last reset
long averageCycleTime = 0; //the average cycle time since last reset
int totalCycles = 0; //the total cycles since the last reset
int sensorCalibration = 0; //the callibration value for the temperature sensor

//variables for the temperature sensor
int dht11Chk = 0;
int dht11Gnd = 29;
int dht11Vcc = 23;
int dht11Data = 25;

//variables for control relays
int fanRelayPin = 50;
int heatRelayPin = 52;
int coolRelayPin = 48;

//array to hold the set temperatures 0 = earlyMorning, 1 = lateMorning, 2 = afternoon, 3 = evening, 4 = night
//earlyMorning = 04:31 - 08:00
//lateMorning = 08:01 - 11:59
//afternoon = 12:00 - 16:30
//evening = 16:31 - 23:00
//night = 23:01 - 04:30 

int setTemps[8][5] = {
  //fill the the first element of the array, not used
  {55, 55, 55, 55, 55 },
  //sunday default temperatures
  {55, 56, 57, 58, 59 },
  //monday default temperatures
  {55, 55, 55, 55, 55 },
  //tuesday default temperatures
  {55, 55, 55, 55, 55 },
  //wednesday default temperatures
  {55, 55, 55, 55, 55 },
  //thursday defualt temperatures
  {55, 55, 55, 55, 55 },
  //friday defualt temperatures
  {55, 55, 55, 55, 55 },
  //saturday default temperatures
  {55, 55, 55, 55, 55 }
};

//declare variables for mode and fan control

boolean overrideSel = false;
boolean fanModeAuto = true;

// define some values used by the panel and buttons
int lcd_key     = 0;
int adc_key_in  = 0;

#define btnRIGHT  0
#define btnUP     1
#define btnDOWN   2
#define btnLEFT   3
#define btnSELECT 4
#define btnNONE   5

//display mode keys
//Main = 0
//Sunday = 1
//Monday = 2
//Tuesday = 3
//Wednesday = 4
//Thursday = 5
//Friday = 6
//Saturday = 7


int read_LCD_buttons(){               // read the buttons
    adc_key_in = analogRead(0);       // read the value from the sensor 

    // my buttons when read are centered at these valies: 0, 144, 329, 504, 741
    // we add approx 50 to those values and check to see if we are close
    // We make this the 1st option for speed reasons since it will be the most likely result

    if (adc_key_in > 1000) return btnNONE; 

    // For V1.1 us this threshold
    if (adc_key_in < 50)   return btnRIGHT;  
    if (adc_key_in < 195)  return btnUP; 
    if (adc_key_in < 380)  return btnDOWN; 
    if (adc_key_in < 555)  return btnLEFT; 
    if (adc_key_in < 790)  return btnSELECT;  

    return btnNONE;                // when all others fail, return this.
}

void setup(){

    Serial.begin(9600);
    delay(100);
    
    if (RTC.haltRTC()) {
      Serial << "The DS1302 is stopped." << endl << "Please, set the time by typing \"SET TIME\".";
      Serial << endl << endl;
    }    
    //setSyncProvider() causes the Time library to synchronize with the
    //external RTC by calling RTC.get() every five minutes by default.
    setSyncProvider(RTC.get); 
    printMenu();


    
    //set up for the dht11 temperature sensor
    pinMode(dht11Gnd, OUTPUT);
    digitalWrite(dht11Gnd, LOW);
    pinMode(dht11Vcc, OUTPUT);
    digitalWrite(dht11Vcc, HIGH);
    DHT11.attach(dht11Data);

    //set up for the control relay pins
    pinMode(fanRelayPin, OUTPUT);
    digitalWrite(fanRelayPin, HIGH);
    pinMode(heatRelayPin, OUTPUT);
    digitalWrite(heatRelayPin, HIGH);
    pinMode(coolRelayPin, OUTPUT);
    digitalWrite(coolRelayPin, HIGH);
    
    //set up for the lcd screen
   lcd.begin(16, 2);               // start the library
   lcd.setCursor(0,0);             // set the LCD cursor   position 
   lcd.print("Hello World! :)");  // print a simple message on the LCD
   lcd.setCursor(0,1);
   lcd.print("I'm Thermostat");
   delay(2000);
   lcd.clear();

   //EEPROM.write(2, sensorCalibration + 128);
   //delay(10);
   //read the data contained in EEPROM to variables
   readDataFromEEPROM();

   //take an initial reading of the temperature sensor
   dht11Chk = DHT11.read();
   cTemp = DHT11.fahrenheit();
   cHum = DHT11.humidity;
   lastTempPoleTime = millis();
}
 
void loop(){
   //get the temperature from the dht11 temperature sensor
   if(millis() - lastTempPoleTime > tempPoleTime)
   {
    dht11Chk = DHT11.read();
    cTemp = DHT11.fahrenheit();
    cHum = DHT11.humidity;
    lastTempPoleTime = millis();
    cTemp = cTemp + sensorCalibration;
   }
   
   //read the buttons on the lcd panel
   lcd_key = read_LCD_buttons();   // read the buttons

   switch (lcd_key){               // depending on which button was pushed, we perform an action

       case btnRIGHT:{
        if (millis() - timeRightPress > debounce)
        {
          if (dispMode == 8)
          {
            //call the function to write data to EEPROM
            writeDataToEEPROM();
          }
          else if (dispMode == 0)
          {
            //change the mode
            if (mode < 2)
            {
              mode++;  
            }
            else
            {
              mode = 0;
            }
          }
          
          timeRightPress = millis();
        }
            break;
       }



       
       case btnLEFT:{
        //switch the set day if in the correct display modes
        if (millis() - timeLeftPress > debounce)
        {
          if (dispMode > 0 && dispMode <= 7)
          {
            //increse the current set day
            if (setDayTime < 4)
            {
              setDayTime ++;
            }
            else
            {
              setDayTime = 0;
            }
          }
          else if (dispMode == 10)
          {
            //change the mode
            if (fanMode == 0)
            {
              fanMode = 1; 
            }
            else
            {
              fanMode = 0;
            }
          }

          else if (dispMode == 0)
          {
            //change the mode
            if (runMode == 0)
            {
              runMode = 1; 
            }
            else
            {
              runMode = 0;
            }
          }
          timeLeftPress = millis();
        }
        
             break;
       }    




       case btnUP:{
        //if statements to determine the display mode
        if (millis() - timeUpPress > debounce)
        {
          if (dispMode == 0)
          {
            if (runMode == 0)
            {
              runMode = 2;
              timeOverrideEnabled = millis();
            }
            
            if (sTemp <= maxTemp)
            {
              sTemp++; 
            }
            
          }
          else if (dispMode > 0 && dispMode <= 7)
          {
            if (setTemps[dispMode][setDayTime] <= maxTemp)
            {
              setTemps[dispMode][setDayTime]++;
            }
          }
          else if (dispMode == 12)
          {
            sensorCalibration++;
          }

          timeUpPress = millis();
        }
        
             break;
       }



       
       case btnDOWN:{
        //if statements to determine the display mode
        if (millis() - timeDownPress > debounce)
        {
          if (dispMode == 0)
          {
            if (runMode == 0)
            {
              runMode = 2;
              timeOverrideEnabled = millis();
            }
            
            if (sTemp >= minTemp)
            {
              sTemp--;
            }
          }
          else if (dispMode > 0 && dispMode <= 7)
          {
            if (setTemps[dispMode][setDayTime] >= minTemp)
            {
              setTemps[dispMode][setDayTime]--;
            }
          }
           else if (dispMode == 12)
          {
            sensorCalibration--;
          }
          timeDownPress = millis();
        }
             break;
       }



       
       case btnSELECT:{
        //check if enough time has elapesed to change the display mode
          if (millis() - timeSelectPress > debounce)
          {
               //change the display mode
               if (dispMode <= 11)
               {
                  dispMode++;
               }
               else
               {
                  dispMode = 0;
               }
               timeSelectPress = millis();
               //clear what is currently on the screen
               lcd.clear();
               //change the setDay back to 0 or sunday
               setDayTime = 0;
                       
          }
          break;
       }



       
       case btnNONE:{
             break;
       }
   }

  //update the time string
  t = now();
  updateStringTime(t);
  
  //switch the display mode
  switch (dispMode){
    //case for main display mode
    case 0:{
      //show an error on the screen if there is a sensor issue
      if (dht11Chk == 0)
      {
        //print display information for the first display
        lcd.setCursor(0,0);
        lcd.print("C");
        lcd.setCursor(1,0);
        lcd.print(cTemp);
        lcd.setCursor(3,0);
        lcd.print("F S");
        lcd.setCursor(6,0);
        lcd.print(sTemp);
        lcd.setCursor(8,0);
        lcd.print("F");
        lcd.setCursor(12,0);
        lcd.print(aMode);
        lcd.setCursor(0,1);
        lcd.print(sTimeHour);
        lcd.setCursor(2,1);
        lcd.print(":");
        lcd.setCursor(3,1);
        lcd.print(sTimeMinute);
        lcd.setCursor(5,1);
        lcd.print(sDayOfWeek);
        lcd.setCursor(9,1);
        lcd.print(sfanMode);
        lcd.setCursor(12,1);
        lcd.print(sRunMode);
      }
      else
      {
        //tell the user that there is a temperature read error and the the thermostat will operate in a timed run mode
        lcd.setCursor(0,0);
        lcd.print("Error Reading   ");
        lcd.setCursor(0,1);
        lcd.print("Temperature!    ");
      }
      break;
    }
    //case for sunday
    case 1:{
      sSetDay = "Sun";
      printDayLCD();

      break;
    }
    //case for monday
    case 2:{
      sSetDay = "Mon";
      printDayLCD();

      break;
    }
    //case for tuesday
    case 3:{
      sSetDay = "Tue";
      printDayLCD();

      break;
    }
    //case for wednesday
    case 4:{
      sSetDay = "Wed";
      printDayLCD();

      break;
    }
    //case for thursday
    case 5:{
      sSetDay = "Thu";
      printDayLCD();

      break;
    }
    //case for friday
    case 6:{
      sSetDay = "Fri";
      printDayLCD();

      break;
    }
    //case for saturday
    case 7:{
      sSetDay = "Sat";
      printDayLCD();

      break;
    }
    case 8:{
      //this case is for writing data to the eeprom
      lcd.setCursor(0,0);
      lcd.print("Save to EEPROM?");
      lcd.setCursor(0,1);
      lcd.print("Press Right");
      break;
    }
    case 9:{
      //this display shows the humidity
      lcd.setCursor(0,0);
      lcd.print("Humidity");
      lcd.setCursor(0,1);
      lcd.print(cHum);
      lcd.setCursor(4,1);
      lcd.print("%");
      break;
      
    }
    case 10:{
      lcd.setCursor(0,0);
      lcd.print("Fan Mode");
      lcd.setCursor(0,1);
      lcd.print(sfanMode);
      break;
    }
    case 11:{
      lcd.setCursor(0,0);
      lcd.print("AvgCycTime: " + String(averageCycleTime));
      lcd.setCursor(0,1);
      lcd.print("TotalCycles: " + String(totalCycles));
      break;
    }
    case 12:{
      lcd.setCursor(0,0);
      lcd.print("Sensor Calibration");
      lcd.setCursor(0,1);
      lcd.print(String(sensorCalibration));
      break;
    }
  }

  //map the string set day time to the actual time of day being set
  switch (setDayTime){
      case 0:{
        sSetDayTime = "EMorning";
        break;
      }
      case 1:{
        sSetDayTime = "LMorning";
        break;
      }
      case 2:{
        sSetDayTime = "AftrNoon";
        break;
      }
      case 3:{
        sSetDayTime = "Evening ";
        break;
      }
      case 4:{
        sSetDayTime = "Night   ";
        break;
      }
    }
    //map the string for the current day to the day of the week
    switch (dayOfWeek){
      case 1:{
        sDayOfWeek = "Sun";
        break;
      }
      case 2:{
        sDayOfWeek = "Mon";
        break;
      }
      case 3:{
        sDayOfWeek = "Tue";
        break;
      }
      case 4:{
        sDayOfWeek = "Wed";
        break;
      }
      case 5:{
        sDayOfWeek = "Thu";
        break;
      }
      case 6:{
        sDayOfWeek = "Fri";
        break;
      }
      case 7:{
        sDayOfWeek = "Sat";
        break;
      }
    }


    //map for the operational mode
    switch (mode){
      case 0:{
        aMode = "Off ";
        break;
      }
      case 1:{
        aMode = "Cool";
        break;
      }
      case 2:{
        aMode = "Heat";
        break;
      }
    }

    //map for the fan mode
    switch (fanMode){
      case 0:{
        sfanMode = "AT";
        break;
      }
      case 1:{
        sfanMode = "ON";
        break;
      }
    }

    //map for the run mode
    switch (runMode){
      case 0:{
        sRunMode = "Auto";
        break;
      }
      case 1:{
        sRunMode = "Manu";
        break;
      }
       case 2:{
        sRunMode = "OvRd";
        break;
      }
    }

    //call the function to control the fan state
    controlFan();
    //call the function to control the heat
    controlHeat();
    //call the function to set the temperature for auto mode
    setTemperature();

    //auto write the data to the EEPROM
    if(millis()- lastEEPROMUpdateTime >= EEPROMUpdatePeriod)
    {
      //write the data to the EEPROM
      writeDataToEEPROM();
      
      //read the data from the EEPROM
      readDataFromEEPROM();

      //update the EEPROM timer
      lastEEPROMUpdateTime = millis();
      
    }

    //check if it is time for a soft reset
    if(millis() >= resetPeriod)
    {
      //call the function to perform a soft reset
      softReset();
    }

    //clear the override if approriate time has elapsed
    if (runMode == 2)
    {
      if (millis() - timeOverrideEnabled >= allowedOverrideTime)
      {
        runMode = 0;
      }
    }


    //code for the RTC Module Management
    if (Serial.available()){
    char buf = '\0';
    buf = Serial.read();
    if (buf == 's' || buf == 'S'){
       while(Serial.available())
         buf = Serial.read();
       Serial << "Set the date and time by entering the current date/time in" << endl;
       Serial << "the IDE serial monitor:" << endl << "  - must be in the 24 hour format, including commas" 
         << endl << endl;
       Serial << "YYYY,MM,DD,HH,DD,SS," << endl << endl
         << "(example: 2000,12,31,23,59,59, would be DEC 31st, 2000  11:59:59 pm)" << endl;
       Serial << "(must be typed in within 30 seconds or SET function times out)" << endl << endl;
       while(Serial.available())
         buf = Serial.read();
       setTimeFunction();
       printMenu();
    }
    else if (buf == 't' || buf == 'T'){
      t = now();
      printDateTime(t);
      Serial << endl << endl;
    }
    else if (buf == 'w' || buf == 'W'){
      Serial.println();
      Serial.println("writing data");
      delay(200);
      Serial.println("Done");
      RTC.writeEN(true);
      int j = 0;
      for(int i = 0; i < 62; i+=2){
        RTC.writeRTC((ramFirstAddress+i), 100+j);
        j++;
      }
      Serial.println();
    }
    else if (buf == 'r' || buf == 'R'){
      for(int i = 0; i < 62; i+=2){
        Serial.print("address :  ");
        Serial.print(ramFirstAddress+i, HEX);
        Serial.print("  contains :  ");
        Serial.print(RTC.readRTC(ramFirstAddress+i));
        Serial.println();
      }
      Serial.println();
      Serial.println("end of RAM read");
    }
    else if (buf == 'h' || buf == 'H'){
      RTC.writeRTC(DS1302_TRICKLE, B10100111);
      Serial << "Trickle Charger ENABLED (HIGH): 1 diodes, 8kohm" << endl << endl;
    }
    else if (buf == 'l' || buf == 'L'){
      RTC.writeRTC(DS1302_TRICKLE, B10101011);
      Serial << "Trickle Charger ENABLED (LOW): 2 diodes, 8kohm" << endl << endl;
    }
    else if (buf == 'o' || buf == 'O'){
      RTC.writeRTC(DS1302_TRICKLE, B01011100);
      Serial << "Trickle Charger DISABLED (OFF)" << endl << endl;
    }    
    while(Serial.available())
      buf = Serial.read();
  }     
}

//function to update the LCD when setting day temperatures
void printDayLCD()
{
    lcd.setCursor(0,0);
      lcd.print("Set ");
      lcd.setCursor(4,0);
      lcd.print(sSetDay);
      lcd.setCursor(8,0);
      lcd.print(sSetDayTime);
      lcd.setCursor(0,1);
      lcd.print(setTemps[dispMode][setDayTime]);
      lcd.setCursor(2,1);
      lcd.print("F");
      
}

void writeDataToEEPROM()
{
  //update the lcd
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Writing...");
  
  EEPROM.update(0, sTemp);
  delay(10);
  EEPROM.update(1, mode);
  delay(10);
  EEPROM.update(2, (sensorCalibration + 128));//add 128 when writing so that the unit can store negative values
  delay(10);
  EEPROM.update(3, fanMode);
  delay(10);
  EEPROM.update(4, runMode);
  delay(10);
  
  int z = 10;
  for(int x=1; x <= 7; x++)
  {
    for(int y=0; y <= 4; y++)
    {
      EEPROM.update(z, setTemps[x][y]);
      delay(10);
      z++;
    }
  }

  //update the lcd
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Done Writing");
  lcd.setCursor(0,1);
  lcd.print("Press Select");

  //read the data back from the EEPROM
  readDataFromEEPROM();
  dispMode = 0;
}

void readDataFromEEPROM()
{
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.write("Reading EEPROM");
  lcd.setCursor(0,1);
  lcd.print("Please Wait");
  
  sTemp = EEPROM.read(0);
  delay(10);
  mode = EEPROM.read(1);
  delay(10);
  sensorCalibration = EEPROM.read(2);
  delay(10);
  sensorCalibration -= 128; //subtract 128 from the value stored in the EEPROM because we can't store negative values there
  fanMode  = EEPROM.read(3);
  delay(10);
  runMode = EEPROM.read(4);
  delay(10);
  
  int z = 10;
  for(int x=1; x <= 7; x++)
  {
    for(int y=0; y <=4; y++)
    {
      setTemps[x][y] = EEPROM.read(z);
      delay(10);
      z++;
    }
  }

  lcd.clear();
}

//function to control the fan state
void controlFan()
{
  if (fanMode == 1)
  {
    //turn the fan on
    digitalWrite(fanRelayPin, LOW);
  }
  else
  {
    digitalWrite(fanRelayPin, HIGH);
  }
}

//function to control the heat
void controlHeat()
{
  //check if the thermostat is in heat mode 
  if(mode == 2 && dht11Chk == 0)
  {
    if (heating)
    {
      //check the temperature and see if we can end the call for heat
      if (cTemp >= (sTemp + tVariance))
      {
        //turn off the heat
        digitalWrite(heatRelayPin, HIGH);
        heating = false;

        //calcuate the average cycle time
        cycleEndTime = millis();
        totalCycleTime += cycleEndTime - cycleStartTime;
        totalCycles ++;
        averageCycleTime = (totalCycleTime / totalCycles) / 60000;
        
      }
    }
    else
    {
      //check the temperature and see if we need to turn on the heat
      if (cTemp <= (sTemp - tVariance))
      {
        //turn on the heat
        digitalWrite(heatRelayPin, LOW);
        cycleStartTime = millis();
        heating = true;
      }
    }
  }

  //else end the call for heat
  else
  {
    digitalWrite(heatRelayPin, HIGH);
    heating = false;
  }
  
}

//function to set the temperature if in automatic mode
void setTemperature()
{
  //determine what time of day it is
  //earlyMorning = 05:00 - 08:00
  if (timeHour >= 5 && timeHour < 8)
  {
    timeOfDay = 0;
  }
  //lateMorning = 08:00 - 12:00
  else if(timeHour >= 8 && timeHour < 12)
  {
    timeOfDay = 1;
  }
  //afternoon = 12:00 - 17:00
  else if(timeHour >= 12 && timeHour < 17)
  {
    timeOfDay = 2;
  }
  //evening = 17:00 - 23:00
  else if(timeHour >= 17 && timeHour < 23)
  {
    timeOfDay = 3;
  }
  //night = 23:00 - 05:00
  else if(timeHour >= 23  || timeHour < 5)
  {
    timeOfDay = 4;
  }

  //set the temperature based on the current day time
  if(runMode == 0)
  {
    sTemp = setTemps[dayOfWeek][timeOfDay];
  }
  
}
void printMenu(){
  Serial.println("  *************************************************");
  Serial.println("****  Programmable Thermostat Serial Setup Interface  ****");
  Serial.println("  *************************************************");
  Serial.println();
  Serial.println("type  \"TIME\"    to view current time");  
  Serial.println("type  \"SET\"     to set the time");
  Serial.println("type  \"WRITE\"   to write to RAM");
  Serial.println("type  \"READ\"    to read from RAM");
  Serial.println("type  \"HIGH\"    to turn ON high current trickle charger");
  Serial.println("type  \"LOW\"     to turn ON low current trickle charger");
  Serial.println("type  \"OFF\"     to turn OFF low current trickle charger");
  Serial << endl << endl << endl;
}
void setTimeFunction(){
  delayTime = millis() + 45000UL;
  while (delayTime >= millis() && !Serial.available()) {
    delay(10);
  }
  if (Serial.available()) {
        //note that the tmElements_t Year member is an offset from 1970,
        //but the RTC wants the last two digits of the calendar year.
        //use the convenience macros from Time.h to do the conversions.
            int y = Serial.parseInt();
            tm.Year = CalendarYrToTm(y);
            tm.Month = Serial.parseInt();
            tm.Day = Serial.parseInt();
            tm.Hour = Serial.parseInt();
            tm.Minute = Serial.parseInt();
            tm.Second = Serial.parseInt();
            t = makeTime(tm);
      //use the time_t value to ensure correct weekday is set
            if(RTC.set(t) == 0) { // Success
              setTime(t);
              Serial << "RTC set to: ";
              Serial << endl << endl;
              printDateTime(t);
              Serial << endl << endl;
              Serial << endl;
      }
      else
        Serial << "RTC set failed!" << endl;
            //dump any extraneous input
            while (Serial.available() > 0) Serial.read();
  }
  else 
    Serial << "timed out, please try again" << endl << endl;
}
void printDateTime(time_t t)
{
    printDate(t);
    Serial << ' ';
    printTime(t);
}
void printTime(time_t t)
{
    printI00(hour(t), ':');
    printI00(minute(t), ':');
    printI00(second(t), ' '); 
}
void printDate(time_t t)
{
    Serial << "DATE/TIME: ";    
    Serial << monthShortStr(month(t)) << ", ";
    printI00(day(t), 0);
    Serial << ", " << _DEC(year(t)) << " ";
}

//function to update the time and day variables
void updateStringTime(time_t t)
{
  timeHour = hour(t);
  //fix the hours for the lcd display
  if(timeHour < 10)
  {
    sTimeHour = "0" + String(timeHour);
  }
  else
  {
    sTimeHour = timeHour;
  }
  
  timeMinute = minute(t);

  //fix the minutes for the lcd display
  if (timeMinute < 10)
  {
    sTimeMinute = "0" + String(timeMinute);
  }
  else
  {
    sTimeMinute = timeMinute;
  }

  //get the day of the week
  dayOfWeek = (((t / 86400) + 4) % 7) + 1;
}
void printI00(int val, char delim)
{
    if (val < 10) Serial << '0';
    Serial << _DEC(val);
    if (delim > 0) Serial << delim;
    return;
}

//function to soft reset the controller
void softReset()
{
  asm volatile ("  jmp 0");
}
