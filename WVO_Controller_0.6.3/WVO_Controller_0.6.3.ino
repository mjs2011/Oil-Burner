/*
	WVO Controller

	This is the code for the WVO Controller development for my pickup. It is intended
	to handle the switching of fuel control valves and display important information on
	the touch screen display.

	The Code is written for use with an Arduino Mega, and may potentially be rewritten for
	an arduino micro.

	A Seeedstudio 2.8" TFT Touch Shield is used. A touch screen breakout may be used in the future.

	The circuit:
	*INPUTS:
		A10 - Photodiode for Auto Backlight Adjustment
		A11 - WVO Filter Head TEMP
    A14 - Fuel Pressure Sender
    A15 - Fuel Level Sender
    
		Future Implementations:
			Analog - Engine Coolant Temperature


	*OUTPUTS:
		22 - Piezo Buzzer Pin
		24 - Return Valve MOSFET Gate
		26 - Supply Valve/Fuel Pump MOSFET Gate

	Touchscreen Features:
		grid coordinates start: (0, 0) end: (239, 319)


	Code in Work
  By Max Sauer
------------------------------------------------------------------------------------------------------------------------------------------------------------------

Changelog:
  0.1.0   -   2/1/2015    -   Initial Prototype with TFT
  0.2.0   -   2/24/2015   -   Features added: Buzzer, Temp Sensor, Mosfets for Switching Valves
  0.3.0   -   3/15/2015   -   Temperature Set Point and Brightness Menu Options Added, creating Settings Window to Hold all settings options
  0.4.0   -   4/20/2015   -   EGT Gage Added
  0.5.0   -   unknown     -   Eeprom function added to save settings
  0.5.1   -   8/25/2015   -   Fixed Bug allowing access to the EGT screen while the controller was in Purge Mode. 
  0.5.2   -   8/26/2015   -   Brightness Scaling Equation Improved. Brightness button now has a smoother transition from dim to bright. 
  0.6.0   -   5/17/2016   -   Auto Brightness Equation Added. Photodiode added to hardware to sense Ambient light for Auto Brightness Equation. Changed Lettering to White. 
  0.6.1   -   6/3/2016    -   Added Fuel Gage functionality.
  0.6.2   -   8/1/2016    -   Bug Fixes which causes fuel gage lines not to display when reloading homescreen
                          -   Added a Cancel Button to the Purging Page, which allows the purge sequence to be cancelled in the event of an accidental Purge
  0.6.3   -   8/21/2017   -   Added Fuel Pressure to display. This utilizes a 3 wire fuel pressure sender. More info at https://www.amazon.com/gp/product/B00RCPDE40/ref=oh_aui_search_detailpage?ie=UTF8&psc=1
                          -   fixed bug causing cancel button not to revert back to oil
                          -   added alarm for fuel pressure when on oil if pressure falls below 6 psi, also added color to fuel pressure indicator on screen 
*/

#include <stdint.h>
#include <SeeedTouchScreen.h>
#include <TFTv2.h>
#include <SPI.h>
#include <Adafruit_MAX31855.h>
#include <EEPROM.h>

#define DO    28
#define CS    30
#define CLK   32
Adafruit_MAX31855 thermocouple(CLK, CS, DO);

const int numReadings = 10.00;				// number of readings to be taken for fuel temperature average
int readings[numReadings];  				  // the readings from the analog input
int index = 0;              				  // the index of the current reading
int total = 0;              				  // the running total
int average = 0;            				  // the average of temperature readings
int oldaverage = 0;          				  // the old average to overwrite on the scree
int temperaturePin = A11;			        // temperature input pin to read temperature of oil temperature
int temp = 0;
int EGT = 0;
int oldEGT = 0;

double gageMin = 50.00;
double gageMax = 950.00;
double theta;
double oldTheta;

boolean circle = 0;

const int buzzerPin = 22;
const int returnValve = 24;      			// red LED
const int supplyValve = 26;    				// green LED, will also power bypass valve and Fuel pump Relay
int page = 0;                  				// the page to prevent button pushes from working on pages where the button doesnt exist
                                      // homescreen = 0, settings = 2, purgeSetTime = 3, tempSet = 4

int purgeTime;					              // purge time of the of valves in seconds. Set to 5 now for ease of troubleshooting.
                                      // minimum in purgeSetTime menu is 30. this value will be changed for final code.
unsigned long Timer = 0;				      // unkown variable.....
boolean autoState;    				        // whether the controller is running in auto or manual mode. Auto = 1, manual = 0.
boolean oilState = 0;     				    // whether the controller is switched to oil or not. Oil = 1, diesel = 0.
boolean tempState = 0;     				    // whether the temp is above or below set temp. Above setTemp = 1, below setTemp = 0.
boolean oldtempState = 0;				      // used to get into if statements only when tempState has just switched and oldTempState is different.
boolean purgeState = 0;					      // whether or not purgeTimer is running. Purging = 1, not purging = 0.
int setTemp;       				            // Automatic oil switch and purge temperature threshold in degrees Farenheit
long previousMillis = 0;				      // time in millis when purge timer function begins
unsigned long currentMillis = 0;			// current time in millis compared to previousMillis to determine when purgeTime is met
int countdown = 0;					          // purgeTime left in seconds, to be displayed during purge process
int oldCountdown = 0;					        // old purgeTime value , compared to countdown to determine when to overwrite countdown value.

int backlightPin = 7;
float backlightValue;
float dayLightVal;
float backlightPercent = 100;
float backlightPerc;
int backlightVal = 255;
int oldBacklightVal;
int oldLightLevel;
int lightLevel;

boolean returnState = 0;
boolean oldReturnState = 0;
boolean bypassState = 0;
boolean oldBypassState = 0;
boolean supplyState = 0;
boolean oldSupplyState = 0;

int autoStateAddress = 0;
int purgeTimeAddress = 1;
int tempSetPointAddress = 2;
int brightnessAddress = 3;

int fuelLevel;                            // value from 0 to 100 equating to the fuel level
float fuelLevelVal;                       // analog value from 728 to 254, read from the fuel level voltage divider;
int xLength;                              // length of bar in fuel level bar graph
int oldxLength;
int fuelLevelPin = A15;                   // pin that is reading the voltage of the voltage divider for the fuel sender

int readingP[numReadings];                // the readings from the analog input
int indexP = 0;                           // the index of the current reading
int sumP = 0;                             // the running total
int psi = 0;
int pressure = 0;                         // current fuel pressure
int oldpressure = 0;                      // old fuel pressure
int pressurePin = A14;                    // analog pressure pin
double pressureVoltage;

TouchScreen ts = TouchScreen(XP, YP, XM, YM);	        // values for coordinate system inputs on touchscreen.

void setup()
{
  screen_brightness();

  Tft.TFTinit();                                        // init TFT library

  pinMode(24, OUTPUT);					                        // needed to set digital pin to high at 5v for switching MOSFET
  pinMode(26, OUTPUT);					                        // needed to set digital pin to high at 5v for switching MOSFET

  Serial.begin(9600);

  Tft.drawRectangle(10, 20, 220, 280, GREEN); 	        // (X-coord start, Y-coord start, X length, y length, color)
  Tft.drawString("OIL", 73, 38, 5, GREEN);
  Tft.drawString("OIL", 75, 40, 5, BLUE);
  delay(500);
  Tft.drawString("BURNER", 23, 78, 5, GREEN);
  Tft.drawString("BURNER", 25, 80, 5, BLUE);
  delay(500);
  Tft.drawString("V 0.6.3", 55, 200, 3, BLUE);
  delay(500);
  Tft.drawString("Max Sauer", 40, 250, 3, WHITE);

  pinMode(buzzerPin, OUTPUT);
  //tone(buzzerPin, 500, 250);
  delay(250);
  //tone(buzzerPin, 750, 500);
  delay(200);

  autoState = EEPROM.read(autoStateAddress);
  backlightPercent = EEPROM.read(brightnessAddress);
  purgeTime = EEPROM.read(purgeTimeAddress);
  setTemp = EEPROM.read(tempSetPointAddress);

  delay(3000);						                        // display boot screen for set time in millis
  Tft.fillScreen();					                      // clear display

  homescreen();						                        // draw homescreen once boot screen is cleared

  Point p = ts.getPoint();				                // setup point to read touch values
  p.x = 0;
  p.y = 0;
}

/*
*************************************************************************************************************************************************************
								END OF SETUP
*************************************************************************************************************************************************************
*/

void loop()
{

  Point p = ts.getPoint();
  p.x = map(p.x, TS_MINX, TS_MAXX, 0, 240);
  p.y = map(p.y, TS_MINY, TS_MAXY, 0, 320);

  temperature();
  screen_brightness();
  fuelGage();
  fuelPressure();

  if (page == 0 && purgeState == 0)
  {
    EGT = thermocouple.readFarenheit();
    if (EGT != oldEGT)
    {
      if (EGT > 999)			// if EGT is 1000 or bigger, display value to left for centering
      {
        Tft.fillRectangle(165, 180, 75, 25, BLACK);      		    // clear old exhaust gas temperature
        Tft.drawNumber(EGT, 165, 180, 3, WHITE);  		// write new exhaust gas temperature
      }
      else					        	// else, temperature is smaller than 1000, display in line with oil and coolant temps
      {
        Tft.fillRectangle(165, 180, 75, 25, BLACK);  			// clear old exhaust gas temperature
        Tft.drawNumber(EGT, 180, 180, 3, WHITE);  		// write new exhaust gas temperature
      }
    }
    oldEGT = EGT;						// once new average has been written, store value in old average for next loop iteration  					        	// plot temperature

    if (EGT > 1100)                                       // sound alarm, EGT is too high
    {
      delay(250);
      tone(buzzerPin, 750, 250);
      delay(250);
      tone(buzzerPin, 500, 250);
      delay(250);
    }

    if (page == 0)
    {
      /*
      returnState = digitalRead(returnCheckPin);
      bypassState = digitalRead(bypassCheckPin);
      supplyState = digitalRead(supplyCheckPin);

      if (returnState != oldReturnState)
      {
        if (returnState == LOW)
        {
          Tft.fillCircle(200, 200, 14, RED);
        }
        else
        {
          Tft.fillCircle(200, 200, 14, BLACK);
        }
      }

      if (bypassState != oldBypassState)
      {
        if (bypassState == LOW)
        {
          Tft.fillCircle(120, 200, 14, RED);
        }
        else
        {
          Tft.fillCircle(120, 200, 14, BLACK);
        }
      }

      if (supplyState != oldSupplyState)
      {
        if (supplyState == LOW)
        {
          Tft.fillCircle(40, 200, 14, RED);
        }
        else
        {
          Tft.fillCircle(40, 200, 14, BLACK);
        }
      }

      oldReturnState = returnState;
      oldBypassState = bypassState;
      oldSupplyState = supplyState;
      */
    }

    if (average != oldaverage)
    {
      Tft.drawString("FUEL TEMP", 0, 120, 3, WHITE);
      Tft.drawString("FUEL PRES.", 0, 150,3,WHITE);
      if (pressure > 6)
      {
        Tft.drawNumber(pressure, 200, 150, 3, GREEN);
      }
      if (pressure <= 6 && oilState == 1)
      {
        Tft.drawNumber(pressure, 200, 150, 3, RED);
        delay(250);
        tone(buzzerPin, 750, 250);
        delay(250);
        tone(buzzerPin, 500, 250);
        delay(250);
      }
      if (oilState == 0)
      {
        Tft.drawNumber(pressure, 200, 150, 3, WHITE);
      }
      Tft.drawString("EGT", 0, 180, 3, WHITE);
      Tft.drawNumber(oldaverage, 180, 120, 3, BLACK);   	    // clear old oil temperature
      Tft.drawNumber(average, 180, 120, 3, WHITE);        		// write new average oil temperature
      //Tft.drawNumber(oldaverage,180,150,3,BLACK);    		// clear old average engine temperature
      //Tft.drawNumber(average,180,150,3,YELLOW);    		// write new average oil temperature
      oldaverage = average;
    }

    if (pressure != oldpressure)
    {
      Tft.drawString("FUEL TEMP", 0, 120, 3, WHITE);
      Tft.drawString("FUEL PRES.", 0, 150,3,WHITE);
      Tft.drawNumber(oldpressure, 200, 150, 3, BLACK);
      if (pressure > 6)
      {
        Tft.drawNumber(pressure, 200, 150, 3, GREEN);
      }
      if (pressure <= 6 && oilState == 1)
      {
        Tft.drawNumber(pressure, 200, 150, 3, RED);
        delay(250);
        tone(buzzerPin, 750, 250);
        delay(250);
        tone(buzzerPin, 500, 250);
        delay(250);
      }
      if (oilState == 0)
      {
        Tft.drawNumber(pressure, 200, 150, 3, WHITE);
      }
      Tft.drawString("EGT", 0, 180, 3, WHITE);
      Tft.drawNumber(average, 180, 120, 3, WHITE);            // write new average oil temperature
      //Tft.drawNumber(oldaverage,180,150,3,BLACK);       // clear old average engine temperature
      //Tft.drawNumber(average,180,150,3,YELLOW);       // write new average oil temperature
      oldpressure = pressure;
    }

  }

  if (purgeState == 1)						// display purge timer on screen if controller is in purge mode
  {
    currentMillis = millis();
    //Serial.println(previousMillis);
    //Serial.println(currentMillis);
    countdown = (purgeTime - (currentMillis - previousMillis) / 1000); 		// if time since purge started is less than purgeSetTime, keep running purge
    if (countdown != oldCountdown)						// if old and new countdown values are different, write countdown to screen
    {
      if (countdown >= 10)							// if countdown is < 10, center countdown value on screen
      {
        Tft.fillRectangle(90, 140, 70, 40, BLACK);
        Tft.drawNumber(countdown, 90, 140, 5, WHITE);
      }
      else									// otherwise, countdown is greater than 9, and countdown is centered
      {
        Tft.fillRectangle(90, 140, 60, 40, BLACK);
        Tft.drawNumber(countdown, 100, 140, 5, WHITE);
      }
    }
    oldCountdown = countdown;
    if ((currentMillis - previousMillis > purgeTime * 1000UL))		// if the purge time has ran longer than purgeSetTime,
      // run returnOff which will turn off the Return valve
    {
      p.x = 0;
      p.y = 0;
      returnOff();
    }
    
    if ((p.x > 0 && p.x < 240 && p.y > 250 && p.y < 320))         // if cancel button is pressed, turn back on supply valve
    {
        tone(buzzerPin, 750, 250);
        p.x = 0;
        p.y = 0;
        digitalWrite(supplyValve, HIGH);
        purgeState = 0;
        oilState = 1;
        homescreen();
    }
  }

  if (average > setTemp)						        // if actual temperature is above temperature threshold, change tempState to true
  {
    tempState = 1;
  }
  if (average < setTemp - 5)							// if actual temp. is below temperature threshold with 5 degree buffer, set tempState to false
  {
    tempState = 0;
  }

  if (tempState == 1 && oilState == 0 && oldtempState == 0 && autoState == 0 && page == 0 && purgeState == 0)
    // display switch button, temp is now high enough for manual switching
  {
    Tft.fillRectangle(0, 250, 240, 70, GREEN);
    Tft.drawString("SWITCH", 44, 270, 4, BLUE);
    oldtempState = tempState;
  }

  if (tempState == 0 && oldtempState == 1 && page == 0 )
    // remove switch button, temp has fallen below set temp for manual switching
  {
    Tft.fillRectangle(0, 250, 240, 70, BLACK);
    oldtempState = tempState;
  }

  if (tempState == 1 && autoState == 1 && oilState == 0 && purgeState == 0 && page == 0)  		//switch to oil, temp is now high enough. Automatic Mode is On.
  {
    oilSwitch();
    tone(buzzerPin, 750, 250);
    digitalWrite(supplyValve, HIGH);
    digitalWrite(returnValve, HIGH);
    Tft.drawString("DIESEL", 52, 70, 4, BLACK);
    Tft.drawString("OIL", 85, 70, 4, GREEN);
    Tft.fillRectangle(0, 250, 240, 70, RED);
    Tft.drawString("PURGE", 54, 270, 4, WHITE);
    delay(500);
  }

  if (oilState == 1 && tempState == 0 && page == 0)				        	// if temperature falls below setTemp, run fuelPurge whether mode is Auto or Manual
  {
    tone(buzzerPin, 750, 250);    						// Give a Warning beep to notify user that temp is low.
    delay(500);
    tone(buzzerPin, 750, 250);
    currentMillis = millis();
    previousMillis = currentMillis;
    oilSwitch();
    fuelPurge();
    //Tft.drawString("OIL",85,70,4,BLACK);
    //Tft.drawString("DIESEL",52,70,4,BLUE);
  }



  if (p.z > __PRESURE)								// Purge Time set button
  {
    if ((p.x > 130) && (p.x < 240) && (p.y > 0) && (p.y < 50))
    {
      if (page == 0 && purgeState == 0)
      {
        tone(buzzerPin, 750, 250);
        settings();
      }
    }
  }

  if (p.z > __PRESURE)								// EGT Button
  {
    if ((p.x > 0) && (p.x < 120) && (p.y > 140) && (p.y < 190))
    {
      if (page == 0 && purgeState ==0)
      {
        tone(buzzerPin, 750, 250);
        gage();
      }
    }
  }

  if (page == 6)
  {
    EGT = thermocouple.readFarenheit();
    if (EGT != oldEGT)
    {
      if (EGT > 999)			// if EGT is 1000 or bigger, display value to left for centering
      {
        Tft.fillRectangle(100, 140, 75, 30, BLACK);      		    // clear old exhaust gas temperature
        Tft.drawNumber(EGT, 100, 140, 3, WHITE);  		// write new exhaust gas temperature
      }
      else					        	// else, temperature is smaller than 1000, display in line with oil and coolant temps
      {
        Tft.fillRectangle(100, 140, 75, 30, BLACK);  			// clear old exhaust gas temperature
        Tft.drawNumber(EGT, 100, 140, 3, WHITE);  		// write new exhaust gas temperature
      }
    }
    oldEGT = EGT;						// once new average has been written, store value in old average for next loop iteration
    theta = 3.14 - 3.14 * ((EGT - gageMin) / (gageMax - gageMin));
    //Serial.println(theta);
    //Serial.println(EGT);
    Tft.fillCircle(120, 120, 5, WHITE);
    if (oldTheta != theta)
    {
      Tft.drawLine(120, 120, 120 + 60 * cos(oldTheta), 120 - 60 * sin(oldTheta), BLACK);
      Tft.fillCircle(120, 120, 5, WHITE);
      Tft.drawLine(120, 120, 120 + 60 * cos(theta), 120 - 60 * sin(theta), WHITE);
    }
    oldTheta = theta;
  }

  if (p.z > __PRESURE)								// switch/purge button
  {
    if (p.x > 0 && p.x < 240 && p.y > 250 && p.y < 320)
    {
      if (page == 0 && autoState == 0 && tempState == 1 && purgeState == 0)
      {
        oilSwitch();
        tone(buzzerPin, 750, 250);
        if (oilState == 0)
        {
          currentMillis = millis();
          previousMillis = currentMillis;
          fuelPurge();
          //Tft.drawString("OIL",85,70,4,BLACK);
          //Tft.drawString("DIESEL",52,70,4,BLUE);
          //Tft.fillRectangle(0,250,240,70,GREEN);
          //Tft.drawString("SWITCH",44,270,4,BLUE);
        }
        if ( oilState == 1)
        {
          digitalWrite(supplyValve, HIGH);
          digitalWrite(returnValve, HIGH);
          Tft.drawString("DIESEL", 52, 70, 4, BLACK);
          Tft.drawString("OIL", 85, 70, 4, GREEN);
          Tft.fillRectangle(0, 250, 240, 70, RED);
          Tft.drawString("PURGE", 54, 270, 4, WHITE);
        }
      }
      if (page == 0 && autoState == 1 && purgeState == 0)
      {
        autoState = 0;
        oilSwitch();
        tone(buzzerPin, 750, 250);
        currentMillis = millis();
        previousMillis = currentMillis;
        fuelPurge();

      }
      delay(500);
    }
  }

  if (p.z > __PRESURE)								// home button
  {
    if (p.x > 200 && p.x < 240 && p.y > 280 && p.y < 320)
    {
      if (page == 2 || page == 3 || page == 4 || page == 5 || page == 6)
      {
        EEPROM.update(brightnessAddress, backlightPercent);
        EEPROM.update(purgeTimeAddress, purgeTime);
        EEPROM.update(tempSetPointAddress, setTemp);
        tone(buzzerPin, 500, 250);
        homescreen();
      }
    }
  }

  if (p.z > __PRESURE)								// back button
  {
    if (p.x < 40 && p.y > 280 && p.y < 320)
    {
      if (page == 2)
      {
        tone(buzzerPin, 1000, 250);
        homescreen();
      }
      if (page == 4 || page == 3 || page == 5 )
      {
        p.x = 0;
        p.y = 0;
        EEPROM.update(brightnessAddress, backlightPercent);
        EEPROM.update(purgeTimeAddress, purgeTime);
        EEPROM.update(tempSetPointAddress, setTemp);
        tone(buzzerPin, 1000, 250);
        settings();
      }
    }
  }

  if (p.z > __PRESURE)								// purge plus button
  {
    if (p.x > 50 && p.x < 190 && p.y > 40 && p.y < 110)
    {
      if (page == 3)
      {
        tone(buzzerPin, 1000, 125);
        purgeTime = purgeTime + 5;
        delay(50);
        Tft.fillRectangle(50, 120, 140, 80, BLACK);
        if (purgeTime < 100)
        {
          Tft.drawNumber(purgeTime, 85, 140, 5, WHITE);
        }
        else
        {
          Tft.drawNumber(purgeTime, 70, 140, 5, WHITE);
        }
      }
      if (page == 4)
      {
        tone(buzzerPin, 1000, 125);
        setTemp = setTemp + 5;
        delay(50);
        Tft.fillRectangle(50, 120, 140, 80, BLACK);
        Tft.drawNumber(setTemp, 70, 140, 5, WHITE);
      }
      if (page == 5)
      {
        tone(buzzerPin, 1000, 125);
        if (backlightPercent < 100)
        {
          backlightPercent = backlightPercent + 5;
          delay(50);
          //backlightValue = ((backlightPercent*backlightPercent*backlightPercent)/3937) + 1;
          //backlightValue = (127 * backlightPercent) / 50 + 1;
          analogWrite(backlightPin, backlightValue);
          Tft.fillRectangle(50, 120, 160, 80, BLACK);
          if (backlightPercent < 10)
          {
            Tft.drawNumber(backlightPercent, 85, 140, 5, WHITE);
            Tft.drawString("%", 125, 140, 5, WHITE);
          }
          if (backlightPercent < 100 && backlightPercent >= 10)
          {
            Tft.drawNumber(backlightPercent, 65, 140, 5, WHITE);
            Tft.drawString("%", 145, 140, 5, WHITE);
          }
          if (backlightPercent == 100)
          {
            Tft.drawNumber(backlightPercent, 50, 140, 5, WHITE);
            Tft.drawString("%", 165, 140, 5, WHITE);
          }
        }
      }
    }
  }


  if (p.z > __PRESURE)								// purge minus button
  {
    if (p.x > 50 && p.x < 190 && p.y > 210 && p.y < 280)
    {
      if (page == 3)
      {
        tone(buzzerPin, 650, 125);
        if (purgeTime >= 35)
        {
          purgeTime = purgeTime - 5;
          delay(50);
          Tft.fillRectangle(50, 120, 140, 80, BLACK);
          if (purgeTime < 100)
          {
            Tft.drawNumber(purgeTime, 85, 140, 5, WHITE);
          }
          else
          {
            Tft.drawNumber(purgeTime, 70, 140, 5, WHITE);
          }
        }
      }
      if (page == 4)
      {
        tone(buzzerPin, 650, 125);
        if (setTemp >= 105)
        {
          setTemp = setTemp - 5;
          delay(50);
          Tft.fillRectangle(50, 120, 140, 80, BLACK);
          Tft.drawNumber(setTemp, 70, 140, 5, WHITE);
        }
      }
      if (page == 5)
      {
        tone(buzzerPin, 650, 125);
        if (backlightPercent > 0)
        {
          backlightPercent = backlightPercent - 5;
          delay(50);
          //backlightValue = ((backlightPercent*backlightPercent*backlightPercent)/3937) + 1;
          //backlightValue = (127 * backlightPercent) / 50 + 1;
          analogWrite(backlightPin, backlightValue);
          Tft.fillRectangle(50, 120, 160, 80, BLACK);
          if (backlightPercent < 10)
          {
            Tft.drawNumber(backlightPercent, 85, 140, 5, WHITE);
            Tft.drawString("%", 125, 140, 5, WHITE);
          }
          if (backlightPercent < 100 && backlightPercent >= 10)
          {
            Tft.drawNumber(backlightPercent, 65, 140, 5, WHITE);
            Tft.drawString("%", 145, 140, 5, WHITE);
          }
          if (backlightPercent == 100)
          {
            Tft.drawNumber(backlightPercent, 50, 140, 5, WHITE);
            Tft.drawString("%", 165, 140, 5, WHITE);
          }
        }
      }
    }
  }

  if (p.z > __PRESURE)								// Various Buttons
  {
    if (p.y > 50 && p.y < 110 && page == 2) 					// purgeSetTime button
    {
      tone(buzzerPin, 750, 250);
      purgeSetTime();
    }

    if (p.y > 125 && p.y < 190 && page == 2) 				       // tempSetTime button
    {
      tone(buzzerPin, 750, 250);
      tempSet();
    }

    if (p.y > 210 && p.y < 270 && page == 2) 				       // brightness button
    {
      tone(buzzerPin, 750, 250);
      brightness();
    }

    if (p.x > 0 && p.x < 109 && p.y > 0 && p.y < 50) 			       // auto/manual button on homescreen
    {
      if (page == 0 && purgeState == 0)
      {
        tone(buzzerPin, 650, 125);
        autoButton();
        EEPROM.update(autoStateAddress, autoState);
        delay(250);
        if (autoState == 1 && oilState == 0)
        {
          Tft.fillRectangle(0, 250, 240, 70, BLACK);
        }
        if (autoState == 0 && average > setTemp && oilState == 0)
        {
          Tft.fillRectangle(0, 250, 240, 70, GREEN);
          Tft.drawString("SWITCH", 44, 270, 4, BLUE);
        }
      }
    }
  }

}

/*
***************************************************************************************************************************************************************
								END of LOOP
***************************************************************************************************************************************************************
*/

void homescreen()						// function that draws homescreen buttons and info
{
  page = 0;
  circle = 0;
  Tft.fillScreen();
  //Tft.fillRectangle(0, 0, 239, 319, WHITE);
  if (oilState == 0)
  {
    Tft.drawString("OIL", 85, 70, 4, BLACK);
    Tft.drawString("DIESEL", 52, 70, 4, WHITE);
  }
  else
  {
    Tft.drawString("DIESEL", 52, 70, 4, BLACK);
    Tft.drawString("OIL", 85, 70, 4, GREEN);
  }
  //settings button
  Tft.fillRectangle(130, 0, 109, 50, BLUE);
  Tft.drawString("SETTINGS", 137, 18, 2, WHITE);

  //auto/manual button
  Tft.fillRectangle(0, 0, 109, 50, BLUE);
  if (autoState == 1)
  {
    Tft.drawString("MANUAL", 15, 18, 2, BLUE);
    Tft.drawString("AUTO", 15, 15, 3, WHITE);
  }
  else
  {
    Tft.drawString("AUTO", 15, 15, 3, BLUE);
    Tft.drawString("MANUAL", 15, 18, 2, WHITE);
  }

  //switch/purge button
  if (autoState == 0 && average > setTemp && oilState == 0)
  {
    Tft.fillRectangle(0, 250, 240, 70, GREEN);
    Tft.drawString("SWITCH", 44, 270, 4, BLUE);
  }

  if (oilState == 1)
  {
    Tft.fillRectangle(0, 250, 240, 70, RED);
    Tft.drawString("PURGE", 54, 270, 4, WHITE);
  }
  if (page == 0)                                              //plot temperature
  {
    Tft.drawString("FUEL TEMP", 0, 120, 3, WHITE);
    Tft.drawString("FUEL PRES.", 0, 150,3,WHITE);
    Tft.drawNumber(oldpressure, 200, 150, 3, BLACK);
    if (pressure > 6)
    {
      Tft.drawNumber(pressure, 200, 150, 3, GREEN);
    }
    if (pressure <= 6 && oilState == 1)
    {
      Tft.drawNumber(pressure, 200, 150, 3, RED);
      delay(250);
      tone(buzzerPin, 750, 250);
      delay(250);
      tone(buzzerPin, 500, 250);
      delay(250);
    }
    if (oilState == 0)
    {
      Tft.drawNumber(pressure, 200, 150, 3, WHITE);
    }
    Tft.drawString("EGT", 0, 180, 3, WHITE);
    Tft.drawNumber(oldaverage, 180, 120, 3, BLACK);               //oil temp
    Tft.drawNumber(average, 180, 120, 3, WHITE);                    //oil temp
    //Tft.drawNumber(oldaverage,180,150,3,BLACK);                   //engine temp
    //Tft.drawNumber(average,180,150,3,YELLOW);                     //engine temp
    if (oldEGT > 999 && EGT > 999)
    {
      Tft.fillRectangle(165, 180, 75, 30, BLACK);                  //exhaust temp
      Tft.drawNumber(EGT, 165, 180, 3, WHITE);          //exhaust temp
    }
    else
    {
      Tft.fillRectangle(165, 180, 75, 30, BLACK);                  //exhaust temp
      Tft.drawNumber(EGT, 180, 180, 3, WHITE);          //exhaust temp
    }
    oldaverage = average;
    oldpressure = pressure;
  }
//
    Tft.drawString("TANK", 0, 210, 3, WHITE);
    Tft.fillRectangle(130,208,106,26,BLUE);
    if (xLength > 100)
    {
      xLength = 100;
    }
    if (xLength < 0)
    {
      xLength = 0;
    }
    Tft.fillRectangle(133, 211, xLength, 20, WHITE);
    for(int xCoord = 133; xCoord <243; xCoord+=10)
    {
      Tft.drawLine(xCoord,211,xCoord,231,BLACK);
    }
    fuelGage();
}

void oilSwitch()				        	// changes oilState for comparison when purging
{
  if (oilState == 0)
  {
    oilState = 1;
  }
  else
  {
    oilState = 0;
  }
}

void homebutton()						// function that draws the home button
{
  // home icon (rotated right now)
  //Tft.drawLine(219, 280, 200, 299, WHITE);
  //Tft.drawLine(200, 300, 204, 304, WHITE);
  //Tft.drawLine(203, 304, 200, 304, WHITE);
  //Tft.drawLine(200, 305, 200, 307, WHITE);
  //Tft.drawLine(200, 308, 208, 308, WHITE);
  //Tft.drawLine(209, 309, 219, 319, WHITE);
  //Tft.drawLine(219, 281, 219, 283, WHITE);
  //Tft.drawLine(219, 316, 219, 318, WHITE);
  //Tft.drawRectangle(219, 284, 21, 32, WHITE);
  //Tft.drawRectangle(225, 295, 15, 10, WHITE);

  Tft.drawRectangle(203, 300, 32, 19, WHITE);
  Tft.drawRectangle(214, 304, 10, 15, WHITE);
  Tft.drawRectangle(207, 282, 4, 10, WHITE);
  Tft.drawTriangle(201, 300, 219, 282, 237, 300, WHITE);
}

void backArrow()						// function that draws the back arrow button
{
  Tft.drawTriangle(3, 300, 21, 278, 21, 318, WHITE);
  Tft.drawRectangle(21, 288, 30, 20, WHITE);
}

void tempSet()							// function that draws buttons for tempSet Page
{
  page = 4;
  Tft.fillScreen();
  homebutton();
  backArrow();
  Tft.drawString("SWITCH TEMP", 55, 5, 2, WHITE);
  Tft.fillRectangle(50, 40, 140, 70, GREEN);
  Tft.fillRectangle(50, 210, 140, 70, GREEN);
  Tft.drawNumber(setTemp, 70, 140, 5, WHITE);
  Tft.drawChar('+', 103, 58, 5, WHITE);
  Tft.drawChar('-', 103, 228, 5, WHITE);
}

void purgeSetTime()						// function that draws buttons for purgeSetTime page
{
  page = 3;
  Tft.fillScreen();
  homebutton();
  backArrow();
  Tft.drawString("Purge Time in Sec", 15, 5, 2, WHITE);
  Tft.fillRectangle(50, 40, 140, 70, GREEN);
  Tft.fillRectangle(50, 210, 140, 70, GREEN);
  if (purgeTime < 100)
  {
    Tft.drawNumber(purgeTime, 85, 140, 5, WHITE);
  }
  else
  {
    Tft.drawNumber(purgeTime, 70, 140, 5, WHITE);
  }
  Tft.drawChar('+', 103, 58, 5, WHITE);
  Tft.drawChar('-', 103, 228, 5, WHITE);
}

void autoButton()						// function that displays autoState in auto/manual button on homescreen
{
  if (autoState == 1)
  {
    autoState = 0;
    Tft.drawString("AUTO", 15, 15, 3, BLUE);
    Tft.drawString("MANUAL", 15, 18, 2, WHITE);
  }
  else
  {
    autoState = 1;
    Tft.drawString("MANUAL", 15, 18, 2, BLUE);
    Tft.drawString("AUTO", 15, 15, 3, WHITE);
  }
}

void fuelPurge()						// function that changes purgeState for purging the oil system
{
  digitalWrite(supplyValve, LOW);
  Tft.fillScreen();
  Tft.drawString("PURGING", 35, 70, 4, WHITE);
  Tft.fillRectangle(0, 250, 240, 70, RED);
  Tft.drawString("CANCEL", 54, 270, 4, WHITE);
  purgeState = 1;
  //timer functions only with delay(), screen is locked up during purge process
  //if(millis() - Timer >= purgeTime*1000UL);
  {
    //digitalWrite(returnValve,LOW);
    //tone(buzzerPin, 500, 250);
    //delay(500);
    //tone(buzzerPin, 750, 250);
  }
}

void returnOff()						// function that turns off returnValve after fuelPurge is complete
{
  digitalWrite(returnValve, LOW);
  tone(buzzerPin, 500, 250);
  delay(500);
  tone(buzzerPin, 750, 250);
  Tft.fillScreen();
  oilState = 0;
  purgeState = 0;
  homescreen();
}

void settings()							// function that draws buttons for settings menu
{
  page = 2;
  Tft.fillScreen();
  homebutton();
  backArrow();
  Tft.drawString("SETTINGS", 43, 10, 3, WHITE);
  Tft.fillRectangle(20, 50, 200, 60, BLUE);
  Tft.fillRectangle(20, 130, 200, 60, BLUE);
  Tft.fillRectangle(20, 210, 200, 60, BLUE);
  Tft.drawString("PURGE TIME", 55, 75, 2, WHITE);
  Tft.drawString("TEMP. SET POINT", 30, 155, 2, WHITE);
  Tft.drawString("BRIGHTNESS", 60, 235, 2, WHITE);
}

void brightness()
{
  page = 5;
  Tft.fillScreen();
  homebutton();
  backArrow();
  Tft.drawString("SCREEN BRIGHTNESS", 15, 5, 2, WHITE);
  Tft.fillRectangle(50, 40, 140, 70, GREEN);
  Tft.fillRectangle(50, 210, 140, 70, GREEN);
  if (backlightPercent < 10)
  {
    Tft.drawNumber(backlightPercent, 85, 140, 5, WHITE);
    Tft.drawString("%", 125, 140, 5, WHITE);
  }
  if (backlightPercent < 100 && backlightPercent >= 10)
  {
    Tft.drawNumber(backlightPercent, 65, 140, 5, WHITE);
    Tft.drawString("%", 145, 140, 5, WHITE);
  }
  if (backlightPercent == 100)
  {
    Tft.drawNumber(backlightPercent, 50, 140, 5, WHITE);
    Tft.drawString("%", 165, 140, 5, WHITE);
  }
  Tft.drawChar('+', 103, 58, 5, WHITE);
  Tft.drawChar('-', 103, 228, 5, WHITE);
}

void gage()
{
  page = 6;
  Tft.fillScreen();
  homebutton();
  if (circle == 0)
  {
    Tft.fillCircle(120, 120, 78, GREEN);  					        	// plot temperature
    Tft.fillCircle(120, 120, 68, BLACK);
    Tft.fillRectangle(0, 120, 240, 115, BLACK);
    circle = 1;
    Tft.drawString("50", 12, 110, 2, WHITE);
    Tft.drawString("500", 103, 23, 2, WHITE);
    Tft.drawString("950", 200, 110, 2, WHITE);
  }

  theta = 3.14 - 3.14 * ((EGT - gageMin) / (gageMax - gageMin));
  //Serial.println(theta);
  //Serial.println(EGT);
  Tft.fillCircle(120, 120, 5, WHITE);
  if (oldTheta != theta)
  {
    Tft.drawLine(120, 120, 120 + 60 * cos(oldTheta), 120 - 60 * sin(oldTheta), BLACK);
    Tft.fillCircle(120, 120, 5, WHITE);
    Tft.drawLine(120, 120, 120 + 60 * cos(theta), 120 - 60 * sin(theta), WHITE);
  }
  oldTheta = theta;
}

void fuelGage()
{
  fuelLevelVal = analogRead(fuelLevelPin);
  fuelLevel = -0.0256*fuelLevelVal + 18.926;
  xLength = fuelLevel*10;
  //Serial.println(fuelLevelVal);
  if (xLength != oldxLength && page == 0)
  {
//    Serial.print("xLength   ");
//    Serial.print(xLength);
//    Serial.print("    ");
//    Serial.println(oldxLength);
    
    oldxLength = xLength;
        
    if (xLength > 100)
    {
      xLength = 100;
    }
    if (xLength < 0)
    {
      xLength = 0;
    }

    Tft.drawString("TANK", 0, 210, 3, WHITE);
    Tft.fillRectangle(130,208,106,26,BLUE);
    Tft.fillRectangle(133, 211, xLength, 20, WHITE);
    for(int xCoord = 133; xCoord <243; xCoord+=10)
    {
      Tft.drawLine(xCoord,211,xCoord,231,BLACK);
    }
  }
}

void fuelPressure()
{
  pressureVoltage = analogRead(pressurePin)*5.0/1023.0;
  psi = 7.5*pressureVoltage - 3.075;
  
  sumP = sumP - readingP[indexP];
  // read from the sensor:
  readingP[indexP] = psi;
  // add the reading to the total:
  sumP = sumP + readingP[indexP];
  // advance to the next position in the array:
  indexP = indexP + 1;
  // if we're at the end of the array...
  if (indexP >= numReadings)
    // ...wrap around to the beginning:
    indexP = 0;
  // calculate the average:
  pressure = sumP / numReadings;
}

void screen_brightness()
{
  /*
   * Below is old code for the backlight brightness when the manual setting was still used, before upgrading to the auto brightness 
   * function with the photo-diode
  Serial.println(dayLightVal);
  backlightPerc = (0.0000003*(dayLightVal*dayLightVal*dayLightVal))-(0.0006*(dayLightVal*dayLightVal))+(0.397*dayLightVal);
  Serial.println(backlightPerc);
  Serial.println(backlightValue);
  backlightValue = ((backlightPercent*backlightPercent*backlightPercent)/3937) + 1.5;
  backlightValue = (127 * backlightPercent) / 50 + 1;
  analogWrite(backlightPin, backlightValue);
  */

  dayLightVal = analogRead(A10);
  backlightValue = 0.2138*dayLightVal + 20.5;
  analogWrite(backlightPin, backlightValue);
}

void temperature()
{
  int sensorValue = analogRead(A11);
  float voltage = sensorValue * (5.0 / 1023.0);
  float v6 = voltage * voltage * voltage * voltage * voltage * voltage;
  float v5 = voltage * voltage * voltage * voltage * voltage;
  float v4 = voltage * voltage * voltage * voltage;
  float v3 = voltage * voltage * voltage;
  float v2 = voltage * voltage;

  temp = (-1.4591 * v6 + 21.784 * v5 - 125.74 * v4 + 350.06 * v3 - 473.05 * v2 + 216.06 * voltage + 267.96) * 0.93;
  // polynomial approximation of signal voltages vs temperature of temp sender with experimentally determined correction factor of 0.93.
  // see excel sheet for more info.       C:\Users\Max\Google Drive\Sync\Bub\WVO Controller\Oil Temp gage values
  total = total - readings[index];              // read from the sensor:
  readings[index] = temp;                       // add the reading to the total:
  total = total + readings[index];              // advance to the next position in the array:
  index = index + 1;                            // if we're at the end of the array...
  if (index >= numReadings)                     // ...wrap around to the beginning:
    index = 0;                                  // calculate the average:
  average = total / numReadings;
}

//float getVoltage(int pin)				        // function that reads the value of the temperature pin and returns it
// to be converted into a temperature value.
//{
//return(analogRead(pin)*0.004882814);
//}

/*
***********************************************************************************************************************************************
                                        END OF CODE
***********************************************************************************************************************************************
*/

