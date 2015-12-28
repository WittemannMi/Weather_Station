/*
 **********************************
 *** Program by M. Wittemann*******
 ****Projekt started November 2015*
 **********************************
 *
--------------------------------------------------------------------------------------
  Pinout & Description
--------------------------------------------------------------------------------------
                                      +-----+
         +----[PWR]-------------------| USB |--+
         |                            +-----+  |
         |         GND/RST2  [ ][ ]            |
         |       MOSI2/SCK2  [ ][ ]  A5/SCL[ ] |   C5 
         |          5V/MISO2 [ ][ ]  A4/SDA[ ] |   C4 
         |                             AREF[ ] |
         |                              GND[ ] |
         | [ ]N/C                    SCK/13[ ] |   B5
         | [ ]v.ref                 MISO/12[ ] |   .
         | [ ]RST                   MOSI/11[ ]~|   .
         | [ ]3V3    +---+               10[ ]~|   .
         | [ ]5v     | A |                9[ ]~|   .
         | [ ]GND   -| R |-               8[ ] |   B0
         | [ ]GND   -| D |-                    |
         | [ ]Vin   -| U |-               7[ ] |   D7
         |          -| I |-               6[ ]~|   .
         | [ ]A0    -| N |-               5[ ]~|   .
         | [ ]A1    -| O |-               4[ ] |   .
         | [ ]A2     +---+           INT1/3[ ]~|   .
         | [ ]A3                     INT0/2[ ] |   .
         | [ ]A4/SDA  RST SCK MISO     TX>1[ ] |   .
         | [ ]A5/SCL  [ ] [ ] [ ]      RX<0[ ] |   D0
         |            [ ] [ ] [ ]              |
         |  UNO_R3    GND MOSI 5V  ____________/
          \_______________________/

Pin description:

 --digital Pins--:
 0: to Tx from BT
 1: to Rx from BT
 2: Int0 for Windrad
 3: TEMP_SENS_PIN
 4: LCD-DB4
 5: LCD-DB5
 6: LCD-DB6
 7: LCD-DB7
 8: LCD-RS 
 9: LCD-Enable
10: LCD-Backlight
11: SoftSerial Rx
12: SoftSerial Tx
13: 

 --analog pins--:
 A0: LCD_Buttons
 A1: 
 A2:
 A3:
 A4:
 A5:

 ADC voltages for the 5 buttons on analog input pin A0:
  
    RIGHT:  0.00V :     0 @ 10 bit
    UP:     0.71V :   145 @ 10 bit
    DOWN:   1.61V :   330 @ 10 bit
    LEFT:   2.47V :   506 @ 10 bit
    SELECT: 3.62V :   742 @ 10 bit
 

/*--------------------------------------------------------------------------------------
  Includes
--------------------------------------------------------------------------------------*/
#include <LiquidCrystal.h>
#include "DHT.h"
#include <SoftwareSerial.h>

/*--------------------------------------------------------------------------------------
  Init the LCD library with the LCD pins to be used, DHT sesor and serial
--------------------------------------------------------------------------------------*/
LiquidCrystal lcd(8, 9, 4, 5, 6, 7);

DHT dht(3, DHT11); // Initialize DHT sensor. first parameter is the pin, second is the model

SoftwareSerial mySerial(11, 12); // Initialize softSerial Pins 11=RX, 12=TX

/*--------------------------------------------------------------------------------------
  Defines
--------------------------------------------------------------------------------------*/
//PINS:
#define BUTTON_ADC_PIN            A0  // A0 is the button ADC input
#define LCD_BACKLIGHT_PIN         10  // D10 controls LCD backlight 

#define WIND_SENS_PIN             2   // INTO to measure Wind speed
#define TEMP_SENS_PIN             3   // D3 to read data from DHT11

//for Key switch function
#define btnRIGHT  0
#define btnUP     1
#define btnDOWN   2
#define btnLEFT   3
#define btnSELECT 4
#define btnNONE   5

// for LCD Menu Array size
#define LCD_ROWS 5
#define LCD_COLLUMS 3


/*--------------------------------------------------------------------------------------
  global Variables
--------------------------------------------------------------------------------------*/

// For Buttons
int adc_key_in  = 0;

//for Anenometer
const float windFactor = 2.4; //Factor = 2,4 km/h per RPM
const int measureTime = 3;
volatile unsigned int windCounter = 0;
float windSpeed = 0.0;
volatile unsigned long alteZeit=0;


int PinState = 0;         // current state of the Windpin
int lastPinState = 0;     // previous state of the Windpin


int lcd_menu_change = 1234;  //  To check for menu changes, intialize with a value thats not possible the first time arround

float max_temp =0, min_temp = 100;  //for min/max temparature, Set min value to 100 for first compare with actual value
float max_humidity = 0, min_humidity = 1000;    //for min/max humidity, set min value to 100 for first compare with actual value
float max_wind_value =0;  // to store max wind value

float temp_value = 0;    // actual tempereate value
float humidity_value =0; // actual humidity value

//--LCD Menu--

int row, collum = 0;    // to count the state in the menu matrix
int MENU [LCD_ROWS][LCD_COLLUMS] = {{00, 01, 02},{10,11,12},{20,21,22},{30,31,32},{40,41,42}};   // 5 rows and 3 collums for this application, set as defines above, defines for the menues maybe in the future
int last_lcd_key = 1234;    // initialize to impossible value for the first compare
unsigned long oldMenutime = 0;   // to "debouce" button presses to not go to the menues to quick 
unsigned long oldbtnSELECTtime =0;  // to calculate the 3 seconds time to delete the strored values
boolean firstpress = true;   // to detect the start of a delete attemt
int menu_state;   // the current state of the menu to be displayed 


/*--------------------------------------------------------------------------------------
  Functions
--------------------------------------------------------------------------------------*/

void get_max_min_temp(float temp)  // Store maximum and minumum temparture values ever
{
 if(temp > max_temp)
   {
    max_temp = temp; // Save the biggest temerature value 
   }
  if(min_temp > temp)
   {
    min_temp = temp; // Save the biggest temerature value 
   }
}

void get_max_min_humidity(float humidity)  // Store maximum and minumum temparture values ever
{
 if(humidity > max_humidity)
   {
    max_humidity = humidity; // Save the biggest temerature value 
   }
  if(min_humidity > humidity)
   {
    min_humidity = humidity; // Save the biggest temerature value 
   }
}

void get_max_wind(float wind)   // Store maximum wind speed ever
{
 if(wind > max_wind_value)
   {
    max_wind_value = wind; // Save the biggest temerature value 
   }
}

void read_WIND_Speed()            // read Wind counts via GPIO and calculate Wind Speed
{
  PinState = digitalRead(WIND_SENS_PIN);    // read the pushbutton input pin

  if (PinState != lastPinState)     // compare the buttonState to its previous state
  {
    if (PinState == HIGH)    // if the current state is HIGH then the button wend from off to on
    {
      windCounter++;    // if the state has changed, increment the counter
    }
   else   // if the current state is LOW then the button wend from on to off:
   {
    // Do nothing
   }
    delay(20);  // Delay a little bit to avoid bouncing
  }
  lastPinState = PinState; // save the current state as the last state,for next time through the loop

 int messZeit= 1000 * measureTime;          // Measurement window for the calulation of the Reed contact
 if((millis() - alteZeit) >= messZeit)      // only if measurement time is reached calculate a new value
 { 
    windSpeed = (float)windCounter / measureTime * windFactor;
    alteZeit = millis(); // remember last time arround   
    windCounter = 0;   //reset Wind Counter for next measurement  
 }
  
}

int read_LCD_buttons()    // read the buttons
{                   
    adc_key_in = analogRead(BUTTON_ADC_PIN);       // read the value from the sensor 
 
    // buttons  are centered at these values: 0, 145, 330, 506, 742
    // add approx 50 to those values and check to see if we are close
    
    // We make this the 1st option for speed reasons since it will be the most likely result
    if (adc_key_in > 1000)
    {
      return btnNONE; 
    }
    if (adc_key_in < 50)
    {
      return btnRIGHT;  
    }
    if (adc_key_in < 200)
    {
      return btnUP; 
    }
    if (adc_key_in < 400) 
    {
      return btnDOWN; 
    }
    if (adc_key_in < 550)
    {
      return btnLEFT; 
    }
    if (adc_key_in < 800) 
    {
      return btnSELECT;  
    }
    return btnNONE;                // when all others fail, return this.
}

void readButtons()
{
 int lcd_key = 0;
 unsigned long MenuTime=1000;       // wait for 1 sec before read the next botton press
 lcd_key = read_LCD_buttons();         // read the buttons

  switch (lcd_key)  // Button Menue to set the values in the LCD Array
  {               
       case btnRIGHT:    
       {
        if((millis() - oldMenutime) >= MenuTime)              // only if the 1 second wait is over switch to the next menu
        { 
         collum++;
         oldMenutime = millis(); // remember last time arround      
        }             
        if(collum > 2)   
        {
         collum = 0;  // Avoid overflow...
        }
        if(row == 4)  //...and in the special case of the delete menu dont go right with a button press
        {
          collum = 0;
        }
        break;
       }
       case btnLEFT:
       {
        if((millis() - oldMenutime) >= MenuTime)              // only if the 1 second wait is over switch to the next menu
        { 
         collum--;
         oldMenutime = millis(); // remember last time arround      
        } 
        if(collum < 0)
        {
         collum = 0;
        }
       break;
       }    
       case btnUP:
       {
        if((millis() - oldMenutime) >= MenuTime)              // only if the 1 second wait is over switch to the next menu
        { 
         row--;
         collum = 0; //If up down is pressed in Sub Menu go to main menu
         oldMenutime = millis(); // remember last time arround      
        }  
        if(row < 0)
        {
          row = 4;
        }
        break;
       }
       case btnDOWN:
       {
        if((millis() - oldMenutime) >= MenuTime)              // only if the 1 second wait is over switch to the next menu
        { 
         row++;
         collum = 0; //If up down is pressed in Sub Menu go to main menu
         oldMenutime = millis(); // remember last time arround      
        } 
        if(row > 4)
        {
          row=0;
        }
        break;
       }
       case btnSELECT:
       { 
        if(firstpress == true)
        {
         oldbtnSELECTtime = millis(); // Save last time here
         firstpress = false; 
        }
        if(((millis() - oldbtnSELECTtime) >= 3000) && (menu_state == 40))             // Delete values only if button is pressed for 3 seconds and we are in the delete menu
        { 
         //delete min/Max values here
         max_temp =0;
         min_temp =100;
         max_humidity = 0;
         min_humidity = 1000;    
         max_wind_value =0;
         oldbtnSELECTtime = millis(); // Save last time here   
         firstpress = true;   // set firstpress for next go arround
         collum++;  // show confirmation screen
        } 
        break;
       }
       case btnNONE:
       {
        firstpress = true;  // set firstpress for delete with select key if the botton was not pressed for 3 seconds
       }
   }
}

void showMenu()
{
  menu_state = MENU [row][collum];
        //Serial.println("Menu:");
        //Serial.println(menu_state);
  switch(menu_state)
  {
    case 00:
    {
    if(menu_state != lcd_menu_change)
     {
      clearLCD();
      lcd_menu_change = menu_state;
     }
      lcd.setCursor(0,0);             // set the LCD cursor   position  
      lcd.print("Arduino for FUN");
      lcd.setCursor(0,1);             // move to the begining of the second line  
      lcd.print("Weather Station");
     break;
    }
    case 01:
    {
     if(menu_state != lcd_menu_change)
     {
      clearLCD();
      lcd_menu_change = menu_state;
     }
     lcd.setCursor(0,0);             // set the LCD cursor   position 
     lcd.print("Control: ");
     lcd.setCursor(0,1);             // move to the begining of the second line  
     lcd.print("UP DOWN <- ->");
     break;
    }
    case 02:
    {
     if(menu_state != lcd_menu_change)
     {
      clearLCD();
      lcd_menu_change = menu_state;
     }
     lcd.setCursor(0,0);             // set the LCD cursor   position 
     lcd.print("By M.Wittemann");
     lcd.setCursor(0,1);             // move to the begining of the second line  
     lcd.print("Rev. 1.1");
     break;
    }
    case 10:
    {
     if(menu_state != lcd_menu_change)
     {
      clearLCD();
      lcd_menu_change = menu_state;
     }
     lcd.setCursor(0,0);             // set the LCD cursor   position 
     lcd.print("Wind Speed:");
     lcd.setCursor(0,1);             // move to the begining of the second line  
     lcd.print(windSpeed);
     lcd.print(" km/h");
     break;
    }
    case 11:
    {
     if(menu_state != lcd_menu_change)
     {
      clearLCD();
      lcd_menu_change = menu_state;
     }
     lcd.setCursor(0,0);             // set the LCD cursor   position 
     lcd.print("Max Wind Speed:");
     lcd.setCursor(0,1);             // move to the begining of the second line  
     lcd.print(max_wind_value);
     lcd.print(" km/h");
     break;
    }
    case 12:
    {
     if(menu_state != lcd_menu_change)
     {
     /* clearLCD();
      lcd_menu_change = menu_state;*/
     }
     collum--;
     break;
    }
    case 20:
    {
     if(menu_state != lcd_menu_change)
     {
      clearLCD();
      lcd_menu_change = menu_state;
     }
     lcd.setCursor(0,0);             // set the LCD cursor   position 
     lcd.print("Temperature:");
     lcd.setCursor(0,1);             // move to the begining of the second line  
     lcd.print(temp_value); 
     lcd.print(" *C"); 
     break;
    }
    case 21:
    {
     if(menu_state != lcd_menu_change)
     {
      clearLCD();
      lcd_menu_change = menu_state;
     }
     lcd.setCursor(0,0);             // set the LCD cursor   position 
     lcd.print("MAX Temperature:"); 
     lcd.setCursor(0,1);             // move to the begining of the second line  
     lcd.print(max_temp);
     lcd.print("*C"); 
     break;
    }
    case 22:
    {
     if(menu_state != lcd_menu_change)
     {
      clearLCD();
      lcd_menu_change = menu_state;
     }
     lcd.setCursor(0,0);             // set the LCD cursor   position 
     lcd.print("MIN Temperature:"); 
     lcd.setCursor(0,1);             // move to the begining of the second line  
     lcd.print(min_temp);
     lcd.print("*C"); 
     break;
    }
    case 30:
    {
     if(menu_state != lcd_menu_change)
     {
      clearLCD();
      lcd_menu_change = menu_state;
     }
     lcd.setCursor(0,0);             // set the LCD cursor   position 
     lcd.print("Humidity:"); 
     lcd.setCursor(0,1);             // move to the begining of the second line  
     lcd.print(humidity_value);
     lcd.print("%");
     break;
    }
    case 31:
    {
     if(menu_state != lcd_menu_change)
     {
      clearLCD();
      lcd_menu_change = menu_state;
     }
     lcd.setCursor(0,0);             // set the LCD cursor   position 
     lcd.print("MAX Humidity:"); 
     lcd.setCursor(0,1);             // move to the begining of the second line  
     lcd.print(max_humidity);               //Min max noch berechnen
     lcd.print("%");
     break;
    }
    case 32:
    {
     if(menu_state != lcd_menu_change)
     {
      clearLCD();
      lcd_menu_change = menu_state;
     }
     lcd.setCursor(0,0);             // set the LCD cursor   position 
     lcd.print("MIN Humidity:"); 
     lcd.setCursor(0,1);             // move to the begining of the second line  
     lcd.print(min_humidity);               //Min max noch berechnen
     lcd.print("%");
     break;
    }
    case 40:
    {
     if(menu_state != lcd_menu_change)
     {
      clearLCD();
      lcd_menu_change = menu_state;
     }
     lcd.setCursor(0,0);             // set the LCD cursor   position 
     lcd.print("Reset MAX/MIN:"); 
     lcd.setCursor(0,1);             // move to the begining of the second line  
     lcd.print("Press Sel-> 3Sec");               //Min max noch berechnen
     break;
    }
    case 41:
    {
     if(menu_state != lcd_menu_change)
     {
      clearLCD();
      lcd_menu_change = menu_state;
     }
     lcd.setCursor(0,0);             // set the LCD cursor   position 
     lcd.print("MIN/MAX Values..."); 
     lcd.setCursor(0,1);             // move to the begining of the second line  
     lcd.print("...are RESET");               //Min max noch berechnen
     break;
    }
    case 42:
    {
     if(menu_state != lcd_menu_change)
     {
     /* clearLCD();
      lcd_menu_change = menu_state;*/
     }
     collum--;
     break;
    }
    default:
    {
     //Do nothing
    }
  }
}

void clearLCD()      // Clear the LCD 
{
 lcd.setCursor(0,0);             // set the LCD cursor   position    
 lcd.print("                ");
 lcd.setCursor(0,1);             // set the LCD cursor   position    
 lcd.print("                ");
}

void BTcomm()   // test function
{
 int state = 0;
 int flag = 0;        // make sure that you return the state only once
 
 if(mySerial.available() > 0)  //if some data is sent, read it and save it in the state variable
 {
  state = mySerial.read();
  flag=0;
 }
 if (state == '0')  // if the state is 0 the led will turn off
 {
  digitalWrite(LCD_BACKLIGHT_PIN, LOW);
  if(flag == 0)
  {
   mySerial.println("Backlight: off");
   flag = 1;
  }
 }
 // if the state is 1 the led will turn on
 else if (state == '1')
 {
  digitalWrite(LCD_BACKLIGHT_PIN, HIGH);
  if(flag == 0)
  {
   mySerial.println("Backlight: on");
   flag = 1;
  }
 } 
}
/*--------------------------------------------------------------------------------------
  setup()
  Called by the Arduino framework once, before the main loop begins
--------------------------------------------------------------------------------------*/

void setup() {
  
   lcd.begin(16, 2);   // set up the LCD's number of columns and rows and start the library

   dht.begin();   // Setup the Temperaute and humidity sensor
   
   Serial.begin(9600);   // Setup the serial debug connection
   
   pinMode(LCD_BACKLIGHT_PIN,OUTPUT);
   digitalWrite(LCD_BACKLIGHT_PIN,HIGH);
   pinMode(WIND_SENS_PIN, INPUT);

   mySerial.begin(9600);
}

/*--------------------------------------------------------------------------------------
  Arduino main loop
--------------------------------------------------------------------------------------*/
void loop()
{
/*--------------------------------------------------------------------------------------
  local Variables
--------------------------------------------------------------------------------------*/

//--------------Function calls------------------------------------
    
   temp_value = dht.readTemperature(); //read Temparature value from sensor
   humidity_value = dht.readHumidity(); //read humidity value from sensor
   read_WIND_Speed();                   // read wind speed from GPIO Pin
       //read_WIND_Speed_INT();             // read wind speed with Interrupt function, which one is better???
       //lcd_key = read_LCD_buttons();         // read the buttons , weg für Menu test

   get_max_min_temp(temp_value);         // check for min and max temperature values
   get_max_wind(windSpeed);              // Check for max speed
   get_max_min_humidity(humidity_value); // check for min and max humidity values

   readButtons();
   showMenu();

   BTcomm();      // Send and receive data via HC06 BT module
}

// ---->End of Code

/*/Debug
              Serial.println(lcd_key);
              Serial.println(lcd_menu_change);
              */

/* -----Unsued functions-----

void read_WIND_Speed_INT()            // Einlesen der Wind counts über ISR und umrechnen in Wind Speed
{
   //Werte für den Windmesser
   windCounter = 0;
   attachInterrupt(digitalPinToInterrupt(WIND_SENS_PIN), countWind, RISING);    //Zähl-Interrupt aktiviere
   delay(1000 * measureTime);    //Abwarten Messzeitraums
   detachInterrupt(digitalPinToInterrupt(WIND_SENS_PIN));    //Zähl-Interrupt deaktivieren

   windSpeed = ((float)windCounter / (float)measureTime) * windFactor;

}


/*--------------------------------------------------------------------------------------
  Interrupt Service Routine
--------------------------------------------------------------------------------------

void countWind() 
{
 volatile unsigned long alteZeit=0, entprellZeit=20;   // Zum entprellen des READ Kontaktes
 if((millis() - alteZeit) > entprellZeit)              // nur wenn Entprellzeit überschritten wieder einen Schaltvorgang zählen
 { 
    windCounter ++;
    alteZeit = millis(); // letzte Schaltzeit merken      
 }
   //Serial.println("***windCounter***");
   //Serial.println(windCounter);
}
 
 */

