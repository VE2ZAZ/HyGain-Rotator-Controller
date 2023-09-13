/*
  _  _  ___  ___  ___   __   ___     _   _  _  __   ___  __  _    _ 
 ( )( )(  _)(__ \(_  ) (  ) (_  )   / ) ( )( )(  ) (__ \(  )( \/\/ )
  \\//  ) _)/ __/ / /  /__\  / /   / /   \\// /__\ / __/ )(  \    / 
  (__) (___)\___)(___)(_)(_)(___) (_/    (__)(_)(_)\___)(__)  \/\/                                                                                          

  Modern HyGain Rotator Controller, Version 3, September 2023.
  Designed to be run on a Raspberry Pi Pico. Developed in Arduino IDE.
  Author: Bertrand Zauhar VE2ZAZ / VA2IW,  https://ve2zaz.net
  Applicable License: Attribution 4.0 International (CC BY 4.0)  Lisez https://creativecommons.org/licenses/by/4.0/deed.fr  
*/

// COMPILER DIRECTIVES

#include "Adafruit_ILI9341.h"
#include "Adafruit_GFX.h"
#include <XPT2046_Touchscreen.h>

// CONSTANTS
// Pins used on the Rasprerry Pi Pico. Numbers correspond to GPxx pin assignment.
#define TFT_DC 21
#define TFT_CS 17
#define TFT_MOSI 19
#define TFT_CLK 18
#define TFT_RST 20
#define TFT_MISO 16
#define TCS_PIN  13
#define TIRQ_PIN 14
// Other constants
#define BREAK_RELAY 10
#define CW_ROTATION_RELAY 11
#define CCW_ROTATION_RELAY 12
#define POT_CURSOR 26
#define POT_MAX 27
#define POT_MIN 28
#define ENCODER_A 2
#define ENCODER_B 3
#define ENCODER_PUSHBUTTON 4
#define DISPLAY_LIGHTING_CTRL 5
#define SLOWDOWN_RESIDUAL_ROTATION 2      // Residual rotation (in degrees) after the motor supply is cut
#define POT_NUMBER_SAMPLE 200
#define CIRCLE_CENTER_X 239
#define CIRCLE_CENTER_Y 81
#define CIRCLE_RADIUS 55
#define IDLE_COLOR ILI9341_GREEN
#define ROTATION_COLOR ILI9341_MAGENTA
#define SLOWDOWN_COLOR ILI9341_YELLOW


// CLASS INSTANTIATIONS
// Built-in SPI (hardware port) of R-Pi Pico for the display. Position for CS, DC and RST pins must be specified.
Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC,TFT_RST); 

// Tactile (touch) function is implemented using the IRQ pin
XPT2046_Touchscreen ts(TCS_PIN, TIRQ_PIN);            // Param 2 - Touch IRQ Pin - interrupt enabled polling


// GLOBAL VARIABLES
int current_heading;
int instant_heading;
int target_heading = -1;
int previous_heading_verif = 0;
int previous_heading_disp = 0;
unsigned long int start_slowdown_ongoing_millis = 0; 
unsigned long int start_rotation_verif_millis = 0; 
unsigned long int start_display_refresh_millis = 0;
boolean ongoing_rotation_verif = false;
boolean azim_refresh_req = true;
boolean active_alarm = false;
boolean cw_rotation = false;
String number = "";
String command_str = "";
double avg_px = 0;
double avg_py = 0;
int avg_count = 0;
boolean ongoing_rotation = false;
boolean ongoing_slowdown = false;
boolean rotation_req = false;
boolean slowdown_req = false;
boolean stopping_req = true;
boolean encoder_logic_level;
boolean encoder_previous_logic_level;
unsigned long int start_display_count_millis = 0;
boolean heading_avail = false;
String rx_command = "";
String current_heading_txt = "";


// Text conversion and formatting function. Stuffs "0" characters ahead of number to make it three wide.
String int_stuff_zeros_str(int a)
{
  if (a >= 10 && a < 100) return "0" + String(a);         // Add "0" if the number is between 10 and 99 and return the string result
  else if (a < 10) return "00" + String(a);               // Add "00" if the number is between 1 and 9 and return the string result
  else return String(a);                                  // Otherwise add no zeros and return the string result
}


// Setup function of the Pico's first core, Arduino style, run only once at power on
void setup() 
{
  pinMode(TCS_PIN,OUTPUT);                                // Ensure that the CS pins are at high level to begin with
  digitalWrite(TCS_PIN,HIGH);                             //    "
  pinMode(TFT_CS,OUTPUT);                                 //    "
  digitalWrite(TFT_CS,HIGH);                              //    "
  pinMode(BREAK_RELAY,OUTPUT);                            // Same for the relay pins 
  digitalWrite(BREAK_RELAY,HIGH);                         //    "
  pinMode(CW_ROTATION_RELAY,OUTPUT);                      //    "
  digitalWrite(CW_ROTATION_RELAY,HIGH);                   //    "
  pinMode(CCW_ROTATION_RELAY,OUTPUT);                     //    " 
  digitalWrite(CCW_ROTATION_RELAY,HIGH);                  //    "
  pinMode(ENCODER_A,INPUT_PULLUP);                        // Initialize the encoder pins as inputs
  pinMode(ENCODER_B,INPUT_PULLUP);                        //    "
  pinMode(ENCODER_PUSHBUTTON,INPUT_PULLUP);               //    "
  pinMode(DISPLAY_LIGHTING_CTRL,OUTPUT);                  // Initialize the display lighting control pin as an output and turn on lighting
  digitalWrite(DISPLAY_LIGHTING_CTRL,HIGH);               //    "
 
  tft.begin(24000000);                                    // Launch the display instance
  tft.setRotation(3);                                     // Rotate display info by 270 degrees
  ts.begin();                                             // Launch the touch instance

  tft.fillScreen(ILI9341_BLACK);                          // Display splash screen
  tft.setTextSize(2);                                     //    "
  tft.setTextColor(ILI9341_GREEN);
  tft.setCursor(10,50);
  tft.print("HyGain Rotator Controller");
  tft.setTextColor(ILI9341_WHITE);
  tft.setCursor(10,90);
  tft.print("  Version 3 - 09/2023");
  tft.setCursor(10,110);
  tft.print("   by VE2ZAZ/VA2IW");
  tft.setCursor(10,130);
  tft.print("  https://ve2zaz.net");
  while (!Serial && (millis() <= 5000));                  // A pause of a few seconds for user to read

  while (!heading_avail);                                 // Wait for current heading availability before continuing
  current_heading = instant_heading;                      // Initialize the heading variables
  previous_heading_disp = current_heading;                //    "
  heading_avail = false;                                  //    "

  //Drawing the compass
  tft.fillScreen(ILI9341_BLACK);    
  tft.drawCircle(CIRCLE_CENTER_X, CIRCLE_CENTER_Y, CIRCLE_RADIUS, ILI9341_YELLOW);
  tft.drawLine(CIRCLE_CENTER_X, CIRCLE_CENTER_Y, CIRCLE_CENTER_X + (CIRCLE_RADIUS-1) * sin(float(current_heading)/180*3.1416), CIRCLE_CENTER_Y - (CIRCLE_RADIUS-1) * cos(float(current_heading)/180*3.1416), IDLE_COLOR);
  tft.setTextSize(2);      
  tft.setTextColor(ILI9341_WHITE);
  tft.setCursor(CIRCLE_CENTER_X-5, CIRCLE_CENTER_Y-CIRCLE_RADIUS-18);
  tft.print("N");
  tft.setCursor(CIRCLE_CENTER_X-5, CIRCLE_CENTER_Y+CIRCLE_RADIUS+6);
  tft.print("S");
  tft.setCursor(CIRCLE_CENTER_X-CIRCLE_RADIUS-15, CIRCLE_CENTER_Y-7);
  tft.print("O");
  tft.setCursor(CIRCLE_CENTER_X+CIRCLE_RADIUS+6, CIRCLE_CENTER_Y-7);
  tft.print("E");
  for (int i=0;i<360;i+=30)                               // Drawing of degree tick marks around the compass
  {
    tft.drawLine(CIRCLE_CENTER_X + (CIRCLE_RADIUS) * sin(float(i)/180*3.1416), CIRCLE_CENTER_Y - (CIRCLE_RADIUS) * cos(float(i)/180*3.1416), CIRCLE_CENTER_X + (CIRCLE_RADIUS+5) * sin(float(i)/180*3.1416), CIRCLE_CENTER_Y - (CIRCLE_RADIUS+5) * cos(float(i)/180*3.1416), ILI9341_WHITE);
  }

  // Drawing the keypad 
  tft.setTextSize(3);      
  tft.setTextColor(ILI9341_WHITE);
  for (int i = 16; i <= 160;i+=36)                          // Drawing of blue keaypad buttons
  {
    tft.fillRect(i-5, 162, 33, 33, ILI9341_BLUE);
    tft.fillRect(i-5, 199, 33, 33, ILI9341_BLUE);
  }
  for (int i = 0; i <= 4;i++)                               // Drawing of numbers from 0 to 9
  {
    tft.setCursor(20 + i*36, 169);
    tft.print(i);
    tft.setCursor(20 + i*36, 205);
    tft.print(i+5);
  }
  for (int i = 196; i <= 260;i+=62)                         // Drawing of remaining button text
  {                                                         //      "
    tft.fillRect(i-5, 162, 59, 33, ILI9341_BLUE);
    tft.fillRect(i-5, 199, 59, 33, ILI9341_BLUE);
  }
  tft.setTextColor(ILI9341_GREEN);
  tft.setCursor(203, 169);
  tft.print("GO");
  tft.setTextColor(ILI9341_ORANGE);
  tft.setCursor(195, 205);
  tft.print("CLR");
  tft.setTextColor(ILI9341_WHITE);
  tft.setCursor(256, 169);
  tft.print("+15");
  tft.setCursor(256, 205);
  tft.print("-15");

  tft.setTextColor(IDLE_COLOR);                             // Drawing of text indicating current rotator state
  azim_refresh_req = true;
  encoder_previous_logic_level = digitalRead(ENCODER_A);    // Encoder initial logic level reading
}


// Setup function of the Pico's second core, Arduino style, run only once at power on
void setup1() 
{
  pinMode(POT_CURSOR,INPUT);                                // Configure ADC pins as inputs
  pinMode(POT_MAX,INPUT);                                   //     "
  pinMode(POT_MIN,INPUT);                                   //     "
  analogReadResolution(12);                                 // Configure ADC resolution to 12 bits
  Serial.begin(4800);                                       // Open serial (USB) port
}


// // Main loop function of the Pico's first core, Arduino style, runs forever
void loop() 
{
  if (heading_avail)                                                                  // New current heading available?
  {
    current_heading = instant_heading;                                                // Update instantaneous heading.
    heading_avail = false;
  }

  // Display refreshing
  if ((unsigned long)(millis() - start_display_refresh_millis) >= 100)                // Display refresh every 100 milliseconds
  {  
    start_display_refresh_millis = millis();                                          // Initializa a new refresh cycle
    if ((previous_heading_disp != current_heading) || (azim_refresh_req))             // Update current heading on cisplay if necessary
    {
      current_heading_txt = int_stuff_zeros_str(current_heading);                     // Add leading zeros and convert to string
      String previous_heading_disp_txt = int_stuff_zeros_str(previous_heading_disp);  // Save current heading for the next display refresh cycle
      for (int i=0;i<=2;i++)                                                          // Update displayed 3-digit heading 
      {
        if ((current_heading_txt[i] != previous_heading_disp_txt[i]) || (azim_refresh_req))   // update of one character only if necessary (reduces flashing)
        {
          tft.setTextSize(8);      
          tft.setCursor(15 + 50*i, 20);                                               // Erase old character (with black font)
          tft.setTextColor(ILI9341_BLACK);                                            //      "
          tft.print(previous_heading_disp_txt[i]);                                    //      "
          tft.setCursor(15 + 50*i, 20);                                               // Write new character with proper color (as a funciton of current rotation phase)  
          if (ongoing_rotation) tft.setTextColor(ROTATION_COLOR);                     //      "
          else if (ongoing_slowdown) tft.setTextColor(SLOWDOWN_COLOR);                //      "
          else tft.setTextColor(IDLE_COLOR);                                          //      "
          tft.print(current_heading_txt[i]);                                          //      "
        }  
      }
      // Compass updating
      tft.drawCircle(CIRCLE_CENTER_X, CIRCLE_CENTER_Y, CIRCLE_RADIUS, ILI9341_YELLOW);     
      tft.drawLine(CIRCLE_CENTER_X, CIRCLE_CENTER_Y, CIRCLE_CENTER_X + (CIRCLE_RADIUS-1) * sin(float(previous_heading_disp)/180*3.1416), CIRCLE_CENTER_Y - (CIRCLE_RADIUS-1) * cos(float(previous_heading_disp)/180*3.1416), ILI9341_BLACK);    // Effacement de l'ancienne aiguille
      tft.drawLine(CIRCLE_CENTER_X, CIRCLE_CENTER_Y, CIRCLE_CENTER_X + (CIRCLE_RADIUS-1) * sin(float(current_heading)/180*3.1416), CIRCLE_CENTER_Y - (CIRCLE_RADIUS-1) * cos(float(current_heading)/180*3.1416), (ongoing_rotation || ongoing_slowdown ? ROTATION_COLOR : IDLE_COLOR));  // Dessin de la nouvelle aiguille avec la bonne couleur   
    }
    if (active_alarm && ongoing_rotation)                           // Alarm message erase required?
    {  // Yes
      tft.fillRect(20, 120, 130, 40, ILI9341_BLACK);                // Erasing of alarm message
      active_alarm = false;                                         // Clearing of the alarm itself
    }
    azim_refresh_req = false;                                       // Clear the displayed heading refresh required flag
    previous_heading_disp = current_heading;                        // Update the previous heading variable in preparation for the next refresh cycle
  }

  // Touch action processing
  if (ts.touched())                                                 // Display touched?
  {                                                                 // Yes
    if (!digitalRead(DISPLAY_LIGHTING_CTRL))                        // Display lighting off?
    {                                                               // Yes
      digitalWrite(DISPLAY_LIGHTING_CTRL,HIGH);                     // Turn display lighting on
      while (ts.touched());                                         // Wait for touch release
    }
    start_display_count_millis = millis();
    active_alarm = false;                                           // Clear the active alarm flag
    while (ts.touched())                                            // A display touch is happening ?
    {                                                               // Yes
      TS_Point p = ts.getPoint();                                   // Get the touch coordinates
      avg_px += p.x;                                                // Calculate an average of the coordinates (improves accuracy)
      avg_py += p.y;                                                //     "
      avg_count++;                                                  //     "      
    }
    delay(100);                                                     // Delay required for debouncing
    avg_px = round(float(avg_px)/avg_count);                        // Complete the average calculation
    avg_py = round(float(avg_py)/avg_count);                        //     "

    // Keypad decoding of selected button vs. x and y coordinates
    if (avg_py > 2700 && avg_py <= 3250)                            // Decoding of the keypad row
    {
      if      (avg_px > 150 && avg_px <= 600)   number += "0";      // Decoding of the keypad column
      else if (avg_px > 600 && avg_px <= 1050)  number += "1";      //     "
      else if (avg_px > 1050 && avg_px <= 1500) number += "2";
      else if (avg_px > 1500 && avg_px <= 1950) number += "3";
      else if (avg_px > 1950 && avg_px <= 2400) number += "4";
      else if (avg_px > 2400 && avg_px <= 3100) command_str = "GO";
      else if (avg_px > 3100 && avg_px <= 3900) command_str = "+15";
    }
    else if (avg_py > 3250 && avg_py <= 3800)                       // Decoding of the keypad row
    {
      if      (avg_px > 150 && avg_px <= 600)   number += "5";      // Decoding of the keypad column
      else if (avg_px > 600 && avg_px <= 1050)  number += "6";      //     "
      else if (avg_px > 1050 && avg_px <= 1500) number += "7";
      else if (avg_px > 1500 && avg_px <= 1950) number += "8";
      else if (avg_px > 1950 && avg_px <= 2400) number += "9";
      else if (avg_px > 2400 && avg_px <= 3100) command_str = "CLR";
      else if (avg_px > 3100 && avg_px <= 3900) command_str = "-15";
    }
    avg_px = 0;                                                     // Clear the average variables in preparation for the next touch cycle    
    avg_py = 0;                                                     //     "
    avg_count = 0;                                                  //     "
    tft.fillRect(20, 120, 130, 40, ILI9341_BLACK);                  // Erase the old character with a black rectangle
    tft.setTextColor(ILI9341_GREEN);                                // white new character corresponding to decoded keypad button
    tft.setTextSize(4);                                             //     "
    tft.setCursor(20,120);                                          //     "
    tft.print(number);                                              //     "
    if ((number.length() > 3) || (number.toInt() > 360)) command_str = "CLR";     // If keyed in heading is not valid, clear the new written characters
  
    if (command_str == "GO")                                        // Is GO key decoded?
    {   
      if (number.length() != 0)                                     // Is a new heading available to be processed
      {                                                             // Yes
        target_heading = number.toInt();                            // Save the new heading as the target heading to be processed
        rotation_req = true;                                        // Launche a new rotation
        number = "";                                                // Clear variables for the next touch cycle
        command_str = "";
      }
    }
    else if (command_str == "CLR")                                  // Is CLR key decoded?
    {
      if ((!ongoing_rotation) && (!ongoing_slowdown))               // Is there a rotation in progress (and not in slowdown)? 
      {                                                             // No
        tft.fillRect(20, 120, 130, 40, ILI9341_BLACK);              // Erase the keyed in heading and variables rotation
        number = "";                                                //     "
        command_str = "";
        target_heading = -1;
      }
      else if (ongoing_rotation)                                     // Orientation in progress?
      {                                                              // Yes
        slowdown_req = true;                                         // Request for a rotation interruption
        ongoing_rotation_verif = false;
      }
    }
    else if (((command_str == "+15") || (command_str == "-15")) && (!ongoing_rotation) && (!ongoing_slowdown))    // Is +15 or -15 key decoded?
    {
      target_heading = current_heading + command_str.toInt();        // Increase or decrease target heading by 15
      if (target_heading > 360) target_heading = 360;                // Do not go higher than 360
      else if (target_heading < 0) target_heading = 0;               // Do not allow negative value
      tft.fillRect(20, 90, 130, 40, ILI9341_BLACK);                  // Erase old target heading
      tft.setTextColor(ILI9341_GREEN);                               // Write new target heading
      tft.setTextSize(3);                                            //      "
      tft.setCursor(20,120);                                         //      "
      tft.print(String(target_heading));                             //      "        
      rotation_req = true;                                           // Launch a new rotation
      number = "";
      command_str = "";
    }
  }

  // Encoder rotation processing                                      
  if ((!ongoing_rotation) && (!ongoing_slowdown))                     // Neither in rotation nor in slowdown?
  {  // Yes
    encoder_logic_level = digitalRead(ENCODER_A);                     // Read logic level of encoder A signal
    if (encoder_logic_level != encoder_previous_logic_level)          // New logic level on A?
    {   // Yes
      if (target_heading == -1) target_heading = current_heading;     // If there is not already a target heading, set the target heading to the current heading
      if (digitalRead(ENCODER_B) != encoder_logic_level)              // Read logic level of encoder B signal. Same level as A signal?
      {  // No   
        target_heading += 5;                                          // Increase target heading by 5 degrees
        while (target_heading%5 != 0) target_heading--;               // Round up to the nearest 5 degree ending
        if (target_heading > 360) target_heading = 360;               // Ensure that target heading does not go beyond 360 degrees
      } 
      else                                                            // Here logic level of B signal different from A signal
      {
        target_heading -= 5;                                          // Decrease target heading by 5 degrees
        while (target_heading%5 != 0) target_heading++;               // Round up to the nearest 5 degree ending
        if (target_heading < 0) target_heading = 0;                   // Ensure that target heading does not go below 0 degrees
      }
      tft.fillRect(20, 120, 130, 40, ILI9341_BLACK);                  // Update target heading on display
      tft.setTextColor(ILI9341_GREEN);                                //      "
      tft.setTextSize(4);                                             //      "
      tft.setCursor(20,120);                                          //      "
      tft.print(target_heading);                                      //      "
    } 
    encoder_previous_logic_level = encoder_logic_level;               // Save A signal logic level for next pass
  }
  
  // Detection of pushbutton or keypad initiated rotation
  if (((!ongoing_rotation) && (!digitalRead(ENCODER_PUSHBUTTON)) && (target_heading != -1)) || (rotation_req))  // The conditions to meet to initiate a rotation
    {
      ongoing_rotation = true;                                        // Set the rotation ongoing rotation flag
      azim_refresh_req = true;                                        // Request a refresh of the displayed azimuth
      active_alarm = false;
      digitalWrite(BREAK_RELAY,LOW);                                  // Release the brake (activave solenoid)
      rotation_req = true;                                            // Request a rotation
      delay(500);                                                     // Pause for 1/2 second
    }

  // Displayed message processing
  if (rotation_req)                                                   // Is there a rotation request?
  {  // Yes
    tft.fillRect(20, 90, 130, 30, ILI9341_BLACK);                     // Display the rotation message
    tft.setTextSize(2);                                               //               "
    tft.setTextColor(ROTATION_COLOR);
    tft.setCursor(20,90);
    tft.print("ROTATION");
  }
  else if (slowdown_req)                                              // Otherwise, request for slowdown?
  { // Yes
    tft.fillRect(20, 90, 130, 30, ILI9341_BLACK);                     // Display the slowdown message
    tft.setTextSize(2);                                               //               "      
    tft.setTextColor(SLOWDOWN_COLOR);
    tft.setCursor(20,90);
    tft.print("SLOWDOWN");
    slowdown_req = false;                                             // Clear the slowdown request
  }
  else if (stopping_req)                                              // Otherwise, request for stop?
  {  // Yes
    tft.fillRect(20, 90, 130, 30, ILI9341_BLACK);                     // Display the Idle message
    tft.setTextSize(2);                                               //               "      
    tft.setTextColor(IDLE_COLOR);
    tft.setCursor(20,90);
    tft.print("  IDLE  ");
    stopping_req = false;                                             // Clear the stop request
  }

  // Rotation phase processing
  if (ongoing_rotation)                                               // Is there an ongoing rotation?
  { // Yes
    // Processing a rotation request
    if (rotation_req)                                                 // Is there a rotation request?
    {  // Yes
      if ((current_heading + SLOWDOWN_RESIDUAL_ROTATION) < target_heading)      // In clockwise rotation
      {
        digitalWrite(CCW_ROTATION_RELAY,HIGH);                      // De-activate the CCW relay
        digitalWrite(CW_ROTATION_RELAY,LOW);                        // Activate the CW relay      
        cw_rotation = true;                                         // Flag rotation as CW
      }
      else if ((current_heading - SLOWDOWN_RESIDUAL_ROTATION) > target_heading) // In counter-clockwise rotation
      { // Yes
        digitalWrite(CW_ROTATION_RELAY,HIGH);                       // De-activate the CW relay
        digitalWrite(CCW_ROTATION_RELAY,LOW);                       // Activate the CCW relay        
        cw_rotation = false;                                        // Flag rotation as CCW
      }
      rotation_req = false;                                         // Clear the rotation request flag
    }
    else
    {  // Already in rotation phase 
      if ((!digitalRead(ENCODER_PUSHBUTTON))                                                              // Forced stop via encoder pushbutton?
          || ((cw_rotation)  && ((current_heading + SLOWDOWN_RESIDUAL_ROTATION) >= target_heading))       // or heading reached during CW rotation?
          || ((!cw_rotation) && ((current_heading - SLOWDOWN_RESIDUAL_ROTATION) <= target_heading))       // or heading reached during CCW rotation?
          || (slowdown_req))                                                                              // or slowdown request?
      { // Yes
        ongoing_slowdown = true;                                    // Raise the slowdown flag
        slowdown_req = true;                                        // Request a slowdown
        azim_refresh_req = true;                                    // Request a refresh of the displayed azimuth  
      } 
    }
     // Other actions during the rotation phase (no slowdown)
    if (!ongoing_slowdown)                                          // Not in slowdown phase?
    {
      // Vérifying that the rotator is indeed rotating
      if (!ongoing_rotation_verif)                                  // Not in rotation verification?                     
      { // Beginning of waiting period before checking for rotation
        start_rotation_verif_millis = millis();                     // Save the rotation verification initial time
        previous_heading_verif = current_heading;                   // Save the current heading as the previous heading for the next pass
        ongoing_rotation_verif = true;                              // Raise the ongoing rotation flag
      }
      else if ((unsigned long)(millis() - start_rotation_verif_millis) >= 2000)    // Rotation verification delay expired?
      {  //Yes
        ongoing_rotation_verif = false;                             // Mark the end of the rotation verification cycle
        if (((cw_rotation) && (current_heading < (previous_heading_verif + 3)) && (current_heading < 360)) 
           || ((!cw_rotation) && (current_heading > (previous_heading_verif - 3)) && (current_heading > 0)))   // Has rotator turned at least 3 degrees in either sense of rotation?
        { // No. Rotor is frozen. Stop everything
          slowdown_req = true;                                      // Request slowdown
          ongoing_slowdown = true;                                  // Raise the ongoing slowdown flag
          tft.fillRect(20, 120, 130, 40, ILI9341_BLACK);            // Display the rotation alarm
          tft.setTextSize(2);                                       //             "
          tft.setTextColor(ILI9341_RED);
          tft.setCursor(20,120);
          tft.print("ROTATION");
          tft.setCursor(20,140);
          tft.print("ALARM");
          active_alarm = true;                                      // Indicate an active alarm
        }     
      }
    }
  }
  // Slowdown phase processing (end of rotation phase)
  if (ongoing_slowdown)                                             // Ongoing slowdown?
  {  // Yes
    if (slowdown_req)    // Beginning of slowdown phase             // New slowdown request?
    {
      digitalWrite(CW_ROTATION_RELAY,HIGH);                         // Stop rotation (release both motor relays)
      digitalWrite(CCW_ROTATION_RELAY,HIGH);                        //     "          
      ongoing_rotation = false;                                     // Clear the ongoing rotation flag
      slowdown_req = true;                                          // Request a slowdown phase
      azim_refresh_req = true;                                      // Request a display refresh
      start_slowdown_ongoing_millis = millis();                     // Initialize the slowdown timer
      ongoing_rotation_verif = false;                               // End the rotation verification
    }  
    else if ((unsigned long)(millis() - start_slowdown_ongoing_millis) >= 6000) // Slowdown delay expired?
    {
      ongoing_slowdown = false;                                     // Yes, clear the ongoing slodown flag
      azim_refresh_req = true;                                      // Request a display refresh
      digitalWrite(BREAK_RELAY,HIGH);                               // Release the brake relay (engage brake)
      slowdown_req = false;                                         // Clear the slowdown request flag
      stopping_req = true;                                          // Request for a complete stop (going idle)
      if (!active_alarm) tft.fillRect(20, 120, 130, 40, ILI9341_BLACK);    // If there is no active alarm, erase the text zone
      target_heading = -1;                                          // Erase the target heading
    } 
  }

  // Display lighting processing
  if ((unsigned long)(millis() - start_display_count_millis) >= 3600000)     // Display lighting timer has reached the 1 hour limit?
  {
    digitalWrite(DISPLAY_LIGHTING_CTRL, LOW);                               // Yes, turn off display lighting
    start_display_count_millis = millis();                                  // Reset the display lighting timer value
  }
}


// // Main loop function of the Pico's second core, Arduino style, runs forever
void loop1()
{
  unsigned int pot_cursor = 0;
  unsigned int pot_maxval = 0;
  unsigned int pot_minval = 0;
  if (!heading_avail)                                                             // Has new heading been read by the main loop?
  {   // Yes
    for (int i=0; i<POT_NUMBER_SAMPLE; i++)                                       // Potentiometer reading loop for averaging purpose
    {
      pot_cursor += analogRead(POT_CURSOR);                                       // Accumulate ADC readings of pot cursor voltage
      pot_maxval += analogRead(POT_MAX);                                          // Accumulate ADC readings of pot maximum voltage
      pot_minval += analogRead(POT_MIN);                                          // Accumulate ADC readings of pot minimum voltage
      delay(1);                                                                   // 1 ms delay
    }
    pot_cursor = round(float(pot_cursor) / POT_NUMBER_SAMPLE);                    // Average calculation of pot cursor voltage
    pot_maxval = round(float(pot_maxval) / POT_NUMBER_SAMPLE);                    // Average calculation of pot maximum voltage
    pot_minval = round(float(pot_minval) / POT_NUMBER_SAMPLE);                    // Average calculation of pot minimum voltage
    instant_heading = 360 - round((float(pot_cursor) / (pot_maxval - pot_minval)) * 360);    // Instantaneous heading calculation. Warning: rotator voltage distribution is inverted compared to heading.
    if (instant_heading > 360) instant_heading = 360;                             // Do not go passed 360 degrees
    if (instant_heading < 0) instant_heading = 0;                                 // Do not allow negative headings
    heading_avail = true;                                                         // Flag of a new instantaneous heading available   
  }

  // Rotor-EZ command processing
  while (Serial.available() > 0)                                                  // Character received on serial port?
  {
    char rx_caract = Serial.read();                                               // Yes, read character
    if ((rx_caract == '\r') || (rx_caract == ';'))                                // Is it one of the two an end-of-command characters?
    {                                                                             // Yes, process the received command                                           
      if (rx_command.substring(0,3) == "AI1")                                     // Report current bearing?
      {
        Serial.print(";" + current_heading_txt);                                  // Send back current heading in the proper format
      }
      else if (((rx_command.substring(0,3) == "AP1") || (rx_command.substring(6)== "\r"))   // Rotate to new heading command received, and not currently in rotation or slowdown?
                && (!ongoing_rotation) && (!ongoing_slowdown))
      {  // Yes
        target_heading = rx_command.substring(3,6).toInt();                       // Convert received target heading string into target heading
        if ((target_heading >= 0) && (target_heading < 360))                        // Received heading within 0-359 degree range?
        {
          rotation_req = true;                                                    // Request a rotation phase
          active_alarm = false;
          tft.fillRect(20, 120, 130, 40, ILI9341_BLACK);                          // Update target heading on display
          tft.setTextColor(ILI9341_GREEN);                                        //      "
          tft.setTextSize(4);                                                     //      "
          tft.setCursor(20,120);                                                  //      "
          tft.print(target_heading);                                              //      "
        }
      }
      else if (rx_command == "")                                                  // Stop rotation command? (";" already removed from received string)
      {
        ongoing_slowdown = true;                                                  // Set the ongoing slowdown flag
        slowdown_req = true;                                                      // Request for a slowdown phase
        azim_refresh_req = true;                                                  // Request an azimuth display refresh
      }
      rx_command = "";                                                            // Erase the received command
    }
    else rx_command += rx_caract;                                                 // Otherwise, no end-of-comamnd character received, accumulate the character into the command variable
  }
}

// End of source code
