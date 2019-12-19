/*
 * The MySensors Arduino library handles the wireless radio link and protocol
 * between your home built sensors/actuators and HA controller of choice.
 * The sensors forms a self healing radio network with optional repeaters. Each
 * repeater and gateway builds a routing tables in EEPROM which keeps track of the
 * network topology allowing messages to be routed to nodes.
 *
 * Created by Henrik Ekblad <henrik.ekblad@mysensors.org>
 * Copyright (C) 2013-2018 Sensnology AB
 * Full contributor list: https://github.com/mysensors/MySensors/graphs/contributors
 *
 * Documentation: http://www.mysensors.org
 * Support Forum: http://forum.mysensors.org
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 *******************************
 *
 * REVISION HISTORY
 * Version 1.0 - Henrik Ekblad
 * Version 1.1 - GizMoCuz
 *
 * DESCRIPTION
 * Use this sensor to measure volume and flow of your house water meter.
 * You need to set the correct pulsefactor of your meter (pulses per m3).
 * The sensor starts by fetching current volume reading from gateway (VAR 1).
 * Reports both volume and flow back to gateway.
 *
 * Unfortunately millis() won't increment when the Arduino is in
 * sleepmode. So we cannot make this sensor sleep if we also want
 * to calculate/report flow.
 * http://www.mysensors.org/build/pulse_water
 */
//--------------------------------------------------------------------
//                 +/          
//                 `hh-        
//        ::        /mm:       
//         hy`      -mmd       
//         omo      +mmm.  -+` 
//         hmy     .dmmm`   od-
//        smmo    .hmmmy    /mh
//      `smmd`   .dmmmd.    ymm
//     `ymmd-   -dmmmm/    omms
//     ymmd.   :mmmmm/    ommd.
//    +mmd.   -mmmmm/    ymmd- 
//    hmm:   `dmmmm/    smmd-  
//    dmh    +mmmm+    :mmd-   
//    omh    hmmms     smm+    
//     sm.   dmmm.     smm`    
//      /+   ymmd      :mm     
//           -mmm       +m:    
//            +mm:       -o    
//             :dy             
//              `+:     
//--------------------------------------------------------------------
//   __|              _/           _ )  |                       
//   _| |  |   ` \    -_)   -_)    _ \  |   -_)  |  |   -_)     
//  _| \_,_| _|_|_| \___| \___|   ___/ _| \___| \_,_| \___|  
//--------------------------------------------------------------------    
// 2019/08/01 - FB modifs V1.21
//--------------------------------------------------------------------
#include <TimeLib.h>
#include <EEPROM.h>

// Enable debug prints to serial monitor
#define MY_DEBUG

// Enable and select radio type attached
#define MY_RADIO_RF24
//#define MY_RADIO_NRF5_ESB
//#define MY_RADIO_RFM69
//#define MY_RADIO_RFM95

#include <MySensors.h>

#define VERSION   "v1.2.1"

#define DIGITAL_INPUT_SENSOR 3              // The digital input you attached your sensor.  (Only 2 and 3 generates interrupt!)
#define MAX_FLOW 100                        // Max flow (l/min) value to report. This filters outliers.
#define CHILD_ID_FLOW	1                   // Id of the sensor child flow
#define CHILD_ID_PULSE  2                   // Id of the sensor child pulse
#define CHILD_ID_VOLUME 3                   // Id of the sensor child volume
#define CHILD_ID_PULSE_FACTOR  	4           // Id of the sensor child pulse factor
#define CHILD_ID_LEAK_THRESHOLD 5           // Id of the sensor child leak threshold
#define CHILD_ID_LEAK  6                    // Id of the sensor child leak
#define HBD      24                         // 24 hours 
#define MY_DEFAULT_ERR_LED_PIN 4  			// Error led pin
#define CONFIG_START EEPROM_LOCAL_CONFIG_ADDRESS+1
#define DEFAULT_PULSE_FACTOR 100			// Number of blinks per m3 of your meter (100: One rotation/10 liters)
#define DEFAULT_LEAK_THRESHOLD 800			// Alert threshold, by default 800 l/day

uint32_t SEND_FREQUENCY =  30000;           // Minimum time between send (in milliseconds). We don't want to spam the gateway.
uint32_t LED_BLINCK_FREQUENCY = 80;         // 
unsigned int  conso[HBD] = {0};             // historic consumption
boolean  flag_leak_type1 = false;           // continuous consumption during 24hr
boolean  old_flag_leak_type1 = true;
boolean  flag_leak_type2 = false;           // consumption in excess of the 24hr threshold
boolean  old_flag_leak_type2 = true;
boolean  on_time = false;
boolean  flag_on_time_blink = false;
unsigned int last_hour = 0;                 // Last hour
unsigned int last_day = 0;                  // Last hour

volatile uint32_t pulseCount = 0;
volatile uint32_t lastBlink = 0;
volatile double flow = 0;
bool pcReceived = false;
uint32_t oldPulseCount = 0;
uint32_t newBlink = 0;
double oldflow = 0;
double volume =0 ;
double oldvolume = 0;
uint32_t lastSend = 0;
uint32_t lastPulse = 0;
uint32_t lastLedBlink = 0;
uint32_t totalPulse = 0;

struct StoreStruct {
  int pulse_factor = DEFAULT_PULSE_FACTOR;               // Number of blinks per m3 of your meter (One rotation/1 liters)
  int leak_threshold = DEFAULT_LEAK_THRESHOLD;           // Alert threshold, by default 800 l/day
} storage_config;

double ppl = ((double)storage_config.pulse_factor)/1000;        // Pulses per liter


MyMessage flowMsg(CHILD_ID_FLOW, V_FLOW);  		    // l/mn
MyMessage volumeMsg(CHILD_ID_VOLUME, V_VOLUME);     // liter by day
MyMessage lastCounterMsg(CHILD_ID_PULSE, V_VAR1);  // pulse
MyMessage pulseFactor(CHILD_ID_PULSE_FACTOR, V_VAR2);  // pulse_factor
MyMessage leakThreshold(CHILD_ID_LEAK_THRESHOLD, V_VAR3);	// leak_threshold
MyMessage leak(CHILD_ID_LEAK, V_VAR4);             	// leak

//-----------------------------------------------------------------------
void before()
{
  pinMode(MY_DEFAULT_ERR_LED_PIN, OUTPUT);
  
  for (int i=0; i<5; i++) {
    digitalWrite(MY_DEFAULT_ERR_LED_PIN, LOW);
    delay(100);
    digitalWrite(MY_DEFAULT_ERR_LED_PIN , HIGH);
    delay(100);
   }
}

//-----------------------------------------------------------------------
void setup()
{
  // initialize our digital pins internal pullup resistor so one pulse switches from high to low (less distortion)
	pinMode(DIGITAL_INPUT_SENSOR, INPUT_PULLUP);

  loadConfig();

  Serial.print(F("Pulse factor: "));
  Serial.println(storage_config.pulse_factor);
  Serial.print(F("Leak threshold: "));
  Serial.println(storage_config.leak_threshold);
  
	pulseCount = oldPulseCount = 0;

	// Fetch last known pulse count value from gw
	request(CHILD_ID_PULSE, V_VAR1);
  
  setSyncProvider(requestSync); 
  //setTime(1357041600); // Jan 1 2013);
  
	lastSend = lastPulse = millis();

	attachInterrupt(digitalPinToInterrupt(DIGITAL_INPUT_SENSOR), onPulse, FALLING);

}

//-----------------------------------------------------------------------
time_t requestSync()
{
  Serial.println(F(">> Request time to gw."));
  
  requestTime();
  delay(500); // wait for time receiving
  return 0;
} 

//-----------------------------------------------------------------------
// This is called when a new time value was received
void receiveTime(unsigned long controllerTime) 
{
const unsigned long DEFAULT_TIME = 1357041600; // Jan 1 2013
char buffer[20];

  // Ok, set incoming time 
  Serial.print(">> Time received from gw: ");
  Serial.print(controllerTime);
  Serial.print(" ");
    
  if (controllerTime > DEFAULT_TIME) {
	  setTime(controllerTime+adjustDstEurope());    // +2 hrs Paris
	  last_hour = hour();
	  last_day = day();
	  on_time = true;
	  Serial.println("*");
  }
  else Serial.println(".");
  
}

//-----------------------------------------------------------------------
uint16_t adjustDstEurope()
{
 /*You can use the following equations to calculate when DST starts and ends.
 The divisions are integer divisions, in which remainders are discarded.
 "mod" means the remainder when doing integer division, e.g., 20 mod 7 = 6.
 That is, 20 divided by 7 is 2 and 6/7th (where six is the remainder).
 With: y = year.
        For the United States:
            Begin DST: Sunday April (2+6*y-y/4) mod 7+1
            End DST: Sunday October (31-(y*5/4+1) mod 7)
           Valid for years 1900 to 2006, though DST wasn't adopted until the 1950s-1960s. 2007 and after:
            Begin DST: Sunday March 14 - (1 + y*5/4) mod 7
            End DST: Sunday November 7 - (1 + y*5/4) mod 7;
        European Economic Community:
            Begin DST: Sunday March (31 - (5*y/4 + 4) mod 7) at 1h U.T.
            End DST: Sunday October (31 - (5*y/4 + 1) mod 7) at 1h U.T.
            Since 1996, valid through 2099
(Equations by Wei-Hwa Huang (US), and Robert H. van Gent (EC))
 
 Adjustig Time with DST Europe/France/Paris: UTC+1h in winter, UTC+2h in summer
 
 */
 
  // last sunday of march
  int beginDSTDate=  (31 - (5* year() /4 + 4) % 7);
  Serial.println(beginDSTDate);
  int beginDSTMonth=3;
  //last sunday of october
  int endDSTDate= (31 - (5 * year() /4 + 1) % 7);
  Serial.println(endDSTDate);
  int endDSTMonth=10;
  // DST is valid as:
  if (((month() > beginDSTMonth) && (month() < endDSTMonth))
      || ((month() == beginDSTMonth) && (day() > beginDSTDate)) 
	  || ((month() == beginDSTMonth) && (day() == beginDSTDate) && (hour() >= 1))
      || ((month() == endDSTMonth) && (day() < endDSTDate))
	  || ((month() == endDSTMonth) && (day() == endDSTDate) && (hour() < 1)))
  return 7200;      // DST europe = utc +2 hour (summer time)
  else return 3600; // nonDST europe = utc +1 hour (winter time)
}

//-----------------------------------------------------------------------
void change_led_state()
{
static int led_state;
  
  led_state = !led_state;
  digitalWrite(MY_DEFAULT_ERR_LED_PIN, led_state);
  
}

//-----------------------------------------------------------------------
void loadConfig() {
boolean flag_check = false;

  for (unsigned int t=0; t<sizeof(storage_config); t++) {
      *((char*)&storage_config + t) = EEPROM.read(CONFIG_START + t);
	}
	
	// check values -----
	if (storage_config.pulse_factor <= 0) {
		storage_config.pulse_factor = DEFAULT_PULSE_FACTOR;
		flag_check = true;
	}
	if (storage_config.leak_threshold <= 0) {
		storage_config.leak_threshold = DEFAULT_LEAK_THRESHOLD; 
		flag_check = true;
	}
	if (flag_check == true) saveConfig();
}

//-----------------------------------------------------------------------
void saveConfig() {
  for (unsigned int t=0; t<sizeof(storage_config); t++)
    EEPROM.write(CONFIG_START + t, *((char*)&storage_config + t));
}

//-----------------------------------------------------------------------
void presentation()
{
  // Send the sketch version information to the gateway and Controller
  sendSketchInfo("FB Water Meter", "1.2.1");

  // Register this device as Water flow sensor
  present(CHILD_ID_FLOW, S_WATER, "Débit");
  present(CHILD_ID_PULSE, S_WATER, "Impulsion");
  present(CHILD_ID_VOLUME, S_WATER, "Volume"); 
  present(CHILD_ID_PULSE_FACTOR, S_WATER, "Facteur impulsion");
  present(CHILD_ID_LEAK_THRESHOLD, S_WATER, "Niveau Fuite");
  present(CHILD_ID_LEAK, S_WATER, "Fuite");

  Serial.println(F("   __|              _/           _ )  |"));
  Serial.println(F("   _| |  |   ` \\    -_)   -_)    _ \\  |   -_)  |  |   -_)"));
  Serial.println(F("  _| \\_,_| _|_|_| \\___| \\___|   ___/ _| \\___| \\_,_| \\___|"));
  Serial.print(F("                                             "));
  Serial.println(VERSION);
}

//-----------------------------------------------------------------------
void printSeparator(uint8_t lg)
{
	for (uint8_t i=0; i<lg; i++) Serial.print(F("-----"));
	Serial.println(F("-"));
}

//-----------------------------------------------------------------------
void receive(const MyMessage &message)
{
  //Serial.println(message.type);

  if (message.type==V_VAR1) {
      uint32_t gwPulseCount=message.getULong();
      pulseCount += gwPulseCount;
      flow=oldflow=0;
      Serial.print(F("Received last pulse count from gw:"));
      Serial.println(pulseCount);
      pcReceived = true;
  }

  if (message.type==V_VAR2) {
      Serial.print(F("Received pulse factor value from gw:"));
      Serial.println(message.getUInt());
      if (storage_config.pulse_factor != message.getUInt()) {
        if (message.getUInt() > 0) {
          storage_config.pulse_factor = message.getUInt();
          saveConfig();
          Serial.println(F("Saved in EEprom"));
        }  
      }
   }
      
   if (message.type==V_VAR3) {
      Serial.print("Received leak threshold value from gw:");
      Serial.println(message.getUInt());
      if (storage_config.leak_threshold != message.getUInt()) {
        if (message.getUInt() > 0) {
          storage_config.leak_threshold = message.getUInt();
          saveConfig();
          Serial.println(F("Saved in EEprom"));
        }
      }
   }
}

//-----------------------------------------------------------------------
void onPulse()
{

	uint32_t newBlink = micros();
	uint32_t interval = newBlink-lastBlink;

	if (interval!=0) {
		lastPulse = millis();
		if (interval<500000L) {
			// Sometimes we get interrupt on RISING,  500000 = 0.5 second debounce ( max 120 l/min)
			return;
		}
		flow = (60000000.0 /interval) / ppl;
	}
	lastBlink = newBlink;

	pulseCount++;
	if (on_time) conso[hour()]++;
}


//-----------------------------------------------------------------------
void loop()
{
  char buffer[20];
  uint32_t currentTime = millis();


	// Only send values at a maximum frequency 
	if (currentTime - lastSend > SEND_FREQUENCY) {
		lastSend=currentTime;
  
		if (!pcReceived) {
			//Last Pulsecount not yet received from controller, request it again
			Serial.print(".");
			request(CHILD_ID_PULSE, V_VAR1);
			return;
		}
		   
		if (flow != oldflow) {
			oldflow = flow;

			Serial.print(F("l/min:"));
			Serial.println(flow);
			
			// Check that we don't get unreasonable large flow value.
			// could happen when long wraps or false interrupt triggered
			if (flow<((uint32_t)MAX_FLOW)) {
				send(flowMsg.set(flow, 2));   // Send flow value to gw
			}
		}

		// No Pulse count received in 2min
		if(currentTime - lastPulse > 120000) {
			flow = 0;
		}

		// Pulse count has changed
		if (pulseCount != oldPulseCount) {
			oldPulseCount = pulseCount;

			Serial.print(F("pulsecount:"));
			Serial.println(pulseCount);

			send(lastCounterMsg.set(pulseCount));  // Send  pulsecount value to gw in VAR1

			double volume = 1000*((double)pulseCount/(double)storage_config.pulse_factor);
			if (volume != oldvolume) {
				oldvolume = volume;

				Serial.print(F("volume:"));
				Serial.println(volume, 3);

				send(volumeMsg.set(volume, 3));  // Send volume value to gw
			}
		}

		if (on_time) {
			if (flag_on_time_blink == false) {
				flag_on_time_blink = true;
				digitalWrite(MY_DEFAULT_ERR_LED_PIN , HIGH);
				Serial.println("#");
			}
			
			// flag leak type 1 or 2 has changed
			if (flag_leak_type1 != old_flag_leak_type1 && flag_leak_type2 != old_flag_leak_type2) {
			  int leak_type = 0;
			  if (flag_leak_type1 == true) leak_type += 1;
			  if (flag_leak_type2 == true) leak_type += 2;
			  Serial.print("Leak type:");
			  Serial.println(leak_type);
			  send(leak.set(leak_type));
			  
			  old_flag_leak_type1 = flag_leak_type1;
			  old_flag_leak_type2 = flag_leak_type2;
			}

			// Check leak --------------
			flag_leak_type1=true;
			flag_leak_type2=true;
			// check continuous consumption during last 24hr
			for (uint8_t i=0; i<HBD; i++) {
			  uint8_t j = i+1;
			  if(j==HBD) j = 0;
			  if(conso[i] ==0 && conso[j] == 0) flag_leak_type1 = false;
			}
			
			// check consumption in excess of the 24hr threshold
			totalPulse=0;
			for (uint8_t i=0; i<HBD; i++) {
			  totalPulse += conso[i];
			}
			
			Serial.print(totalPulse);
			Serial.print("/");
			Serial.println(storage_config.leak_threshold);
			if (totalPulse < storage_config.leak_threshold) flag_leak_type2 = false;
		}
		else {
			// Only send values at a maximum frequency 
			if (currentTime - lastLedBlink > LED_BLINCK_FREQUENCY) {
				lastLedBlink=currentTime;
				change_led_state();
			}
			flag_on_time_blink=false;
		}
		// simulate test ----
		onPulse();
		
		// Print resume ------------------------------------------
		sprintf(buffer, "%d/%d - %d:%d", day(), month(), hour(), minute());
		Serial.println(buffer);
		printSeparator(HBD);
		for (uint8_t i=0; i<HBD; i++) {
		  char buff[10];
		  sprintf(buff, "| %2d ", i);
		  Serial.print(buff);
		}
		Serial.println(F("|"));
		printSeparator(HBD);
		for (uint8_t i=0; i<HBD; i++) {
		  char buff[10];
		  sprintf(buff, "|%4d", conso[i]);
		  Serial.print(buff);
		}
		Serial.println(F("|"));
		printSeparator(HBD);
		if (flag_leak_type1 == false && flag_leak_type2 == false) {
		  digitalWrite(MY_DEFAULT_ERR_LED_PIN, HIGH);
		  Serial.println(F("No leak detected"));
		}
		else {
			digitalWrite(MY_DEFAULT_ERR_LED_PIN, LOW);
			Serial.print(F("!! Leak detected type "));
			if (flag_leak_type1 == true) Serial.println(F("1"));
			if (flag_leak_type2 == true) Serial.println(F("2"));
		}

    // --------------------------------
    Serial.println(F("Send pulse, leak info"));
    send(pulseFactor.set(storage_config.pulse_factor));
    send(leakThreshold.set(storage_config.leak_threshold));
	}

	
	if (on_time) {
		// Every hours reset conso --------
		if (hour() != last_hour)  {
		  Serial.println(F("Reset conso"));
		  conso[hour()] = 0;
		  last_hour = hour();
		}
	  
		// Every days reset counters --------
		if (day() != last_day)  {
			Serial.println(F("Reset counters"));
			pulseCount = 0;
			flow = 0;
			volume = 0;
			last_day = day();
		}
	}

}
