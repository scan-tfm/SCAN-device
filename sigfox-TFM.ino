/*
 *  Copyright (C) 2016 Daniel Cárdenes
 *  
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see http://www.gnu.org/licenses/.
 *
 *  Version: 1.0
 *  Design and Implementation:  Daniel Cárdenes
 */ 
 
//Include libraries for SIGFOX Module 
#include <cookingClasses.h>
#include <cookingSigfox.h>
#include <cookingUART.h>
#include <cookingUtils.h>

//Include Library for EEPROM management
#include <EEPROM.h>

//Include Library for software serial emulation
#include <SoftwareSerial.h>

//Define Pins
#define LOW_LED   8
#define MED_LED   9
#define HIG_LED   10
#define TX_LED   11
#define LOCK_LED  12
#define ERASE_BUTTON 4
#define LEVEL_BUTTON 5
#define TX_BUTTON 6
#define REMOTE_BUTTON 7
#define GPS_TX 2
#define GPS_RX 3

//Define Baud Rates for Serial Ports
#define GPS_BAUD 9600
#define CONSOLE_BAUD 9600

#define ANTIREBOUND 200
//Seconds to keep erase button pressed to delete EEPROM
#define SECONDS_TO_ERASE  5
//Max NMEA frame length
#define MAX_FRAME_LENGTH 60
//Max NMEA header length
#define MAX_HEADER_LENGTH 6

//Define max frame and header of user commands
#define MAX_USER_FRAME_LENGTH   10
#define MAX_USER_HEADER_LENGTH  3
#define MAX_CMD_PAR_LENGTH    3
#define MAX_POS_FIELD 15

//EEPROM Map
#define TXID_ADDRESS          0
#define TXNUM_ADDRESS         2
#define HIG_POWER_ADDRESS     3
#define MED_POWER_ADDRESS     4
#define LOW_POWER_ADDRESS     5
#define START_REG_ADDRESS     6

#define BYTES_TO_SEND         11
//maximum number of transmission allowed. This means than eeprom is full
#define TX_MAX                90

//////////////Global Variables//////////////
//This type stores sigfox payload to be transmmited
struct sigfoxData{
  unsigned int txId;
  byte txLevel;
  long latitude;
  long longitude;
};

sigfoxData payload;

//this byte array stores payload to be sent in the format of sigfox.send
uint8_t dataReadyToSend[BYTES_TO_SEND];

SoftwareSerial myGps(GPS_RX,GPS_TX);
char gpsChar=' ';
char userChar=' ';
bool levelButtonPushed=false;
bool eraseButtonPushed=false;
bool remoteButtonPushed=false;
bool txButtonPushed=false;
bool lowLedActive=false;
bool medLedActive=false;
bool higLedActive=true;
bool lockLedActive=false;
bool txLedActive=false;
bool gpsLocked=false;
bool remoteMode=false;
//store the seconds the erase button is kept pushed
int secondsEraseButtonPushed=0;
//store current tx level. Default at start up is HIGH power;
byte level;
//To store transmission ID
unsigned int txid;

//To store the number of registers in the eeprom
byte txnum;

//EEPROM address variable
unsigned int address;

byte sigfoxError=0;
bool eepromFull=false;



byte levelButtonStatus;
byte levelButtonLastStatus;
byte txButtonStatus;
byte txButtonLastStatus;
byte remoteButtonStatus;
byte remoteButtonLastStatus;
//This variable stores GPS extracted frame
char frame[MAX_FRAME_LENGTH];
bool fillingFrame=false;
bool frameCompleted=false;
bool fillingHeader=false;
int indexFrame=0;

//This variable stores User command frame
char userFrame[MAX_USER_FRAME_LENGTH];
bool fillingUserFrame=false;
bool userFrameCompleted=false;
int indexUserFrame=0;

//Variables to store extracted GPS info
char lat[MAX_POS_FIELD];
char lon[MAX_POS_FIELD];
char dirNS;
char dirWE;
char validData;

//Variables to store extracted User command parameters
char commandedLevel[MAX_CMD_PAR_LENGTH];

//Stores if ACK a transmission with ACK was performed and if ACK was received
bool txACK, rxACK;

//////////End Global Variables/////////////

//Activate three level LEDs to indicate that an error has ocurrued with Sigfox
void onSigfoxError()
{
  digitalWrite(HIG_LED,HIGH);
  digitalWrite(MED_LED,HIGH);
  digitalWrite(LOW_LED,HIGH);
}

//Initial setup
void setup() { 
  //Initialize pins
  pinMode(GPS_RX,INPUT);
  pinMode(GPS_TX,OUTPUT);
  pinMode(LOW_LED,OUTPUT);
  pinMode(MED_LED,OUTPUT);
  pinMode(HIG_LED,OUTPUT);
  pinMode(TX_LED,OUTPUT);
  pinMode(LOCK_LED,OUTPUT);
  pinMode(ERASE_BUTTON,INPUT);
  pinMode(LEVEL_BUTTON,INPUT);
  pinMode(TX_BUTTON,INPUT);
  pinMode(REMOTE_BUTTON,INPUT);

  //Initialize values from EEPROM
  EEPROM.get(TXID_ADDRESS,txid);
  txnum=EEPROM.read(TXNUM_ADDRESS);
  level=EEPROM.read(HIG_POWER_ADDRESS);
  
  //Initialize serial ports
  myGps.begin(GPS_BAUD);
  Serial.begin(CONSOLE_BAUD);

  digitalWrite(HIG_LED,HIGH);

  levelButtonStatus=digitalRead(LEVEL_BUTTON);
  levelButtonLastStatus=levelButtonStatus;
  txButtonStatus=digitalRead(TX_BUTTON);
  txButtonLastStatus=txButtonStatus;
  remoteButtonStatus=digitalRead(REMOTE_BUTTON);
  remoteButtonLastStatus=remoteButtonStatus;


  //Setup interrupt for Timer 1 (1 second)
  noInterrupts();
  TCCR1A=0;
  TCCR1B=0;
  TCNT1=3036;
  TCCR1B |= (1 << CS12);
  TIMSK1 |= (1 << TOIE1);
  interrupts();
  //End of setup interrupt
}

//SErvice routine for Timer1
ISR(TIMER1_OVF_vect)
{
  TCNT1=3036;
  if (gpsLocked) digitalWrite(LOCK_LED,HIGH);
  else digitalWrite(LOCK_LED,digitalRead(LOCK_LED)^1);
  if (remoteMode) digitalWrite(TX_LED,digitalRead(TX_LED)^1);
  if (eraseButtonPushed) secondsEraseButtonPushed++;
  else secondsEraseButtonPushed=0;
}


//Update Led status
void ledStatus()
{
  if (levelButtonPushed)
  {
    levelButtonPushed=false;
    if (higLedActive) {
      higLedActive=false;
      digitalWrite(HIG_LED,LOW);
      lowLedActive=true;
      digitalWrite(LOW_LED,HIGH);
      //level=LOW_LEVEL;
      level=EEPROM.read(LOW_POWER_ADDRESS);
    } else if (medLedActive) {
      medLedActive=false;
      digitalWrite(MED_LED,LOW);
      higLedActive=true;
      digitalWrite(HIG_LED,HIGH);
      //level=HIG_LEVEL;
      level=EEPROM.read(HIG_POWER_ADDRESS);
    } else {
      lowLedActive=false;
      digitalWrite(LOW_LED,LOW);
      medLedActive=true;
      digitalWrite(MED_LED,HIGH);
      //level=MED_LEVEL;
      level=EEPROM.read(MED_POWER_ADDRESS);
    }
  }

   if (remoteButtonPushed)
  {
    remoteButtonPushed=false;
    if (remoteMode) remoteMode=false;
    else remoteMode=true;
  }

  
}

//Check if buttons have been pressed
void checkButtons()
{
  //Check if Level_Button has been pushed
  levelButtonStatus=digitalRead(LEVEL_BUTTON);

  if (levelButtonStatus != levelButtonLastStatus){

    if (levelButtonStatus==HIGH) 
    {
      levelButtonPushed=true;
      delay(ANTIREBOUND);
    }
  }
  levelButtonLastStatus=levelButtonStatus;

  //Check if Tx_Button has been pushed
  txButtonStatus=digitalRead(TX_BUTTON);
  if (txButtonStatus != txButtonLastStatus){
    if (txButtonStatus==HIGH) 
    {
      txButtonPushed=true;
      delay(ANTIREBOUND);
    }
  }
  txButtonLastStatus=txButtonStatus;

  //Check if Remote Mode button has been pushed
  remoteButtonStatus=digitalRead(REMOTE_BUTTON);
  if (remoteButtonStatus != remoteButtonLastStatus){
    if (remoteButtonStatus==HIGH) 
    {
      remoteButtonPushed=true;
      delay(ANTIREBOUND);
    }
  }
  remoteButtonLastStatus=remoteButtonStatus;

  //Check if Erase button is kept pushed
  if (digitalRead(ERASE_BUTTON)==LOW) eraseButtonPushed=true;
  else eraseButtonPushed=false;
}


//Extracts Locked/Not Locked, Lat, Lon, N/S, W/E from received frame
void processFrame()
{
  int i=0;
  int j=0;
  int field=0;
  //read validity flag
  for (i=0;(i<MAX_FRAME_LENGTH) && (field != 6);i++)
  {
    if (frame[i]==',') field++;
  }
  if (field==6){
    validData=frame[i];
  } else validData='V';

  //Check if data is valid, i.e. position is fixed
  if (validData=='A'){
    gpsLocked=true;
    //read Latitude
    i=0;
    field=0;
    for (i=0;(i<MAX_FRAME_LENGTH) && (frame[i]!=',');i++);
    if (frame[i]==','){
      i++;
      j=0;
      for (i;(i<MAX_FRAME_LENGTH) && (frame[i]!=',');i++){
        lat[j]=frame[i];
        j++;
      }
      if (frame[i]==','){
        lat[j]='\0';
        i++;
        //Read dirNS
        dirNS=frame[i];
        i++;
        i++;
        j=0;
        //Read Longitude
        for (i;(i<MAX_FRAME_LENGTH) && (frame[i]!=',');i++) {
          lon[j]=frame[i];
          j++;
        }
        if (frame[i]==','){
          lon[j]='\0';
          i++;
          //Read dirWE
          dirWE=frame[i];
        }
      }
    }
    
    
  } else {
    //position not valid
    gpsLocked=false;
    lat[0]='0';lat[1]='\0';
    lon[0]='0';lon[1]='\0';
    dirNS='X';
    dirWE='X';
  }
  
}

//Process commands from remote computer
void processUserFrame()
{
  String tempString=String(userFrame);
  String toPrint=String("");
  unsigned int tempInt;
  byte tempByte;

  if (tempString.length() >= MAX_USER_HEADER_LENGTH)
  {
  //extract command type
  String tempField=tempString.substring(0,3);

  if (!tempField.compareTo("$GP"))
  {
   EEPROM.get(TXID_ADDRESS,tempInt);
   toPrint.concat(tempInt);
   toPrint.concat(",");
   tempByte=EEPROM.read(TXNUM_ADDRESS);
   toPrint.concat(tempByte);
   toPrint.concat(",");
   tempByte=EEPROM.read(HIG_POWER_ADDRESS);
   toPrint.concat(tempByte);
   toPrint.concat(",");
   tempByte=EEPROM.read(MED_POWER_ADDRESS);
   toPrint.concat(tempByte);
   toPrint.concat(",");
   tempByte=EEPROM.read(LOW_POWER_ADDRESS);
   toPrint.concat(tempByte);
   Serial.println(toPrint);
   Serial.println("EOF");
       
  } else if (!tempField.compareTo("$GD"))
  {
    sigfoxData tempData;
    unsigned int tempAddress;
    tempAddress=START_REG_ADDRESS;
    for (int i=0;i<txnum;i++)
    {
      toPrint="";
      EEPROM.get(tempAddress,tempData);
      toPrint.concat(tempData.txId);
      toPrint.concat(',');
      toPrint.concat(tempData.txLevel);
      toPrint.concat(',');
      toPrint.concat(tempData.latitude);
      toPrint.concat(',');
      toPrint.concat(tempData.longitude);     
      Serial.println(toPrint); 
      tempAddress += BYTES_TO_SEND;
    }
    Serial.println("EOF");
  } else if (!tempField.compareTo("$HL"))
  {
    if (tempString.length() == 6)
    {
      tempField=tempString.substring(4,6);
      if (tempField.startsWith("0") || tempField.startsWith("1"))
      {
        tempByte=(byte)tempField.toInt();
        if ((tempByte >= 0) && (tempByte <= 14))
        {
          EEPROM.write(HIG_POWER_ADDRESS,tempByte);
        }
      }     
    }
  } else if (!tempField.compareTo("$ML"))
  {
    if (tempString.length() == 6)
    {
      tempField=tempString.substring(4,6);
      if (tempField.startsWith("0") || tempField.startsWith("1"))
      {
        tempByte=(byte)tempField.toInt();
        if ((tempByte >= 0) && (tempByte <= 14))
        {
          EEPROM.write(MED_POWER_ADDRESS,tempByte);
        }
      }     
    }
  } else if (!tempField.compareTo("$LL"))
  {
    if (tempString.length() == 6)
    {
      tempField=tempString.substring(4,6);
      if (tempField.startsWith("0") || tempField.startsWith("1"))
      {
        tempByte=(byte)tempField.toInt();
        if ((tempByte >= 0) && (tempByte <= 14))
        {
          EEPROM.write(LOW_POWER_ADDRESS,tempByte);
        }
      }     
    }
  }

  
  }

}

void prepareSigfoxPayload()
{
    long tempLong;
    //Prepare sigfox payload
    payload.txId=txid;
    payload.txLevel=level;
    
    //prepare Latitude
    String tempString=String(lat);
    int decimalSeparator=tempString.indexOf('.');
    String firstPart=tempString.substring(0,decimalSeparator);
    String secondPart=tempString.substring(decimalSeparator+1);
    tempString=String(firstPart);
    tempString.concat(secondPart);
    //Convert to long
    tempLong=tempString.toInt();
    //Change sign if necessary
    if (dirNS=='S') tempLong=0-tempLong;
    //load to payload
    payload.latitude=tempLong;

    //prepare Longitude
    tempString=String(lon);
    decimalSeparator=tempString.indexOf('.');
    firstPart=tempString.substring(0,decimalSeparator);
    secondPart=tempString.substring(decimalSeparator+1);
    tempString=String(firstPart);
    tempString.concat(secondPart);
    //Convert to long
    tempLong=tempString.toInt();
    //Change sign if necessary
    if (dirWE=='W') tempLong=0-tempLong;
    //load to payload
    payload.longitude=tempLong;
    


}

//Led status if EEPROM is full
void onEepromFull()
{
  digitalWrite(HIG_LED,HIGH);
  digitalWrite(MED_LED,LOW);
  digitalWrite(LOW_LED,HIGH);
}

//Led status if EEPROM has been deleted
void onEepromErase()
{
  digitalWrite(HIG_LED,LOW);
  digitalWrite(MED_LED,LOW);
  digitalWrite(LOW_LED,LOW);
}

//Main loop
void loop() {

  if (!remoteMode) digitalWrite(TX_LED,LOW);
  
  ledStatus();
  checkButtons();

  //If erase button has been pushed for more than SECONDS_TO_ERASE seconds, then eeprom will be erased
  if (secondsEraseButtonPushed >= SECONDS_TO_ERASE)
  {
    eraseButtonPushed=false;
    secondsEraseButtonPushed=0;
    txnum=0;
    EEPROM.write(TXNUM_ADDRESS,txnum);
    onEepromErase();
  }

  //Check if transmission needs to be performed. 
  //Conditions to start transmission are GPS LOCKED and EEPROM not FULL

  //Disable Transmission and GPS decoding if in remoteMode
  if(!remoteMode)
  {
  if (txButtonPushed && gpsLocked && !eepromFull)
  {
    txButtonPushed=false;
    //check the class of transmission to be performed:
    //If remoteButton is pressed when txButton is released, then transmission with ACK
    //Else normal transmission without ACK
    if (remoteButtonLastStatus==LOW) txACK=true;
    else txACK=false;

    //Release last status of remoteButton. This prevents to set the mode in Remote after finish transmission
    remoteButtonLastStatus=HIGH;
    
    digitalWrite(TX_LED,HIGH);
   
    prepareSigfoxPayload();

    //fill the byte array to be sent
    //Caution in the backend: arduino is LITTLE-ENDIAN
    memcpy(dataReadyToSend, &payload,BYTES_TO_SEND);
    
    //switch On Sigfox module
    sigfoxError=Sigfox.ON(SOCKET0);
    if (sigfoxError==0)
    {
      //Set module power
      sigfoxError=Sigfox.setPower(payload.txLevel);

      if (sigfoxError==0)
      {

        //Transmit with ACK if txACK=true. This can take a long time (up to 1 minute)
        if (txACK)
        {
          sigfoxError=Sigfox.sendACK(dataReadyToSend,BYTES_TO_SEND);
        } else
        {
          sigfoxError=Sigfox.send(dataReadyToSend,BYTES_TO_SEND);
        }
        
        if (sigfoxError!=0)
        {
          onSigfoxError();
          //Update rxACK variable
          rxACK=false;
          digitalWrite(TX_LED,LOW);
        } else
        {
          //if transmission OK, increment txid and txnum and store new values in EEPROM
          //Update rxACK variable
          if (txACK)
          {
            if (Sigfox._length>30)
            {
              //_lenght stores the size of the received response
              //Response for a correct ACK received is (+RX here is sample data):
              //<CR><LF>+RX=55 fa 9c 04 00 00 ff a5<CR><LF>
              //+RX END<CR><LF>             
              rxACK=true;
            } else rxACK=false;
          }
          else rxACK=false;
          
          //First indicates that ACK was received flahing txLED during 10 seconds
          if (rxACK)
          {
            for (int i=0;i<20;i++)
            {
              digitalWrite(TX_LED,digitalRead(TX_LED)^1);
              delay(500);
            }
            digitalWrite(TX_LED,LOW);
          } else
          {
            digitalWrite(TX_LED,LOW);
          }

          //Store in EEPROM. Update de value of payload.txLevel according to txACK and rxACK
          //Bit 5 corresponds to txACK
          //Bit 6 corresponds to rxACK
          payload.txLevel=payload.txLevel & 0x0F;
          if (txACK) payload.txLevel=payload.txLevel | 0x10;

          if (rxACK) payload.txLevel=payload.txLevel | 0x20;
         
          txid++;
          txnum++;
          EEPROM.put(TXID_ADDRESS,txid);
          EEPROM.write(TXNUM_ADDRESS,txnum);

          //calculate eeprom address to store register
          address=START_REG_ADDRESS;
          for (int i=1;i<txnum;i++)
          {
            address += BYTES_TO_SEND;
          }
          //store register
          EEPROM.put(address,payload);
  

          //Check if eeprom is full
          if (txnum > TX_MAX)
          {
            eepromFull=true;
            onEepromFull();
          }
        }
      } else onSigfoxError();
    } else onSigfoxError();

    
  } else
  {
    txButtonPushed=false;
  }

 //Check if there are data from GPS and fill frame of interest ($GPGLL)
  if(myGps.available())
  {
    gpsChar=myGps.read();
    if (fillingFrame && (indexFrame < MAX_FRAME_LENGTH))
    {
      if (gpsChar=='$')
      {
        //End of Frame reached
        frame[indexFrame]='\0';
        fillingFrame=false;
        frameCompleted=true;
        indexFrame=0;
      } else
      {
        frame[indexFrame]=gpsChar;
        indexFrame++; 
      }
      
    } else if (fillingHeader && (indexFrame < MAX_HEADER_LENGTH))
      {
      frame[indexFrame]=gpsChar;
      indexFrame++;
      if (indexFrame>=MAX_HEADER_LENGTH)
      {
        //Header completed. Check if match to the one we are looking for
        frame[indexFrame]='\0';
        if (strcmp(frame,"$GPGLL")==0)
        {
          fillingFrame=true;
          fillingHeader=false;
        } else
        {
          fillingHeader=false;
          indexFrame=0;
        }
      }
    }  else if (gpsChar=='$')
    {
      fillingHeader=true;
      frame[indexFrame]=gpsChar;
      indexFrame++;    
    }
    if (frameCompleted) 
    {
      processFrame();
      frameCompleted=false;
    }
  }

  } else
  {
    //In remoteMode. Wait for commands from computer
    if(Serial.available())
    {
    userChar=Serial.read();
    if (fillingUserFrame && (indexUserFrame < MAX_USER_FRAME_LENGTH))
    {
      if (userChar=='$')
      {
        //End of Frame reached
        userFrame[indexUserFrame]='\0';
        fillingUserFrame=false;
        userFrameCompleted=true;
        indexUserFrame=0;
      } else
      {
        userFrame[indexUserFrame]=userChar;
        indexUserFrame++; 
      }
      
    }   else if (userChar=='$')
    {
      fillingUserFrame=true;
      userFrame[indexUserFrame]=userChar;
      indexUserFrame++;    
    }
    
    if (userFrameCompleted) 
    {
      processUserFrame();

      userFrameCompleted=false;
    }
  }
  }

}
