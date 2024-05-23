/*
  SPDX-FileCopyrightText: 2024 Turm Design Works LLC. <info@turmdesignworks.com>
  SPDX-License-Identifier: MIT

  Seeed SAMD21 version 1.8.3 required.
*/

#include "MIDIUSB.h"
#include "Wire.h"

// average calc times
const byte AVGCT = 5;

// S Pins Switch
const int Pin_S[4] = {10, 12, 6, 7};

// V Pins Value (1-RING, 1-TIP, 2-RING, 2-TIP, 1-EXP, 2-EXP)
const int Pin_V[6] = {11, A4, 13, A3, A2, A1};

// Slide SW Pin
const int Pin_SL = 5;

// 7seg Pin
const int Pin_SER = A5;
const int Pin_SCLK = 2;
const int Pin_RCLK = A0;

// DIP Switch Pin
const int Pin_D[4] = {3, 4, 9, 8};

// DIP Switch Values
byte DIPV[4] = {0, 0, 0, 0};

// DIP Switch result
byte DIPNUM = 0;
byte DIPNUMp = 0;

// FS Random Value
byte RAND[4] = {0, 0, 0, 0};

/////////////////////////////////////////////// Function Mode
/////////////////////////////////////////////// FS : 0.Latch CC, 1.Moment CC, 2.Random CC, 3.PGM, 4.Note, 5.BANK++, 6.BANK--
/////////////////////////////////////////////// EXP: 0.CC, 1.CC (Reverse)
byte MODE[6] = {4, 4, 5, 6, 0, 0};

// BANK Value
byte BANK = 0;
byte BANKp = 255;

/////////////////////////////////////////////// FS+EXP MIDI Channel
byte MIDICH[6] = {0, 0, 0, 0, 0, 0};

/////////////////////////////////////////////// FS+EXP CCID
byte CCID[6] = {64, 65, 66, 67, 7, 11};

/////////////////////////////////////////////// EXP Curve
byte curve[2] = {1, 1};

/////////////////////////////////////////////// FS+EXP CC Minimum Value
byte MINV[6] = {0, 0, 0, 0, 0, 0};
/////////////////////////////////////////////// FS+EXP CC Maximum Value
byte MAXV[6] = {127, 127, 127, 127, 127, 127};

/////////////////////////////////////////////// FS CC Random Minimum Value
byte MINR[4] = {0, 0, 0, 0};
/////////////////////////////////////////////// FS CC Random Maximum Value
byte MAXR[4] = {127, 127, 127, 127};

/////////////////////////////////////////////// FS Program Change Value 0~127 outputs 1~128
byte PGMV[4] = {19, 20, 21, 22};

/////////////////////////////////////////////// FS Note Number
byte NOTE[4] = {48, 50, 52, 53};

/////////////////////////////////////////////// FS Velocity
byte VELO[4] = {64, 64, 64, 64};

/////////////////////////////////////////////// FS POLALITY TYPE
byte NCO[4] = {0, 0, 1, 1};

/////////////////////////////////////////////// EEPROM DATA to BANK
/////////////////////////////////////////////// 0~5:MODE, 6~11:MIDICH, 12~17:CCID, 18~19:Curve, 20~25:MINV, 26~31:MAXV,
/////////////////////////////////////////////// 32~35:MINR, 36~39:MAXR, 40~43:PGMV, 44~47:NOTE, 48~51:VELO, 52~55:NCO
byte EEPV[10][56] = { 
    { /*MODE*/ 1, 1, 6, 5, 0, 0, /*MIDICH*/ 0, 0, 0, 0, 0, 0, /*CCID*/ 64, 69, 0, 0, 7, 11, /*Curve*/ 1, 1, /*MINV*/ 0, 0, 0, 0, 0, 0, /*MAXV*/ 127, 127, 127, 127, 127, 127, /*MINR*/ 0, 0, 0, 0,  /*MAXR*/ 127, 127, 127, 127, /*PGMV*/ 0, 0, 0, 0, /*NOTE*/ 48, 50, 52, 53, /*VELO*/ 64, 64, 64, 64, /*NCO*/ 0, 0, 0, 0 },
    { /*MODE*/ 0, 0, 6, 5, 0, 0, /*MIDICH*/ 0, 0, 0, 0, 0, 0, /*CCID*/ 82, 83, 0, 0, 7, 11, /*Curve*/ 1, 1, /*MINV*/ 0, 0, 0, 0, 0, 0, /*MAXV*/ 127, 127, 127, 127, 127, 127, /*MINR*/ 0, 0, 0, 0,  /*MAXR*/ 127, 127, 127, 127, /*PGMV*/ 0, 0, 0, 0, /*NOTE*/ 48, 50, 52, 53, /*VELO*/ 64, 64, 64, 64, /*NCO*/ 0, 0, 0, 0 },
    { /*MODE*/ 2, 2, 6, 5, 0, 0, /*MIDICH*/ 0, 0, 0, 0, 0, 0, /*CCID*/ 1, 2, 0, 0, 7, 11, /*Curve*/ 1, 1, /*MINV*/ 0, 0, 0, 0, 0, 0, /*MAXV*/ 127, 127, 127, 127, 127, 127, /*MINR*/ 0, 0, 0, 0,  /*MAXR*/ 127, 127, 127, 127, /*PGMV*/ 0, 0, 0, 0, /*NOTE*/ 48, 50, 52, 53, /*VELO*/ 64, 64, 64, 64, /*NCO*/ 0, 0, 0, 0 },
    { /*MODE*/ 3, 3, 6, 5, 0, 0, /*MIDICH*/ 0, 0, 0, 0, 0, 0, /*CCID*/ 0, 0, 0, 0, 7, 11, /*Curve*/ 1, 1, /*MINV*/ 0, 0, 0, 0, 0, 0, /*MAXV*/ 127, 127, 127, 127, 127, 127, /*MINR*/ 0, 0, 0, 0,  /*MAXR*/ 127, 127, 127, 127, /*PGMV*/ 1, 2, 0, 0, /*NOTE*/ 48, 50, 52, 53, /*VELO*/ 64, 64, 64, 64, /*NCO*/ 0, 0, 0, 0 },
    { /*MODE*/ 4, 4, 6, 5, 0, 0, /*MIDICH*/ 0, 0, 0, 0, 0, 0, /*CCID*/ 0, 0, 0, 0, 7, 11, /*Curve*/ 1, 1, /*MINV*/ 0, 0, 0, 0, 0, 0, /*MAXV*/ 127, 127, 127, 127, 127, 127, /*MINR*/ 0, 0, 0, 0, /*MAXR*/ 127, 127, 127, 127, /*PGMV*/ 0, 0, 0, 0, /*NOTE*/ 48, 50, 52, 53, /*VELO*/ 64, 64, 64, 64, /*NCO*/ 0, 0, 0, 0 },
    { /*MODE*/ 1, 1, 6, 5, 1, 1, /*MIDICH*/ 0, 0, 0, 0, 0, 0, /*CCID*/ 64, 69, 0, 0, 7, 11, /*Curve*/ 1, 1, /*MINV*/ 0, 0, 0, 0, 0, 0, /*MAXV*/ 127, 127, 127, 127, 127, 127, /*MINR*/ 0, 0, 0, 0, /*MAXR*/ 127, 127, 127, 127, /*PGMV*/ 0, 0, 0, 0, /*NOTE*/ 48, 50, 52, 53, /*VELO*/ 64, 64, 64, 64, /*NCO*/ 0, 0, 0, 0 },
    { /*MODE*/ 0, 0, 6, 5, 1, 1, /*MIDICH*/ 0, 0, 0, 0, 0, 0, /*CCID*/ 82, 83, 0, 0, 7, 11, /*Curve*/ 1, 1, /*MINV*/ 0, 0, 0, 0, 0, 0, /*MAXV*/ 127, 127, 127, 127, 127, 127, /*MINR*/ 0, 0, 0, 0, /*MAXR*/ 127, 127, 127, 127, /*PGMV*/ 0, 0, 0, 0, /*NOTE*/ 48, 50, 52, 53, /*VELO*/ 64, 64, 64, 64, /*NCO*/ 0, 0, 0, 0 },
    { /*MODE*/ 2, 2, 6, 5, 1, 1, /*MIDICH*/ 0, 0, 0, 0, 0, 0, /*CCID*/ 1, 2, 0, 0, 7, 11, /*Curve*/ 1, 1, /*MINV*/ 0, 0, 0, 0, 0, 0, /*MAXV*/ 127, 127, 127, 127, 127, 127, /*MINR*/ 0, 0, 0, 0, /*MAXR*/ 127, 127, 127, 127, /*PGMV*/ 0, 0, 0, 0, /*NOTE*/ 48, 50, 52, 53, /*VELO*/ 64, 64, 64, 64, /*NCO*/ 0, 0, 0, 0 },
    { /*MODE*/ 3, 3, 6, 5, 1, 1, /*MIDICH*/ 0, 0, 0, 0, 0, 0, /*CCID*/ 0, 0, 0, 0, 7, 11, /*Curve*/ 1, 1, /*MINV*/ 0, 0, 0, 0, 0, 0, /*MAXV*/ 127, 127, 127, 127, 127, 127, /*MINR*/ 0, 0, 0, 0, /*MAXR*/ 127, 127, 127, 127, /*PGMV*/ 1, 2, 0, 0, /*NOTE*/ 48, 50, 52, 53, /*VELO*/ 64, 64, 64, 64, /*NCO*/ 0, 0, 0, 0 },
    { /*MODE*/ 4, 4, 6, 5, 1, 1, /*MIDICH*/ 0, 0, 0, 0, 0, 0, /*CCID*/ 0, 0, 0, 0, 7, 11, /*Curve*/ 1, 1, /*MINV*/ 0, 0, 0, 0, 0, 0, /*MAXV*/ 127, 127, 127, 127, 127, 127, /*MINR*/ 0, 0, 0, 0, /*MAXR*/ 127, 127, 127, 127, /*PGMV*/ 0, 0, 0, 0, /*NOTE*/ 48, 50, 52, 53, /*VELO*/ 64, 64, 64, 64, /*NCO*/ 0, 0, 0, 0 },
  };

// Jack Switch Values
byte ON[4] = {0, 0, 0, 0};
byte ONp[4] = {0, 0, 0, 0};

//EXP FLAG
byte outFLAG[2] = {0, 0};
byte outFLAGp[2] = {0, 0};

// Read Value is "times of for()"
int VAL[6] = {0, 0, 0, 0, 0, 0};
int VALp[6] = {0, 0, 0, 0, 0, 0};

// Foot Switch State for output pipe*~ delay
byte FS_State[4] = {0, 0, 0, 0};
byte FSp_State[4] = {0, 0, 0, 0};

// FootSwitch result Value
byte FootSwitch[4] = {0, 0, 0, 0};

// EXP Average of Vals
int AVV[2] = {0, 0};
int AVVp[2] = {0, 0};

//detect maxexpvalue
int MAXSTOMP[2] = {1011, 1011};

//detect minexpvalue
int MINSTOMP[2] = {24, 24};

//EXP sorted
byte EXP[2] = {0, 0};
byte EXPp[2] = {0, 0};

//EXP result
int EXPCCV[2] = {0, 0};
int EXPCCVp[2] = {0, 0};

//for output waiting time
unsigned long CTM[4] = {0, 0, 0, 0};

//for calc waiting time
unsigned long VTM[6] = {0, 0, 0, 0, 0, 0};

//SIDE-SW Value
byte SLSW = 0;
byte SLSWp = 0;
byte LCDR = 0;


//MIDI for USB

void controlChange(byte channel, byte control, byte value) {
  midiEventPacket_t event = {0x0B, 0xB0 | channel, control, value};
  MidiUSB.sendMIDI(event);
}

void programChange(byte channel, byte program) {
  midiEventPacket_t pc = {0x0C, 0xC0 | channel, program, 0};
  MidiUSB.sendMIDI(pc);
}

void noteOn(byte channel, byte pitch, byte velocity) {
  midiEventPacket_t noteOn = {0x09, 0x90 | channel, pitch, velocity};
  MidiUSB.sendMIDI(noteOn);
}

void noteOff(byte channel, byte pitch, byte velocity) {
  midiEventPacket_t noteOff = {0x08, 0x80 | channel, pitch, velocity};
  MidiUSB.sendMIDI(noteOff);
}

void SysExStart(byte ROW, byte COL) {
  midiEventPacket_t event = {0x04, 0xF0, ROW, COL};
  MidiUSB.sendMIDI(event);
}

void SysExEND(byte sVAL ) {
  midiEventPacket_t event = {0x06, sVAL,0xF7, 0x00};
  MidiUSB.sendMIDI(event);
}



//MIDI for DIN5

void D_controlChange(byte channel, byte control, byte value) {
  Serial1.write( 0xB0 | channel );
  Serial1.write( control );
  Serial1.write( value );
  Serial1.flush();
}

void D_programChange(byte channel, byte value) {
  Serial1.write( 0xC0 | channel );
  Serial1.write( value );
  Serial1.flush();
}

void D_noteOn(byte channel, byte pitch, byte velocity) {
  Serial1.write( 0x90 | channel );
  Serial1.write( pitch );
  Serial1.write( velocity );
  Serial1.flush();
}

void D_noteOff(byte channel, byte pitch, byte velocity) {
  Serial1.write( 0x80 | channel );
  Serial1.write( pitch );
  Serial1.write( velocity );
  Serial1.flush();
}



//Sync

uint8_t Sstart[4] = {0x0F, 0xFA, 0, 0};

uint8_t Sstop[4] = {0x0F, 0xFC, 0, 0};

uint8_t Sclock[4] = {0x0F, 0xF8, 0, 0};






//EEPROM

int Wadd;
byte waitEEP;

byte ROWid;
byte COLid;

void WriteEEP(int d_address, unsigned int rom_address, byte s_data){
  Wire.beginTransmission(d_address);
  Wire.write((int)(rom_address >> 8));
  Wire.write((int)(rom_address & 0xFF));
  Wire.write(s_data);
  Wire.endTransmission();
}

byte ReadEEP(int d_address, unsigned int rom_address){
  Wire.beginTransmission(d_address);
  Wire.write((int)(rom_address >> 8));
  Wire.write((int)(rom_address & 0xFF));
  Wire.endTransmission();

  Wire.requestFrom(d_address,1);
  if(Wire.available()){}
  return Wire.read();
}  





//7 SEGMENT Patterns OSL10561-LB

byte SEGP[11] = {
    B10111110, //0       0
    B00101000, //1       1
    B10011101, //2       2
    B00111101, //3       3
    B00101011, //4       4
    B00110111, //5       5
    B10110111, //6       6
    B00101110, //7       7
    B10111111, //8       8
    B00111111, //9       9
    B00000000, //Null    10
};

byte SEGPr[11] = {
    B10111110, //0       0
    B10000010, //1       1
    B10011101, //2       2
    B10010111, //3       3
    B10100011, //4       4
    B00110111, //5       5
    B00111111, //6       6
    B10110010, //7       7
    B10111111, //8       8
    B10110111, //9       9
    B00000000, //Null    10
};





void setup() {

  
  /////////////////////////////////////////////////////////////////// PinMode SetUp
  
  for (byte i=0; i <= 3; i++){
    pinMode( Pin_S[i], INPUT );
    pinMode( Pin_D[i], INPUT );
  }

  pinMode( Pin_V[0], INPUT_PULLUP );
  pinMode( Pin_V[1], INPUT_PULLUP );
  pinMode( Pin_V[2], INPUT_PULLUP );
  pinMode( Pin_V[3], INPUT_PULLUP );

  pinMode( Pin_SL, INPUT_PULLUP );

  pinMode( Pin_SER, OUTPUT );
  pinMode( Pin_SCLK, OUTPUT );
  pinMode( Pin_RCLK, OUTPUT );

  delay(100);

  digitalWrite( Pin_RCLK, LOW );
  shiftOut( Pin_SER, Pin_SCLK, LSBFIRST, SEGP[0] );
  digitalWrite( Pin_RCLK, HIGH );



  /////////////////////////////////////////////////////////////////// LCD BLACKOUT
  
  digitalWrite( Pin_RCLK, LOW );
  shiftOut( Pin_SER, Pin_SCLK, LSBFIRST, SEGP[10] );
  digitalWrite( Pin_RCLK, HIGH );


  
  /////////////////////////////////////////////////////////////////// Transmission Begin

  Serial1.begin(31250);  
  SerialUSB.begin(115200);
  Wire.begin();
  //WriteEEP(0x50, 0, 88);
  delay (1000);



  /////////////////////////////////////////////////////////////////// Read EEPROM

  for ( byte i=0; i <= 55; i++ ){
    if ( int r=0; ReadEEP(0x50, i) <= 127 ){ EEPV[r][i] = ReadEEP(0x50, i); delay(5); } else { delay(5); }
    if ( int r=1, p=56; ReadEEP(0x50, i+p) <= 127 ){ EEPV[r][i] = ReadEEP(0x50, i+p); delay(5); } else { delay(5); }
    if ( int r=2, p=112; ReadEEP(0x50, i+p) <= 127 ){ EEPV[r][i] = ReadEEP(0x50, i+p); delay(5); } else { delay(5); }
    if ( int r=3, p=168; ReadEEP(0x50, i+p) <= 127 ){ EEPV[r][i] = ReadEEP(0x50, i+p); delay(5); } else { delay(5); }
    if ( int r=4, p=224; ReadEEP(0x50, i+p) <= 127 ){ EEPV[r][i] = ReadEEP(0x50, i+p); delay(5); } else { delay(5); }
    if ( int r=5, p=280; ReadEEP(0x50, i+p) <= 127 ){ EEPV[r][i] = ReadEEP(0x50, i+p); delay(5); } else { delay(5); }
    if ( int r=6, p=336; ReadEEP(0x50, i+p) <= 127 ){ EEPV[r][i] = ReadEEP(0x50, i+p); delay(5); } else { delay(5); }
    if ( int r=7, p=392; ReadEEP(0x50, i+p) <= 127 ){ EEPV[r][i] = ReadEEP(0x50, i+p); delay(5); } else { delay(5); }
    if ( int r=8, p=448; ReadEEP(0x50, i+p) <= 127 ){ EEPV[r][i] = ReadEEP(0x50, i+p); delay(5); } else { delay(5); }
    if ( int r=9, p=504; ReadEEP(0x50, i+p) <= 127 ){ EEPV[r][i] = ReadEEP(0x50, i+p); delay(5); } else { delay(5); }
    
    if ( i>=0 && i<=7 ) { digitalWrite( Pin_RCLK, LOW ); shiftOut( Pin_SER, Pin_SCLK, LSBFIRST, B00000001 ); digitalWrite( Pin_RCLK, HIGH ); }
    if ( i>=8 && i<=15 ) { digitalWrite( Pin_RCLK, LOW ); shiftOut( Pin_SER, Pin_SCLK, LSBFIRST, B00000010 ); digitalWrite( Pin_RCLK, HIGH ); }
    if ( i>=16 && i<=23 ) { digitalWrite( Pin_RCLK, LOW ); shiftOut( Pin_SER, Pin_SCLK, LSBFIRST, B00000100 ); digitalWrite( Pin_RCLK, HIGH ); }
    if ( i>=24 && i<=31 ) { digitalWrite( Pin_RCLK, LOW ); shiftOut( Pin_SER, Pin_SCLK, LSBFIRST, B00001000 ); digitalWrite( Pin_RCLK, HIGH ); }
    if ( i>=32 && i<=39 ) { digitalWrite( Pin_RCLK, LOW ); shiftOut( Pin_SER, Pin_SCLK, LSBFIRST, B00100000 ); digitalWrite( Pin_RCLK, HIGH ); }
    if ( i>=40 && i<=47 ) { digitalWrite( Pin_RCLK, LOW ); shiftOut( Pin_SER, Pin_SCLK, LSBFIRST, B00010000 ); digitalWrite( Pin_RCLK, HIGH ); }
    if ( i>=48 && i<=55 ) { digitalWrite( Pin_RCLK, LOW ); shiftOut( Pin_SER, Pin_SCLK, LSBFIRST, B10000000 ); digitalWrite( Pin_RCLK, HIGH ); }
    
  }
  

}


void loop() {


/////////////////////////////////////////////////////////////////////////////////////////////// MIDI Rx

  midiEventPacket_t rx;
  do {
    rx = MidiUSB.read();
    if ( rx.header != 0 ) {
      Serial.print("MIDI Rx: ");
      Serial.print(rx.header);
      Serial.print(", ");
      Serial.print(rx.byte1);
      Serial.print(", ");
      Serial.print(rx.byte2);
      Serial.print(", ");
      Serial.println(rx.byte3);

///////////////////////////////////////////////////////////// CC from HOST
      if ( rx.header == 11 ){
        D_controlChange( rx.byte1 - 176, rx.byte2, rx.byte3 ); 
      }
      
///////////////////////////////////////////////////////////// NOTE ON from HOST
      else if ( rx.header == 9 ){ 
        D_noteOn( rx.byte1 - 144, rx.byte2, rx.byte3 ); 
      }
      
///////////////////////////////////////////////////////////// NOTE OFF from HOST
      else if ( rx.header == 8 ){ 
        D_noteOn( rx.byte1 - 128, rx.byte2, rx.byte3 ); 
      }

///////////////////////////////////////////////////////////// Poly & After touch
      else if ( rx.header == 10 ){ 
        Serial1.write(rx.byte1);
        Serial1.write(rx.byte2);
        Serial1.write(rx.byte3);
      }

///////////////////////////////////////////////////////////// Channel After touch
      else if ( rx.header == 13 ){ 
        Serial1.write(rx.byte1);
        Serial1.write(rx.byte2);
      }
      
///////////////////////////////////////////////////////////// Pitch Bend
      else if ( rx.header == 14 ){ 
        Serial1.write(rx.byte1);
        Serial1.write(rx.byte2);
        Serial1.write(rx.byte3);
      }

///////////////////////////////////////////////////////////// 0 (future expanssion)
      else if ( rx.header == 0 ){ 
        
      }

///////////////////////////////////////////////////////////// 1 (Cable event. future expanssion)
      else if ( rx.header == 0 ){ 
        
      }

///////////////////////////////////////////////////////////// 2 Byte System Common MSG
      else if ( rx.header == 2 ){ 
        Serial1.write(rx.byte1);
        Serial1.write(rx.byte2);
      }

///////////////////////////////////////////////////////////// 3 Byte System Common MSG
      else if ( rx.header == 3 ){ 
        Serial1.write(rx.byte1);
        Serial1.write(rx.byte2);
        Serial1.write(rx.byte3);
      }
      
///////////////////////////////////////////////////////////// Single Byte Clock
      else if ( rx.header == 15 ){ 
        Serial1.write(rx.byte1);
      }
      
///////////////////////////////////////////////////////////// Program Change from HOST
      else if ( rx.header == 12 ){
        D_programChange( rx.byte1 - 192, rx.byte2 ); 
                                                           // PGM to BANNK (CH.16)
        if ( rx.byte1 == 207 ){
          BANK = constrain( rx.byte2, 0, 9);
        }
      }
      
///////////////////////////////////////////////////////////// System Exclusive from HOST
      else if ( rx.header == 4 ){
        waitEEP = 0;
        Serial1.write(rx.byte1);
        Serial1.write(rx.byte2);
        Serial1.write(rx.byte3);

        if ( rx.byte2 >= 100 && rx.byte2 <= 110 ){
          ROWid = rx.byte2;
          COLid = rx.byte3;
          Wadd = rx.byte3 + ( 55 * (rx.byte2 - 100) + (rx.byte2 - 100) );
          //Serial.print("Address: ");
          //Serial.println(Wadd);
          waitEEP = 1;
        }
      }

      else if ( rx.header == 5 ){ 
        Serial1.write(rx.byte1);
        Serial1.write(rx.byte2);
        Serial1.write(rx.byte3);
      }

      else if ( rx.header == 6 ){ 
        Serial1.write(rx.byte1);
        Serial1.write(rx.byte2);
        Serial1.write(rx.byte3);

        if ( waitEEP == 1 ){
          WriteEEP( 0x50, Wadd, rx.byte1 );
          delay(250);
          SysExStart( ROWid, COLid );
          SysExEND( 1 );
          MidiUSB.flush();
        }
      }

      else if ( rx.header == 7 ){ 
        Serial1.write(rx.byte1);
        Serial1.write(rx.byte2);
        Serial1.write(rx.byte3);
      }
        
    }
  } while (rx.header != 0);




/////////////////////////////////////////////////////////////////////////////////////////////// SLIDE SWITCH

  SLSW = digitalRead( Pin_SL );

  if ( SLSW != SLSWp ){
    LCDR = SLSW;
    SLSWp = SLSW;
  }

  

/////////////////////////////////////////////////////////////////////////////////////////////// JACK Connection

  for ( byte i=0; i <= 3; i++ ){ ON[i] = digitalRead(Pin_S[i]); }

  //SW1&2
  
  if ( byte i=0, ii=1; ON[i] != ONp[i] ){ 
    ONp[i] = ON[i]; 
//    noteOff(MIDICH[i], NOTE[i], VELO[i]);
//    noteOff(MIDICH[ii], NOTE[ii], VELO[ii]);
//    D_noteOff(MIDICH[i], NOTE[i], VELO[i]);
//    D_noteOff(MIDICH[ii], NOTE[ii], VELO[ii]);
    CTM[i] = millis() + 500; 
    delay(10); 
  }

  //SW3&4
  
  if ( byte i=1, ii=2, iii=3; ON[i] != ONp[i] ){ 
    ONp[i] = ON[i]; 
//    noteOff(MIDICH[ii], NOTE[ii], VELO[ii]);
//    noteOff(MIDICH[iii], NOTE[iii], VELO[iii]);
//    D_noteOff(MIDICH[ii], NOTE[ii], VELO[ii]);
//    D_noteOff(MIDICH[iii], NOTE[iii], VELO[iii]);
    CTM[i] = millis() + 500; 
    delay(10); 
  }

  //EXP1
  
  if ( byte i=2; ON[i] != ONp[i] ){ 
    ONp[i] = ON[i]; 
    CTM[i] = millis() + 500; 
    delay(10); 
  }

  //EXP2
  
  if ( byte i=3; ON[i] != ONp[i] ){ 
    ONp[i] = ON[i];
    CTM[i] = millis() + 500; 
    delay(10); 
  }


/////////////////////////////////////////////////////////////////////////////////////////////// DIP switch values
  
  for ( byte d=0, i=0; i <= 3; i++ ){
    DIPV[i] = digitalRead(Pin_D[i]);
    d = bitWrite(d, i, DIPV[i]);
    if (i==3) { DIPNUM = d; }
  }
  
  if ( DIPNUM != DIPNUMp ){
    BANK = DIPNUM;
    SerialUSB.print("DIP : ");
    SerialUSB.println(DIPNUM);
    DIPNUMp = DIPNUM;
  }


/////////////////////////////////////////////////////////////////////////////////////////////// EEP to BANK Param

  if ( BANK != BANKp ){
    for ( byte c=0; c <= 55; c++ ){
      //MODE
      if ( c == 0){ MODE[0] = EEPV[BANK][c]; }
      if ( c == 1){ MODE[1] = EEPV[BANK][c]; }
      if ( c == 2){ MODE[2] = EEPV[BANK][c]; }
      if ( c == 3){ MODE[3] = EEPV[BANK][c]; }
      if ( c == 4){ MODE[4] = EEPV[BANK][c]; }
      if ( c == 5){ MODE[5] = EEPV[BANK][c]; }
      //MIDICH
      if ( c == 6){ MIDICH[0] = EEPV[BANK][c]; }
      if ( c == 7){ MIDICH[1] = EEPV[BANK][c]; }
      if ( c == 8){ MIDICH[2] = EEPV[BANK][c]; }
      if ( c == 9){ MIDICH[3] = EEPV[BANK][c]; }
      if ( c == 10){ MIDICH[4] = EEPV[BANK][c]; }
      if ( c == 11){ MIDICH[5] = EEPV[BANK][c]; }
      //CCID
      if ( c == 12){ CCID[0] = EEPV[BANK][c]; }
      if ( c == 13){ CCID[1] = EEPV[BANK][c]; }
      if ( c == 14){ CCID[2] = EEPV[BANK][c]; }
      if ( c == 15){ CCID[3] = EEPV[BANK][c]; }
      if ( c == 16){ CCID[4] = EEPV[BANK][c]; }
      if ( c == 17){ CCID[5] = EEPV[BANK][c]; }
      //Curve
      if ( c == 18){ curve[0] = EEPV[BANK][c]; }
      if ( c == 19){ curve[1] = EEPV[BANK][c]; }
      //MINV
      if ( c == 20){ MINV[0] = EEPV[BANK][c]; }
      if ( c == 21){ MINV[1] = EEPV[BANK][c]; }
      if ( c == 22){ MINV[2] = EEPV[BANK][c]; }
      if ( c == 23){ MINV[3] = EEPV[BANK][c]; }
      if ( c == 24){ MINV[4] = EEPV[BANK][c]; }
      if ( c == 25){ MINV[5] = EEPV[BANK][c]; }
      //MAXV
      if ( c == 26){ MAXV[0] = EEPV[BANK][c]; }
      if ( c == 27){ MAXV[1] = EEPV[BANK][c]; }
      if ( c == 28){ MAXV[2] = EEPV[BANK][c]; }
      if ( c == 29){ MAXV[3] = EEPV[BANK][c]; }
      if ( c == 30){ MAXV[4] = EEPV[BANK][c]; }
      if ( c == 31){ MAXV[5] = EEPV[BANK][c]; }
      //MINR
      if ( c == 32){ MINR[0] = EEPV[BANK][c]; }
      if ( c == 33){ MINR[1] = EEPV[BANK][c]; }
      if ( c == 34){ MINR[2] = EEPV[BANK][c]; }
      if ( c == 35){ MINR[3] = EEPV[BANK][c]; }
      //MAXR
      if ( c == 36){ MAXR[0] = EEPV[BANK][c]; }
      if ( c == 37){ MAXR[1] = EEPV[BANK][c]; }
      if ( c == 38){ MAXR[2] = EEPV[BANK][c]; }
      if ( c == 39){ MAXR[3] = EEPV[BANK][c]; }
      //PGMV
      if ( c == 40){ PGMV[0] = EEPV[BANK][c]; }
      if ( c == 41){ PGMV[1] = EEPV[BANK][c]; }
      if ( c == 42){ PGMV[2] = EEPV[BANK][c]; }
      if ( c == 43){ PGMV[3] = EEPV[BANK][c]; }
      //NOTE
      if ( c == 44){ NOTE[0] = EEPV[BANK][c]; }
      if ( c == 45){ NOTE[1] = EEPV[BANK][c]; }
      if ( c == 46){ NOTE[2] = EEPV[BANK][c]; }
      if ( c == 47){ NOTE[3] = EEPV[BANK][c]; }
      //VELO
      if ( c == 48){ VELO[0] = EEPV[BANK][c]; }
      if ( c == 49){ VELO[1] = EEPV[BANK][c]; }
      if ( c == 50){ VELO[2] = EEPV[BANK][c]; }
      if ( c == 51){ VELO[3] = EEPV[BANK][c]; }
      //NCO
      if ( c == 52){ NCO[0] = EEPV[BANK][c]; }
      if ( c == 53){ NCO[1] = EEPV[BANK][c]; }
      if ( c == 54){ NCO[2] = EEPV[BANK][c]; }
      if ( c == 55){ NCO[3] = EEPV[BANK][c]; }
    }
    BANKp = BANK;
  }

  
/////////////////////////////////////////////////////////////////////////////////////////////// FS1 i=0

  if ( byte i=0; ON[0] == 1 ){
    VAL[i] = digitalRead(Pin_V[i]);  

    if ( millis() >= CTM[0] ){ 
      
      if ( VAL[i] != VALp[i] ){
        VTM[i] = millis() + 10;
        FS_State[i] = VAL[i];
        VALp[i] = VAL[i];
      }

      if ( millis() >= VTM[i] && FS_State[i] != FSp_State[i]){

  //Normal Close
        if ( NCO[i] == 0 ){

  //FS1 : Stomp NC
          if ( VAL[i] == 1 ){
          
            if ( MODE[i] == 0 ){
              if ( FootSwitch[i] == 0 ){
                FootSwitch[i] = 1;
                controlChange(MIDICH[i], CCID[i], MAXV[i]);
                MidiUSB.flush();
                D_controlChange(MIDICH[i], CCID[i], MAXV[i]);
                //SerialUSB.print("FS1 : ");
                //SerialUSB.println(FootSwitch[i]);
                FSp_State[i] = FS_State[i];
              } else if ( FootSwitch[i] == 1 ){
                FootSwitch[i] = 0;
                controlChange(MIDICH[i], CCID[i], MINV[i]);
                MidiUSB.flush();
                D_controlChange(MIDICH[i], CCID[i], MINV[i]);
                //SerialUSB.print("FS1 : ");
                //SerialUSB.println(FootSwitch[i]);
                FSp_State[i] = FS_State[i];
              }
            } 
            
            else if ( MODE[i] == 1 ){
              FootSwitch[i] = 1;
              controlChange(MIDICH[i], CCID[i], MAXV[i]);
              MidiUSB.flush();
              D_controlChange(MIDICH[i], CCID[i], MAXV[i]);
              //SerialUSB.print("FS1 : ");
              //SerialUSB.println(FootSwitch[i]);
              FSp_State[i] = FS_State[i];
            }
  
            else if ( MODE[i] == 2 ){
              FootSwitch[i] = 1;
              RAND[i] = random(MINR[i], MAXR[i]);
              controlChange(MIDICH[i], CCID[i], RAND[i]);
              MidiUSB.flush();
              D_controlChange(MIDICH[i], CCID[i], RAND[i]);
              //SerialUSB.print("FS1 : ");
              //SerialUSB.println(RAND[i]);
              FSp_State[i] = FS_State[i];
            }
  
            else if ( MODE[i] == 3 ){
              FootSwitch[i] = 1;
              programChange(MIDICH[i], PGMV[i]);
              MidiUSB.flush();
              D_programChange(MIDICH[i], PGMV[i]);
              //SerialUSB.print("FS1 : ");
              //SerialUSB.println(PGMV[i]);
              FSp_State[i] = FS_State[i];
            }
  
            else if ( MODE[i] == 4 ){
              FootSwitch[i] = 1;
              noteOn(MIDICH[i], NOTE[i], VELO[i]);
              MidiUSB.flush();
              D_noteOn(MIDICH[i], NOTE[i], VELO[i]);
              //SerialUSB.print("FS1 : ");
              //SerialUSB.println(FootSwitch[i]);
              FSp_State[i] = FS_State[i];
            }
  
            else if ( MODE[i] == 5 ){
              if ( BANK != 9 ){
                FootSwitch[i] = 1;
                BANK = ++BANK;
                SerialUSB.print("FS1 : ");
                SerialUSB.println(BANK);
                FSp_State[i] = FS_State[i];
              } else if ( BANK == 9 ){
                FootSwitch[i] = 1;
                BANK = 0;
                SerialUSB.print("FS1 : ");
                SerialUSB.println(BANK);
                FSp_State[i] = FS_State[i];
              }
            }
  
            else if ( MODE[i] == 6 ){
              if ( BANK != 0 ){
                FootSwitch[i] = 1;
                BANK = --BANK;
                SerialUSB.print("FS1 : ");
                SerialUSB.println(BANK);
                FSp_State[i] = FS_State[i];
              } else if ( BANK == 0 ){
                FootSwitch[i] = 1;
                BANK = 9;
                SerialUSB.print("FS1 : ");
                SerialUSB.println(BANK);
                FSp_State[i] = FS_State[i];
              }
            }
  
    //FS1 : Release NC
          } else if ( VAL[i] == 0 ){
            
            if ( MODE[i] == 0 ){
              FSp_State[i] = FS_State[i];
            } 
            
            else if ( MODE[i] == 1 ){
              FootSwitch[i] = 0;
              controlChange(MIDICH[i], CCID[i], MINV[i]);
              MidiUSB.flush();
              D_controlChange(MIDICH[i], CCID[i], MINV[i]);
              //SerialUSB.print("FS1 : ");
              //SerialUSB.println(FootSwitch[i]);
              FSp_State[i] = FS_State[i];
            }
            
            else if ( MODE[i] == 2 ){
              FootSwitch[i] = 0;
              FSp_State[i] = FS_State[i];
            }
  
            else if ( MODE[i] == 3 ){
              FootSwitch[i] = 0;
              FSp_State[i] = FS_State[i];
            }
  
             else if ( MODE[i] == 4 ){
              FootSwitch[i] = 0;
              noteOff(MIDICH[i], NOTE[i], VELO[i]);
              MidiUSB.flush();
              D_noteOff(MIDICH[i], NOTE[i], VELO[i]);
              //SerialUSB.print("FS1 : ");
              //SerialUSB.println(FootSwitch[i]);
              FSp_State[i] = FS_State[i];
            }
            
            else if ( MODE[i] == 5 ){
              FootSwitch[i] = 0;
              FSp_State[i] = FS_State[i];
            }
            
            else if ( MODE[i] == 6 ){
              FootSwitch[i] = 0;
              FSp_State[i] = FS_State[i];
            }
          }

  //NORMAL OPEN        
        } else if ( NCO[i] == 1 ){
          
  //FS1 : Stomp NO
          if ( VAL[i] == 0 ){
          
            if ( MODE[i] == 0 ){
              if ( FootSwitch[i] == 0 ){
                FootSwitch[i] = 1;
                controlChange(MIDICH[i], CCID[i], MAXV[i]);
                MidiUSB.flush();
                D_controlChange(MIDICH[i], CCID[i], MAXV[i]);
                //SerialUSB.print("FS1 : ");
                //SerialUSB.println(FootSwitch[i]);
                FSp_State[i] = FS_State[i];
              } else if ( FootSwitch[i] == 1 ){
                FootSwitch[i] = 0;
                controlChange(MIDICH[i], CCID[i], MINV[i]);
                MidiUSB.flush();
                D_controlChange(MIDICH[i], CCID[i], MINV[i]);
                //SerialUSB.print("FS1 : ");
                //SerialUSB.println(FootSwitch[i]);
                FSp_State[i] = FS_State[i];
              }
            } 
            
            else if ( MODE[i] == 1 ){
              FootSwitch[i] = 1;
              controlChange(MIDICH[i], CCID[i], MAXV[i]);
              MidiUSB.flush();
              D_controlChange(MIDICH[i], CCID[i], MAXV[i]);
              //SerialUSB.print("FS1 : ");
              //SerialUSB.println(FootSwitch[i]);
              FSp_State[i] = FS_State[i];
            }
  
            else if ( MODE[i] == 2 ){
              FootSwitch[i] = 1;
              RAND[i] = random(MINR[i], MAXR[i]);
              controlChange(MIDICH[i], CCID[i], RAND[i]);
              MidiUSB.flush();
              D_controlChange(MIDICH[i], CCID[i], RAND[i]);
              //SerialUSB.print("FS1 : ");
              //SerialUSB.println(RAND[i]);
              FSp_State[i] = FS_State[i];
            }
  
            else if ( MODE[i] == 3 ){
              FootSwitch[i] = 1;
              programChange(MIDICH[i], PGMV[i]);
              MidiUSB.flush();
              D_programChange(MIDICH[i], PGMV[i]);
              //SerialUSB.print("FS1 : ");
              //SerialUSB.println(PGMV[i]);
              FSp_State[i] = FS_State[i];
            }
  
            else if ( MODE[i] == 4 ){
              FootSwitch[i] = 1;
              noteOn(MIDICH[i], NOTE[i], VELO[i]);
              MidiUSB.flush();
              D_noteOn(MIDICH[i], NOTE[i], VELO[i]);
              //SerialUSB.print("FS1 : ");
              //SerialUSB.println(FootSwitch[i]);
              FSp_State[i] = FS_State[i];
            }
  
            else if ( MODE[i] == 5 ){
              if ( BANK != 9 ){
                FootSwitch[i] = 1;
                BANK = ++BANK;
                SerialUSB.print("FS1 : ");
                SerialUSB.println(BANK);
                FSp_State[i] = FS_State[i];
              } else if ( BANK == 9 ){
                FootSwitch[i] = 1;
                BANK = 0;
                SerialUSB.print("FS1 : ");
                SerialUSB.println(BANK);
                FSp_State[i] = FS_State[i];
              }
            }
  
            else if ( MODE[i] == 6 ){
              if ( BANK != 0 ){
                FootSwitch[i] = 1;
                BANK = --BANK;
                SerialUSB.print("FS1 : ");
                SerialUSB.println(BANK);
                FSp_State[i] = FS_State[i];
              } else if ( BANK == 0 ){
                FootSwitch[i] = 1;
                BANK = 9;
                SerialUSB.print("FS1 : ");
                SerialUSB.println(BANK);
                FSp_State[i] = FS_State[i];
              }
            }
  
    //FS1 : Release NO
          } else if ( VAL[i] == 1 ){
            
            if ( MODE[i] == 0 ){
              FSp_State[i] = FS_State[i];
            } 
            
            else if ( MODE[i] == 1 ){
              FootSwitch[i] = 0;
              controlChange(MIDICH[i], CCID[i], MINV[i]);
              MidiUSB.flush();
              D_controlChange(MIDICH[i], CCID[i], MINV[i]);
              //SerialUSB.print("FS1 : ");
              //SerialUSB.println(FootSwitch[i]);
              FSp_State[i] = FS_State[i];
            }
            
            else if ( MODE[i] == 2 ){
              FootSwitch[i] = 0;
              FSp_State[i] = FS_State[i];
            }
  
            else if ( MODE[i] == 3 ){
              FootSwitch[i] = 0;
              FSp_State[i] = FS_State[i];
            }
  
             else if ( MODE[i] == 4 ){
              FootSwitch[i] = 0;
              noteOff(MIDICH[i], NOTE[i], VELO[i]);
              MidiUSB.flush();
              D_noteOff(MIDICH[i], NOTE[i], VELO[i]);
              //SerialUSB.print("FS1 : ");
              //SerialUSB.println(FootSwitch[i]);
              FSp_State[i] = FS_State[i];
            }
            
            else if ( MODE[i] == 5 ){
              FootSwitch[i] = 0;
              FSp_State[i] = FS_State[i];
            }
            
            else if ( MODE[i] == 6 ){
              FootSwitch[i] = 0;
              FSp_State[i] = FS_State[i];
            }
          }
        }
      }        
    }
  }
  

/////////////////////////////////////////////////////////////////////////////////////////////// FS2 i=1 ON $ CTM=0

  if ( byte i=1; ON[0] == 1 ){
    VAL[i] = digitalRead(Pin_V[i]);  

    if ( millis() >= CTM[0] ){ 
      
      if ( VAL[i] != VALp[i] ){
        VTM[i] = millis() + 10;
        FS_State[i] = VAL[i];
        VALp[i] = VAL[i];
      }

      if ( millis() >= VTM[i] && FS_State[i] != FSp_State[i]){

  //Normal Close
        if ( NCO[i] == 0 ){

  //FS2 : Stomp NC
          if ( VAL[i] == 1 ){
          
            if ( MODE[i] == 0 ){
              if ( FootSwitch[i] == 0 ){
                FootSwitch[i] = 1;
                controlChange(MIDICH[i], CCID[i], MAXV[i]);
                MidiUSB.flush();
                D_controlChange(MIDICH[i], CCID[i], MAXV[i]);
                //SerialUSB.print("FS2 : ");
                //SerialUSB.println(FootSwitch[i]);
                FSp_State[i] = FS_State[i];
              } else if ( FootSwitch[i] == 1 ){
                FootSwitch[i] = 0;
                controlChange(MIDICH[i], CCID[i], MINV[i]);
                MidiUSB.flush();
                D_controlChange(MIDICH[i], CCID[i], MINV[i]);
                //SerialUSB.print("FS2 : ");
                //SerialUSB.println(FootSwitch[i]);
                FSp_State[i] = FS_State[i];
              }
            } 
            
            else if ( MODE[i] == 1 ){
              FootSwitch[i] = 1;
              controlChange(MIDICH[i], CCID[i], MAXV[i]);
              MidiUSB.flush();
              D_controlChange(MIDICH[i], CCID[i], MAXV[i]);
              //SerialUSB.print("FS2 : ");
              //SerialUSB.println(FootSwitch[i]);
              FSp_State[i] = FS_State[i];
            }
  
            else if ( MODE[i] == 2 ){
              FootSwitch[i] = 1;
              RAND[i] = random(MINR[i], MAXR[i]);
              controlChange(MIDICH[i], CCID[i], RAND[i]);
              MidiUSB.flush();
              D_controlChange(MIDICH[i], CCID[i], RAND[i]);
              //SerialUSB.print("FS2 : ");
              //SerialUSB.println(RAND[i]);
              FSp_State[i] = FS_State[i];
            }
  
            else if ( MODE[i] == 3 ){
              FootSwitch[i] = 1;
              programChange(MIDICH[i], PGMV[i]);
              MidiUSB.flush();
              D_programChange(MIDICH[i], PGMV[i]);
              //SerialUSB.print("FS2 : ");
              //SerialUSB.println(PGMV[i]);
              FSp_State[i] = FS_State[i];
            }
  
            else if ( MODE[i] == 4 ){
              FootSwitch[i] = 1;
              noteOn(MIDICH[i], NOTE[i], VELO[i]);
              MidiUSB.flush();
              D_noteOn(MIDICH[i], NOTE[i], VELO[i]);
              //SerialUSB.print("FS2 : ");
              //SerialUSB.println(FootSwitch[i]);
              FSp_State[i] = FS_State[i];
            }
  
            else if ( MODE[i] == 5 ){
              if ( BANK != 9 ){
                FootSwitch[i] = 1;
                BANK = ++BANK;
                SerialUSB.print("FS2 : ");
                SerialUSB.println(BANK);
                FSp_State[i] = FS_State[i];
              } else if ( BANK == 9 ){
                FootSwitch[i] = 1;
                BANK = 0;
                SerialUSB.print("FS2 : ");
                SerialUSB.println(BANK);
                FSp_State[i] = FS_State[i];
              }
            }
  
            else if ( MODE[i] == 6 ){
              if ( BANK != 0 ){
                FootSwitch[i] = 1;
                BANK = --BANK;
                SerialUSB.print("FS2 : ");
                SerialUSB.println(BANK);
                FSp_State[i] = FS_State[i];
              } else if ( BANK == 0 ){
                FootSwitch[i] = 1;
                BANK = 9;
                SerialUSB.print("FS2 : ");
                SerialUSB.println(BANK);
                FSp_State[i] = FS_State[i];
              }
            }
  
    //FS2 : Release NC
          } else if ( VAL[i] == 0 ){
            
            if ( MODE[i] == 0 ){
              FSp_State[i] = FS_State[i];
            } 
            
            else if ( MODE[i] == 1 ){
              FootSwitch[i] = 0;
              controlChange(MIDICH[i], CCID[i], MINV[i]);
              MidiUSB.flush();
              D_controlChange(MIDICH[i], CCID[i], MINV[i]);
              //SerialUSB.print("FS2 : ");
              //SerialUSB.println(FootSwitch[i]);
              FSp_State[i] = FS_State[i];
            }
            
            else if ( MODE[i] == 2 ){
              FootSwitch[i] = 0;
              FSp_State[i] = FS_State[i];
            }
  
            else if ( MODE[i] == 3 ){
              FootSwitch[i] = 0;
              FSp_State[i] = FS_State[i];
            }
  
             else if ( MODE[i] == 4 ){
              FootSwitch[i] = 0;
              noteOff(MIDICH[i], NOTE[i], VELO[i]);
              MidiUSB.flush();
              D_noteOff(MIDICH[i], NOTE[i], VELO[i]);
              //SerialUSB.print("FS2 : ");
              //SerialUSB.println(FootSwitch[i]);
              FSp_State[i] = FS_State[i];
            }
            
            else if ( MODE[i] == 5 ){
              FootSwitch[i] = 0;
              FSp_State[i] = FS_State[i];
            }
            
            else if ( MODE[i] == 6 ){
              FootSwitch[i] = 0;
              FSp_State[i] = FS_State[i];
            }
          }

  //NORMAL OPEN        
        } else if ( NCO[i] == 1 ){
          
  //FS2 : Stomp NO
          if ( VAL[i] == 0 ){
          
            if ( MODE[i] == 0 ){
              if ( FootSwitch[i] == 0 ){
                FootSwitch[i] = 1;
                controlChange(MIDICH[i], CCID[i], MAXV[i]);
                MidiUSB.flush();
                D_controlChange(MIDICH[i], CCID[i], MAXV[i]);
                //SerialUSB.print("FS2 : ");
                //SerialUSB.println(FootSwitch[i]);
                FSp_State[i] = FS_State[i];
              } else if ( FootSwitch[i] == 1 ){
                FootSwitch[i] = 0;
                controlChange(MIDICH[i], CCID[i], MINV[i]);
                MidiUSB.flush();
                D_controlChange(MIDICH[i], CCID[i], MINV[i]);
                //SerialUSB.print("FS2 : ");
                //SerialUSB.println(FootSwitch[i]);
                FSp_State[i] = FS_State[i];
              }
            } 
            
            else if ( MODE[i] == 1 ){
              FootSwitch[i] = 1;
              controlChange(MIDICH[i], CCID[i], MAXV[i]);
              MidiUSB.flush();
              D_controlChange(MIDICH[i], CCID[i], MAXV[i]);
              //SerialUSB.print("FS2 : ");
              //SerialUSB.println(FootSwitch[i]);
              FSp_State[i] = FS_State[i];
            }
  
            else if ( MODE[i] == 2 ){
              FootSwitch[i] = 1;
              RAND[i] = random(MINR[i], MAXR[i]);
              controlChange(MIDICH[i], CCID[i], RAND[i]);
              MidiUSB.flush();
              D_controlChange(MIDICH[i], CCID[i], RAND[i]);
              //SerialUSB.print("FS2 : ");
              //SerialUSB.println(RAND[i]);
              FSp_State[i] = FS_State[i];
            }
  
            else if ( MODE[i] == 3 ){
              FootSwitch[i] = 1;
              programChange(MIDICH[i], PGMV[i]);
              MidiUSB.flush();
              D_programChange(MIDICH[i], PGMV[i]);
              //SerialUSB.print("FS2 : ");
              //SerialUSB.println(PGMV[i]);
              FSp_State[i] = FS_State[i];
            }
  
            else if ( MODE[i] == 4 ){
              FootSwitch[i] = 1;
              noteOn(MIDICH[i], NOTE[i], VELO[i]);
              MidiUSB.flush();
              D_noteOn(MIDICH[i], NOTE[i], VELO[i]);
              //SerialUSB.print("FS2 : ");
              //SerialUSB.println(FootSwitch[i]);
              FSp_State[i] = FS_State[i];
            }
  
            else if ( MODE[i] == 5 ){
              if ( BANK != 9 ){
                FootSwitch[i] = 1;
                BANK = ++BANK;
                SerialUSB.print("FS2 : ");
                SerialUSB.println(BANK);
                FSp_State[i] = FS_State[i];
              } else if ( BANK == 9 ){
                FootSwitch[i] = 1;
                BANK = 0;
                SerialUSB.print("FS2 : ");
                SerialUSB.println(BANK);
                FSp_State[i] = FS_State[i];
              }
            }
  
            else if ( MODE[i] == 6 ){
              if ( BANK != 0 ){
                FootSwitch[i] = 1;
                BANK = --BANK;
                SerialUSB.print("FS2 : ");
                SerialUSB.println(BANK);
                FSp_State[i] = FS_State[i];
              } else if ( BANK == 0 ){
                FootSwitch[i] = 1;
                BANK = 9;
                SerialUSB.print("FS2 : ");
                SerialUSB.println(BANK);
                FSp_State[i] = FS_State[i];
              }
            }
  
    //FS2 : Release NO
          } else if ( VAL[i] == 1 ){
            
            if ( MODE[i] == 0 ){
              FSp_State[i] = FS_State[i];
            } 
            
            else if ( MODE[i] == 1 ){
              FootSwitch[i] = 0;
              controlChange(MIDICH[i], CCID[i], MINV[i]);
              MidiUSB.flush();
              D_controlChange(MIDICH[i], CCID[i], MINV[i]);
              //SerialUSB.print("FS2 : ");
              //SerialUSB.println(FootSwitch[i]);
              FSp_State[i] = FS_State[i];
            }
            
            else if ( MODE[i] == 2 ){
              FootSwitch[i] = 0;
              FSp_State[i] = FS_State[i];
            }
  
            else if ( MODE[i] == 3 ){
              FootSwitch[i] = 0;
              FSp_State[i] = FS_State[i];
            }
  
             else if ( MODE[i] == 4 ){
              FootSwitch[i] = 0;
              noteOff(MIDICH[i], NOTE[i], VELO[i]);
              MidiUSB.flush();
              D_noteOff(MIDICH[i], NOTE[i], VELO[i]);
              //SerialUSB.print("FS2 : ");
              //SerialUSB.println(FootSwitch[i]);
              FSp_State[i] = FS_State[i];
            }
            
            else if ( MODE[i] == 5 ){
              FootSwitch[i] = 0;
              FSp_State[i] = FS_State[i];
            }
            
            else if ( MODE[i] == 6 ){
              FootSwitch[i] = 0;
              FSp_State[i] = FS_State[i];
            }
          }
        }
      }        
    }
  }
  

/////////////////////////////////////////////////////////////////////////////////////////////// FS3 i=2 ON $ CTM=1

  if ( byte i=2; ON[1] == 1 ){
    VAL[i] = digitalRead(Pin_V[i]);  

    if ( millis() >= CTM[1] ){ 
      
      if ( VAL[i] != VALp[i] ){
        VTM[i] = millis() + 10;
        FS_State[i] = VAL[i];
        VALp[i] = VAL[i];
      }

      if ( millis() >= VTM[i] && FS_State[i] != FSp_State[i]){

  //Normal Close
        if ( NCO[i] == 0 ){

  //FS3 : Stomp NC
          if ( VAL[i] == 1 ){
          
            if ( MODE[i] == 0 ){
              if ( FootSwitch[i] == 0 ){
                FootSwitch[i] = 1;
                controlChange(MIDICH[i], CCID[i], MAXV[i]);
                MidiUSB.flush();
                D_controlChange(MIDICH[i], CCID[i], MAXV[i]);
                //SerialUSB.print("FS3 : ");
                //SerialUSB.println(FootSwitch[i]);
                FSp_State[i] = FS_State[i];
              } else if ( FootSwitch[i] == 1 ){
                FootSwitch[i] = 0;
                controlChange(MIDICH[i], CCID[i], MINV[i]);
                MidiUSB.flush();
                D_controlChange(MIDICH[i], CCID[i], MINV[i]);
                //SerialUSB.print("FS3 : ");
                //SerialUSB.println(FootSwitch[i]);
                FSp_State[i] = FS_State[i];
              }
            } 
            
            else if ( MODE[i] == 1 ){
              FootSwitch[i] = 1;
              controlChange(MIDICH[i], CCID[i], MAXV[i]);
              MidiUSB.flush();
              D_controlChange(MIDICH[i], CCID[i], MAXV[i]);
              //SerialUSB.print("FS3 : ");
              //SerialUSB.println(FootSwitch[i]);
              FSp_State[i] = FS_State[i];
            }
  
            else if ( MODE[i] == 2 ){
              FootSwitch[i] = 1;
              RAND[i] = random(MINR[i], MAXR[i]);
              controlChange(MIDICH[i], CCID[i], RAND[i]);
              MidiUSB.flush();
              D_controlChange(MIDICH[i], CCID[i], RAND[i]);
              //SerialUSB.print("FS3 : ");
              //SerialUSB.println(RAND[i]);
              FSp_State[i] = FS_State[i];
            }
  
            else if ( MODE[i] == 3 ){
              FootSwitch[i] = 1;
              programChange(MIDICH[i], PGMV[i]);
              MidiUSB.flush();
              D_programChange(MIDICH[i], PGMV[i]);
              //SerialUSB.print("FS3 : ");
              //SerialUSB.println(PGMV[i]);
              FSp_State[i] = FS_State[i];
            }
  
            else if ( MODE[i] == 4 ){
              FootSwitch[i] = 1;
              noteOn(MIDICH[i], NOTE[i], VELO[i]);
              MidiUSB.flush();
              D_noteOn(MIDICH[i], NOTE[i], VELO[i]);
              //SerialUSB.print("FS3 : ");
              //SerialUSB.println(FootSwitch[i]);
              FSp_State[i] = FS_State[i];
            }
  
            else if ( MODE[i] == 5 ){
              if ( BANK != 9 ){
                FootSwitch[i] = 1;
                BANK = ++BANK;
                SerialUSB.print("FS3 : ");
                SerialUSB.println(BANK);
                FSp_State[i] = FS_State[i];
              } else if ( BANK == 9 ){
                FootSwitch[i] = 1;
                BANK = 0;
                SerialUSB.print("FS3 : ");
                SerialUSB.println(BANK);
                FSp_State[i] = FS_State[i];
              }
            }
  
            else if ( MODE[i] == 6 ){
              if ( BANK != 0 ){
                FootSwitch[i] = 1;
                BANK = --BANK;
                SerialUSB.print("FS3 : ");
                SerialUSB.println(BANK);
                FSp_State[i] = FS_State[i];
              } else if ( BANK == 0 ){
                FootSwitch[i] = 1;
                BANK = 9;
                SerialUSB.print("FS3 : ");
                SerialUSB.println(BANK);
                FSp_State[i] = FS_State[i];
              }
            }
  
    //FS3 : Release NC
          } else if ( VAL[i] == 0 ){
            
            if ( MODE[i] == 0 ){
              FSp_State[i] = FS_State[i];
            } 
            
            else if ( MODE[i] == 1 ){
              FootSwitch[i] = 0;
              controlChange(MIDICH[i], CCID[i], MINV[i]);
              MidiUSB.flush();
              D_controlChange(MIDICH[i], CCID[i], MINV[i]);
              //SerialUSB.print("FS3 : ");
              //SerialUSB.println(FootSwitch[i]);
              FSp_State[i] = FS_State[i];
            }
            
            else if ( MODE[i] == 2 ){
              FootSwitch[i] = 0;
              FSp_State[i] = FS_State[i];
            }
  
            else if ( MODE[i] == 3 ){
              FootSwitch[i] = 0;
              FSp_State[i] = FS_State[i];
            }
  
             else if ( MODE[i] == 4 ){
              FootSwitch[i] = 0;
              noteOff(MIDICH[i], NOTE[i], VELO[i]);
              MidiUSB.flush();
              D_noteOff(MIDICH[i], NOTE[i], VELO[i]);
              //SerialUSB.print("FS3 : ");
              //SerialUSB.println(FootSwitch[i]);
              FSp_State[i] = FS_State[i];
            }
            
            else if ( MODE[i] == 5 ){
              FootSwitch[i] = 0;
              FSp_State[i] = FS_State[i];
            }
            
            else if ( MODE[i] == 6 ){
              FootSwitch[i] = 0;
              FSp_State[i] = FS_State[i];
            }
          }

  //NORMAL OPEN        
        } else if ( NCO[i] == 1 ){
          
  //FS3 : Stomp NO
          if ( VAL[i] == 0 ){
          
            if ( MODE[i] == 0 ){
              if ( FootSwitch[i] == 0 ){
                FootSwitch[i] = 1;
                controlChange(MIDICH[i], CCID[i], MAXV[i]);
                MidiUSB.flush();
                D_controlChange(MIDICH[i], CCID[i], MAXV[i]);
                //SerialUSB.print("FS3 : ");
                //SerialUSB.println(FootSwitch[i]);
                FSp_State[i] = FS_State[i];
              } else if ( FootSwitch[i] == 1 ){
                FootSwitch[i] = 0;
                controlChange(MIDICH[i], CCID[i], MINV[i]);
                MidiUSB.flush();
                D_controlChange(MIDICH[i], CCID[i], MINV[i]);
                //SerialUSB.print("FS3 : ");
                //SerialUSB.println(FootSwitch[i]);
                FSp_State[i] = FS_State[i];
              }
            } 
            
            else if ( MODE[i] == 1 ){
              FootSwitch[i] = 1;
              controlChange(MIDICH[i], CCID[i], MAXV[i]);
              MidiUSB.flush();
              D_controlChange(MIDICH[i], CCID[i], MAXV[i]);
              //SerialUSB.print("FS3 : ");
              //SerialUSB.println(FootSwitch[i]);
              FSp_State[i] = FS_State[i];
            }
  
            else if ( MODE[i] == 2 ){
              FootSwitch[i] = 1;
              RAND[i] = random(MINR[i], MAXR[i]);
              controlChange(MIDICH[i], CCID[i], RAND[i]);
              MidiUSB.flush();
              D_controlChange(MIDICH[i], CCID[i], RAND[i]);
              //SerialUSB.print("FS3 : ");
              //SerialUSB.println(RAND[i]);
              FSp_State[i] = FS_State[i];
            }
  
            else if ( MODE[i] == 3 ){
              FootSwitch[i] = 1;
              programChange(MIDICH[i], PGMV[i]);
              MidiUSB.flush();
              D_programChange(MIDICH[i], PGMV[i]);
              //SerialUSB.print("FS3 : ");
              //SerialUSB.println(PGMV[i]);
              FSp_State[i] = FS_State[i];
            }
  
            else if ( MODE[i] == 4 ){
              FootSwitch[i] = 1;
              noteOn(MIDICH[i], NOTE[i], VELO[i]);
              MidiUSB.flush();
              D_noteOn(MIDICH[i], NOTE[i], VELO[i]);
              //SerialUSB.print("FS3 : ");
              //SerialUSB.println(FootSwitch[i]);
              FSp_State[i] = FS_State[i];
            }
  
            else if ( MODE[i] == 5 ){
              if ( BANK != 9 ){
                FootSwitch[i] = 1;
                BANK = ++BANK;
                SerialUSB.print("FS3 : ");
                SerialUSB.println(BANK);
                FSp_State[i] = FS_State[i];
              } else if ( BANK == 9 ){
                FootSwitch[i] = 1;
                BANK = 0;
                SerialUSB.print("FS3 : ");
                SerialUSB.println(BANK);
                FSp_State[i] = FS_State[i];
              }
            }
  
            else if ( MODE[i] == 6 ){
              if ( BANK != 0 ){
                FootSwitch[i] = 1;
                BANK = --BANK;
                SerialUSB.print("FS3 : ");
                SerialUSB.println(BANK);
                FSp_State[i] = FS_State[i];
              } else if ( BANK == 0 ){
                FootSwitch[i] = 1;
                BANK = 9;
                SerialUSB.print("FS3 : ");
                SerialUSB.println(BANK);
                FSp_State[i] = FS_State[i];
              }
            }
  
    //FS3 : Release NO
          } else if ( VAL[i] == 1 ){
            
            if ( MODE[i] == 0 ){
              FSp_State[i] = FS_State[i];
            } 
            
            else if ( MODE[i] == 1 ){
              FootSwitch[i] = 0;
              controlChange(MIDICH[i], CCID[i], MINV[i]);
              MidiUSB.flush();
              D_controlChange(MIDICH[i], CCID[i], MINV[i]);
              //SerialUSB.print("FS3 : ");
              //SerialUSB.println(FootSwitch[i]);
              FSp_State[i] = FS_State[i];
            }
            
            else if ( MODE[i] == 2 ){
              FootSwitch[i] = 0;
              FSp_State[i] = FS_State[i];
            }
  
            else if ( MODE[i] == 3 ){
              FootSwitch[i] = 0;
              FSp_State[i] = FS_State[i];
            }
  
             else if ( MODE[i] == 4 ){
              FootSwitch[i] = 0;
              noteOff(MIDICH[i], NOTE[i], VELO[i]);
              MidiUSB.flush();
              D_noteOff(MIDICH[i], NOTE[i], VELO[i]);
              //SerialUSB.print("FS3 : ");
              //SerialUSB.println(FootSwitch[i]);
              FSp_State[i] = FS_State[i];
            }
            
            else if ( MODE[i] == 5 ){
              FootSwitch[i] = 0;
              FSp_State[i] = FS_State[i];
            }
            
            else if ( MODE[i] == 6 ){
              FootSwitch[i] = 0;
              FSp_State[i] = FS_State[i];
            }
          }
        }
      }        
    }
  }
  

/////////////////////////////////////////////////////////////////////////////////////////////// FS4 i=3 ON $ CTM=1

  if ( byte i=3; ON[1] == 1 ){
    VAL[i] = digitalRead(Pin_V[i]);  

    if ( millis() >= CTM[1] ){ 
      
      if ( VAL[i] != VALp[i] ){
        VTM[i] = millis() + 10;
        FS_State[i] = VAL[i];
        VALp[i] = VAL[i];
      }

      if ( millis() >= VTM[i] && FS_State[i] != FSp_State[i]){

  //Normal Close
        if ( NCO[i] == 0 ){

  //FS4 : Stomp NC
          if ( VAL[i] == 1 ){
          
            if ( MODE[i] == 0 ){
              if ( FootSwitch[i] == 0 ){
                FootSwitch[i] = 1;
                controlChange(MIDICH[i], CCID[i], MAXV[i]);
                MidiUSB.flush();
                D_controlChange(MIDICH[i], CCID[i], MAXV[i]);
                //SerialUSB.print("FS4 : ");
                //SerialUSB.println(FootSwitch[i]);
                FSp_State[i] = FS_State[i];
              } else if ( FootSwitch[i] == 1 ){
                FootSwitch[i] = 0;
                controlChange(MIDICH[i], CCID[i], MINV[i]);
                MidiUSB.flush();
                D_controlChange(MIDICH[i], CCID[i], MINV[i]);
                //SerialUSB.print("FS4 : ");
                //SerialUSB.println(FootSwitch[i]);
                FSp_State[i] = FS_State[i];
              }
            } 
            
            else if ( MODE[i] == 1 ){
              FootSwitch[i] = 1;
              controlChange(MIDICH[i], CCID[i], MAXV[i]);
              MidiUSB.flush();
              D_controlChange(MIDICH[i], CCID[i], MAXV[i]);
              //SerialUSB.print("FS4 : ");
              //SerialUSB.println(FootSwitch[i]);
              FSp_State[i] = FS_State[i];
            }
  
            else if ( MODE[i] == 2 ){
              FootSwitch[i] = 1;
              RAND[i] = random(MINR[i], MAXR[i]);
              controlChange(MIDICH[i], CCID[i], RAND[i]);
              MidiUSB.flush();
              D_controlChange(MIDICH[i], CCID[i], RAND[i]);
              //SerialUSB.print("FS4 : ");
              //SerialUSB.println(RAND[i]);
              FSp_State[i] = FS_State[i];
            }
  
            else if ( MODE[i] == 3 ){
              FootSwitch[i] = 1;
              programChange(MIDICH[i], PGMV[i]);
              MidiUSB.flush();
              D_programChange(MIDICH[i], PGMV[i]);
              //SerialUSB.print("FS4 : ");
              //SerialUSB.println(PGMV[i]);
              FSp_State[i] = FS_State[i];
            }
  
            else if ( MODE[i] == 4 ){
              FootSwitch[i] = 1;
              noteOn(MIDICH[i], NOTE[i], VELO[i]);
              MidiUSB.flush();
              D_noteOn(MIDICH[i], NOTE[i], VELO[i]);
              //SerialUSB.print("FS4 : ");
              //SerialUSB.println(FootSwitch[i]);
              FSp_State[i] = FS_State[i];
            }
  
            else if ( MODE[i] == 5 ){
              if ( BANK != 9 ){
                FootSwitch[i] = 1;
                BANK = ++BANK;
                SerialUSB.print("FS4 : ");
                SerialUSB.println(BANK);
                FSp_State[i] = FS_State[i];
              } else if ( BANK == 9 ){
                FootSwitch[i] = 1;
                BANK = 0;
                SerialUSB.print("FS4 : ");
                SerialUSB.println(BANK);
                FSp_State[i] = FS_State[i];
              }
            }
  
            else if ( MODE[i] == 6 ){
              if ( BANK != 0 ){
                FootSwitch[i] = 1;
                BANK = --BANK;
                SerialUSB.print("FS4 : ");
                SerialUSB.println(BANK);
                FSp_State[i] = FS_State[i];
              } else if ( BANK == 0 ){
                FootSwitch[i] = 1;
                BANK = 9;
                SerialUSB.print("FS4 : ");
                SerialUSB.println(BANK);
                FSp_State[i] = FS_State[i];
              }
            }
  
    //FS4 : Release NC
          } else if ( VAL[i] == 0 ){
            
            if ( MODE[i] == 0 ){
              FSp_State[i] = FS_State[i];
            } 
            
            else if ( MODE[i] == 1 ){
              FootSwitch[i] = 0;
              controlChange(MIDICH[i], CCID[i], MINV[i]);
              MidiUSB.flush();
              D_controlChange(MIDICH[i], CCID[i], MINV[i]);
              //SerialUSB.print("FS4 : ");
              //SerialUSB.println(FootSwitch[i]);
              FSp_State[i] = FS_State[i];
            }
            
            else if ( MODE[i] == 2 ){
              FootSwitch[i] = 0;
              FSp_State[i] = FS_State[i];
            }
  
            else if ( MODE[i] == 3 ){
              FootSwitch[i] = 0;
              FSp_State[i] = FS_State[i];
            }
  
             else if ( MODE[i] == 4 ){
              FootSwitch[i] = 0;
              noteOff(MIDICH[i], NOTE[i], VELO[i]);
              MidiUSB.flush();
              D_noteOff(MIDICH[i], NOTE[i], VELO[i]);
              //SerialUSB.print("FS4 : ");
              //SerialUSB.println(FootSwitch[i]);
              FSp_State[i] = FS_State[i];
            }
            
            else if ( MODE[i] == 5 ){
              FootSwitch[i] = 0;
              FSp_State[i] = FS_State[i];
            }
            
            else if ( MODE[i] == 6 ){
              FootSwitch[i] = 0;
              FSp_State[i] = FS_State[i];
            }
          }

  //NORMAL OPEN        
        } else if ( NCO[i] == 1 ){
          
  //FS4 : Stomp NO
          if ( VAL[i] == 0 ){
          
            if ( MODE[i] == 0 ){
              if ( FootSwitch[i] == 0 ){
                FootSwitch[i] = 1;
                controlChange(MIDICH[i], CCID[i], MAXV[i]);
                MidiUSB.flush();
                D_controlChange(MIDICH[i], CCID[i], MAXV[i]);
                //SerialUSB.print("FS4 : ");
                //SerialUSB.println(FootSwitch[i]);
                FSp_State[i] = FS_State[i];
              } else if ( FootSwitch[i] == 1 ){
                FootSwitch[i] = 0;
                controlChange(MIDICH[i], CCID[i], MINV[i]);
                MidiUSB.flush();
                D_controlChange(MIDICH[i], CCID[i], MINV[i]);
                //SerialUSB.print("FS4 : ");
                //SerialUSB.println(FootSwitch[i]);
                FSp_State[i] = FS_State[i];
              }
            } 
            
            else if ( MODE[i] == 1 ){
              FootSwitch[i] = 1;
              controlChange(MIDICH[i], CCID[i], MAXV[i]);
              MidiUSB.flush();
              D_controlChange(MIDICH[i], CCID[i], MAXV[i]);
              //SerialUSB.print("FS4 : ");
              //SerialUSB.println(FootSwitch[i]);
              FSp_State[i] = FS_State[i];
            }
  
            else if ( MODE[i] == 2 ){
              FootSwitch[i] = 1;
              RAND[i] = random(MINR[i], MAXR[i]);
              controlChange(MIDICH[i], CCID[i], RAND[i]);
              MidiUSB.flush();
              D_controlChange(MIDICH[i], CCID[i], RAND[i]);
              //SerialUSB.print("FS4 : ");
              //SerialUSB.println(RAND[i]);
              FSp_State[i] = FS_State[i];
            }
  
            else if ( MODE[i] == 3 ){
              FootSwitch[i] = 1;
              programChange(MIDICH[i], PGMV[i]);
              MidiUSB.flush();
              D_programChange(MIDICH[i], PGMV[i]);
              //SerialUSB.print("FS4 : ");
              //SerialUSB.println(PGMV[i]);
              FSp_State[i] = FS_State[i];
            }
  
            else if ( MODE[i] == 4 ){
              FootSwitch[i] = 1;
              noteOn(MIDICH[i], NOTE[i], VELO[i]);
              MidiUSB.flush();
              D_noteOn(MIDICH[i], NOTE[i], VELO[i]);
              //SerialUSB.print("FS4 : ");
              //SerialUSB.println(FootSwitch[i]);
              FSp_State[i] = FS_State[i];
            }
  
            else if ( MODE[i] == 5 ){
              if ( BANK != 9 ){
                FootSwitch[i] = 1;
                BANK = ++BANK;
                //SerialUSB.print("FS4 : ");
                //SerialUSB.println(BANK);
                FSp_State[i] = FS_State[i];
              } else if ( BANK == 9 ){
                FootSwitch[i] = 1;
                BANK = 0;
                //SerialUSB.print("FS4 : ");
                //SerialUSB.println(BANK);
                FSp_State[i] = FS_State[i];
              }
            }
  
            else if ( MODE[i] == 6 ){
              if ( BANK != 0 ){
                FootSwitch[i] = 1;
                BANK = --BANK;
                //SerialUSB.print("FS4 : ");
                //SerialUSB.println(BANK);
                FSp_State[i] = FS_State[i];
              } else if ( BANK == 0 ){
                FootSwitch[i] = 1;
                BANK = 9;
                //SerialUSB.print("FS4 : ");
                //SerialUSB.println(BANK);
                FSp_State[i] = FS_State[i];
              }
            }
  
    //FS4 : Release NO
          } else if ( VAL[i] == 1 ){
            
            if ( MODE[i] == 0 ){
              FSp_State[i] = FS_State[i];
            } 
            
            else if ( MODE[i] == 1 ){
              FootSwitch[i] = 0;
              controlChange(MIDICH[i], CCID[i], MINV[i]);
              MidiUSB.flush();
              D_controlChange(MIDICH[i], CCID[i], MINV[i]);
              //SerialUSB.print("FS4 : ");
              //SerialUSB.println(FootSwitch[i]);
              FSp_State[i] = FS_State[i];
            }
            
            else if ( MODE[i] == 2 ){
              FootSwitch[i] = 0;
              FSp_State[i] = FS_State[i];
            }
  
            else if ( MODE[i] == 3 ){
              FootSwitch[i] = 0;
              FSp_State[i] = FS_State[i];
            }
  
             else if ( MODE[i] == 4 ){
              FootSwitch[i] = 0;
              noteOff(MIDICH[i], NOTE[i], VELO[i]);
              MidiUSB.flush();
              D_noteOff(MIDICH[i], NOTE[i], VELO[i]);
              //SerialUSB.print("FS4 : ");
              //SerialUSB.println(FootSwitch[i]);
              FSp_State[i] = FS_State[i];
            }
            
            else if ( MODE[i] == 5 ){
              FootSwitch[i] = 0;
              FSp_State[i] = FS_State[i];
            }
            
            else if ( MODE[i] == 6 ){
              FootSwitch[i] = 0;
              FSp_State[i] = FS_State[i];
            }
          }
        }
      }        
    }
  }



/////////////////////////////////////////////////////////////////////////////////////////////// Initialize Draft EXP Values

  VAL[4] = 0;
  VAL[5] = 0;

/////////////////////////////////////////////////////////////////////////////////////////////// EXP1 i=4 ON&CTM=2 m=0

  byte iE1 = 4;
  byte mE1 = 0;

  for ( byte avg=0; avg < AVGCT; avg++ ){ 
      VAL[iE1] = VAL[iE1] + analogRead(Pin_V[iE1]);
  }
  
  AVV[mE1] = round( ( AVV[mE1] * 0.95 ) + ( VAL[iE1] / AVGCT * 0.05 ) );
  
  if ( AVV[mE1] != AVVp[mE1] ){
    
    if ( curve[mE1] == 0 ){ 
      EXP[mE1] = map( AVV[mE1], MINSTOMP[mE1], MAXSTOMP[mE1], 0, 127 );
    } else if ( curve[mE1] == 1 ){ 
      if ( AVV[mE1] >= 0 && AVV[mE1] <= 244 ) { EXP[mE1] = map( AVV[mE1], MINSTOMP[mE1], 244, 0, 0);} 
      else if ( AVV[mE1] >= 245 && AVV[mE1] <= 690 ) { EXP[mE1] = map( AVV[mE1], 245, 690, 0, 10); } 
      else if ( AVV[mE1] >= 691 && AVV[mE1] <= 898 ) { EXP[mE1] = map( AVV[mE1], 691, 898, 11, 39); } 
      else if ( AVV[mE1] >= 899 && AVV[mE1] <= 988 ) { EXP[mE1] = map( AVV[mE1], 899, 988, 40, 64); } 
      else if ( AVV[mE1] >= 989 && AVV[mE1] <= 999 ) { EXP[mE1] = map( AVV[mE1], 989, 999, 65, 91); } 
      else if ( AVV[mE1] >= 1000 && AVV[mE1] <= 1005 ) { EXP[mE1] = map( AVV[mE1], 1000, 1005, 94, 114); } 
      else if ( AVV[mE1] >= 1006 && AVV[mE1] <= MAXSTOMP[mE1] ) { EXP[mE1] = map( AVV[mE1], 1001, MAXSTOMP[mE1], 107, 127); }
    }

    AVVp[mE1] = AVV[mE1];
    
  }

  if ( EXP[mE1] != EXPp[mE1] ){
        
    if ( MODE[iE1] == 0 ){
      EXP[mE1] = constrain( map( EXP[mE1], 0, 127, MINV[iE1], MAXV[iE1] ), MINV[iE1], MAXV[iE1] );
      
      if ( outFLAG[mE1] == 1 && 10>=abs(EXP[mE1]-EXPp[mE1])){
        controlChange(MIDICH[iE1], CCID[iE1], EXP[mE1]);
        D_controlChange(MIDICH[iE1], CCID[iE1], EXP[mE1]);
        EXPp[mE1] = EXP[mE1];
      }
      
    } else if ( MODE[iE1] == 1 ){
      EXP[mE1] = map( constrain( map( EXP[mE1], 0, 127, MINV[iE1], MAXV[iE1] ), MINV[iE1], MAXV[iE1] ), MINV[iE1], MAXV[iE1], MAXV[iE1], MINV[iE1] );

      if ( outFLAG[mE1] == 1 && 10>=abs(EXP[mE1]-EXPp[mE1])){
        controlChange(MIDICH[iE1], CCID[iE1], EXP[mE1]);
        D_controlChange(MIDICH[iE1], CCID[iE1], EXP[mE1]);
        EXPp[mE1] = EXP[mE1];
      }
      
    }
    
  }

  if ( ON[2] == 1 ){

    if ( millis() > CTM[2] ){
      outFLAG[mE1] = 1;
      
      if ( outFLAG[mE1] != outFLAGp[mE1] ){
        MAXSTOMP[mE1] = 1011;
        MINSTOMP[mE1] = 24;
        outFLAGp[mE1] = 1;        
      }

    } else {
      EXPp[mE1] = EXP[mE1];
    }
    
  } else if ( ON[2] == 0 ){
    outFLAG[mE1] = 0;

    if ( outFLAG[mE1] != outFLAGp[mE1] ){
      outFLAGp[mE1] = 0;
    }
    
  }


/////////////////////////////////////////////////////////////////////////////////////////////// EXP2 i=5 ON&CTM=3 m=1

  byte iE2 = 5;
  byte mE2 = 1;

  for ( byte avg=0; avg < AVGCT; avg++ ){ 
      VAL[iE2] = VAL[iE2] + analogRead(Pin_V[iE2]);
  }
  
  AVV[mE2] = round( ( AVV[mE2] * 0.95 ) + ( VAL[iE2] / AVGCT * 0.05 ) );
  
  if ( AVV[mE2] != AVVp[mE2] ){
    
    if ( curve[mE2] == 0 ){ 
      EXP[mE2] = map( AVV[mE2], MINSTOMP[mE2], MAXSTOMP[mE2], 0, 127 );
    } else if ( curve[mE2] == 1 ){ 
      if ( AVV[mE2] >= 0 && AVV[mE2] <= 244 ) { EXP[mE2] = map( AVV[mE2], MINSTOMP[mE2], 244, 0, 0);} 
      else if ( AVV[mE2] >= 245 && AVV[mE2] <= 690 ) { EXP[mE2] = map( AVV[mE2], 245, 690, 0, 10); } 
      else if ( AVV[mE2] >= 691 && AVV[mE2] <= 898 ) { EXP[mE2] = map( AVV[mE2], 691, 898, 11, 39); } 
      else if ( AVV[mE2] >= 899 && AVV[mE2] <= 988 ) { EXP[mE2] = map( AVV[mE2], 899, 988, 40, 64); } 
      else if ( AVV[mE2] >= 989 && AVV[mE2] <= 999 ) { EXP[mE2] = map( AVV[mE2], 989, 999, 65, 91); } 
      else if ( AVV[mE2] >= 1000 && AVV[mE2] <= 1005 ) { EXP[mE2] = map( AVV[mE2], 1000, 1005, 94, 114); } 
      else if ( AVV[mE2] >= 1006 && AVV[mE2] <= MAXSTOMP[mE2] ) { EXP[mE2] = map( AVV[mE2], 1006, MAXSTOMP[mE2], 117, 127); }
    }

    AVVp[mE2] = AVV[mE2];
    
  }

  if ( EXP[mE2] != EXPp[mE2] ){
        
    if ( MODE[iE2] == 0 ){
      EXP[mE2] = constrain( map( EXP[mE2], 0, 127, MINV[iE2], MAXV[iE2] ), MINV[iE2], MAXV[iE2] );
      
      if ( outFLAG[mE2] == 1 && 10>=abs(EXP[mE2]-EXPp[mE2])){
        controlChange(MIDICH[iE2], CCID[iE2], EXP[mE2]);
        D_controlChange(MIDICH[iE2], CCID[iE2], EXP[mE2]);
        EXPp[mE2] = EXP[mE2];
      }
      
    } else if ( MODE[iE2] == 1 ){
      EXP[mE2] = map( constrain( map( EXP[mE2], 0, 127, MINV[iE2], MAXV[iE2] ), MINV[iE2], MAXV[iE2] ), MINV[iE2], MAXV[iE2], MAXV[iE2], MINV[iE2] );

      if ( outFLAG[mE2] == 1 && 10>=abs(EXP[mE2]-EXPp[mE2])){
        controlChange(MIDICH[iE2], CCID[iE2], EXP[mE2]);
        D_controlChange(MIDICH[iE2], CCID[iE2], EXP[mE2]);
        EXPp[mE2] = EXP[mE2];
      }
      
    }
    
  }

  if ( ON[3] == 1 ){

    if ( millis() > CTM[3] ){
      outFLAG[mE2] = 1;
      
      if ( outFLAG[mE2] != outFLAGp[mE2] ){
        MAXSTOMP[mE2] = 1011;
        MINSTOMP[mE2] = 24;
        outFLAGp[mE2] = 1;        
      }

    } else {
      EXPp[mE2] = EXP[mE2];
    }
    
  } else if ( ON[3] == 0 ){
    outFLAG[mE2] = 0;

    if ( outFLAG[mE2] != outFLAGp[mE2] ){
      outFLAGp[mE2] = 0;
    }
    
  }



/////////////////////////////////////////////////////////////////////////////////////////////// 7 SEG

  if ( LCDR == 0 ){
    digitalWrite( Pin_RCLK, LOW );
    shiftOut( Pin_SER, Pin_SCLK, LSBFIRST, SEGP[BANK] );
    digitalWrite( Pin_RCLK, HIGH );
  } else if ( LCDR == 1 ){
    digitalWrite( Pin_RCLK, LOW );
    shiftOut( Pin_SER, Pin_SCLK, LSBFIRST, SEGPr[BANK] );
    digitalWrite( Pin_RCLK, HIGH );
  }

  


}
