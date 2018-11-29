#include <SoftwareSerial.h>
SoftwareSerial mySerial(10, 11); // RX, TX
#include "Arduino.h"
#include "AX12A.h"

  unsigned char Checksum;
  unsigned char Direction_Pin;
  unsigned char Time_Counter;
  unsigned char Incoming_Byte;               
  unsigned char Position_High_Byte;
  unsigned char Position_Low_Byte;
  unsigned char Speed_High_Byte;
  unsigned char Speed_Low_Byte;
  unsigned char Load_High_Byte;
  unsigned char Load_Low_Byte;
  
  int Moving_Byte;
  int RWS_Byte;
  int Speed_Long_Byte;
  int Load_Long_Byte;
  int Position_Long_Byte;
  int Temperature_Byte;
  int Voltage_Byte;
  int Error_Byte; 

  int returned_Byte;

  int read_error(void);

  HardwareSerial *varSerial;


int16_t speed1[3]={50,10,11};

int sendAXPacket(unsigned char *packet, unsigned int length);
void setup() {

  mySerial.begin(1000000);
  setEndless(254, OFF);

}

void loop() { 
//  for (int i=0; i<3;i++){
//   mySerial.write( speed1[i]); 
//  }
moveSpeed(254,100,100);
}

int move(unsigned char ID, int Position)
{
    char Position_H,Position_L;
    Position_H = Position >> 8;           // 16 bits - 2 x 8 bits variables
    Position_L = Position;

    const unsigned int length = 9;
    unsigned char packet[length];

  Checksum = (~(ID + AX_GOAL_LENGTH + AX_WRITE_DATA + AX_GOAL_POSITION_L + Position_L + Position_H)) & 0xFF;

    packet[0] = AX_START;
    packet[1] = AX_START;
    packet[2] = ID;
    packet[3] = AX_GOAL_LENGTH;
    packet[4] = AX_WRITE_DATA;
    packet[5] = AX_GOAL_POSITION_L;
    packet[6] = Position_L;
    packet[7] = Position_H;
    packet[8] = Checksum;

    return (sendPacket(packet, length));
}

int sendPacket(unsigned char *packet, unsigned int length){
    for (int i=0; i<length;i++){
   mySerial.write( packet[i]); 
  }
}

//void begin(long baud, unsigned char directionPin, HardwareSerial *srl)
//{
//  varSerial = srl;
//  Direction_Pin = directionPin;
//  setDPin(Direction_Pin, OUTPUT);
//  beginCom(baud);
//}


int setEndless(unsigned char ID, bool Status)
{
  if ( Status )
  {
    const unsigned int length = 9;
    unsigned char packet[length];

    Checksum = (~(ID + AX_GOAL_LENGTH + AX_WRITE_DATA + AX_CCW_ANGLE_LIMIT_L)) & 0xFF;

      packet[0] = AX_START;
      packet[1] = AX_START;
      packet[2] = ID;
      packet[3] = AX_GOAL_LENGTH;
      packet[4] = AX_WRITE_DATA;
      packet[5] = AX_CCW_ANGLE_LIMIT_L;
      packet[6] = 0;            // full rotation
      packet[7] = 0;            // full rotation
      packet[8] = Checksum;

      return (sendPacket(packet, length));
  }
  else
  {
//    turn(ID,0,0);

    const unsigned int length = 9;
    unsigned char packet[length];

    Checksum = (~(ID + AX_GOAL_LENGTH + AX_WRITE_DATA + AX_CCW_ANGLE_LIMIT_L + AX_CCW_AL_L + AX_CCW_AL_H)) & 0xFF;

      packet[0] = AX_START;
      packet[1] = AX_START;
      packet[2] = ID;
      packet[3] = AX_GOAL_LENGTH;
      packet[4] = AX_WRITE_DATA;
      packet[5] = AX_CCW_ANGLE_LIMIT_L;
      packet[6] = AX_CCW_AL_L;
      packet[7] = AX_CCW_AL_H;
      packet[8] = Checksum;

      return (sendPacket(packet, length));
  }
}

int moveSpeed(unsigned char ID, int Position, int Speed)
{
    char Position_H,Position_L,Speed_H,Speed_L;
    Position_H = Position >> 8;    
    Position_L = Position;                // 16 bits - 2 x 8 bits variables
    Speed_H = Speed >> 8;
    Speed_L = Speed;                      // 16 bits - 2 x 8 bits variables

    const unsigned int length = 11;
    unsigned char packet[length];
    
    Checksum = (~(ID + AX_GOAL_SP_LENGTH + AX_WRITE_DATA + AX_GOAL_POSITION_L + Position_L + Position_H + Speed_L + Speed_H)) & 0xFF;

    packet[0] = AX_START;
    packet[1] = AX_START;
    packet[2] = ID;
    packet[3] = AX_GOAL_SP_LENGTH;
    packet[4] = AX_WRITE_DATA;
    packet[5] = AX_GOAL_POSITION_L;
    packet[6] = Position_L;
    packet[7] = Position_H;
    packet[8] = Speed_L;
    packet[9] = Speed_H;
    packet[10] = Checksum;

    return (sendPacket(packet, length));
}
