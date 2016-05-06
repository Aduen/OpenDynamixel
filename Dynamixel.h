/*
 * Dynamixel.h
 *
 *  Created on: 2013. 11. 8.
 *      Author: in2storm
 *      brief : 2013-11-12 [ROBOTIS] revised for OpenCM board
 */

#ifndef DYNAMIXEL_H_
#define DYNAMIXEL_H_

#include "Arduino.h"
#include "dxl.h"

class Dynamixel {
public:
	Dynamixel(HardwareSerial *serial);
	virtual ~Dynamixel();

	/////////// Device control methods /////////////
	void begin(int buad, uint8_t DE_pin);
	uint8_t readRaw(void);
	uint8_t available(void);
	void writeRaw(uint8_t);

	byte getResult(void); // use getTxRxStatus() instead of getResult() method
	byte getTxRxStatus(void);// made by NaN (Robotsource.org)
	byte getError(byte errbit);
	byte setLibStatusReturnLevel(byte); // made by NaN (Robotsource.org)
	byte setLibNumberTxRxAttempts(byte);// made by NaN (Robotsource.org)

	byte txRxPacket(byte bID, byte bInst, int bTxParaLen);
	byte txPacket(byte bID, byte bInstruction, int bParameterLength);
	byte rxPacket(int bRxLength);

	void setPacketType(byte ver);
	byte getPacketType(void);

	word  ping(byte  bID);
	uint32_t  ping(void); //Broadcast ping in DXL 2.0 protocol

	//// High communication methods ////////
	byte readByte(byte bID, word bAddress);
	byte writeByte(byte bID, word bAddress, byte bData);
	word readWord(byte bID, word bAddress);
	byte writeWord(byte bID, word bAddress, word wData);

	byte writeDword( byte bID, word wAddress, uint32_t value );
	uint32_t readDword( byte bID, word wAddress );

	byte setPosition(byte ServoID, int Position, int Speed);
	byte syncWrite(int start_addr, byte data_length, int *param, int param_length);// DWORD(32bit) syncwrite() for DXL PRO
	byte syncWrite(int start_addr, int data_length, word *param, int param_length); // WORD(16bit) syncwrite() for DXL


	/////// Methods for making a packet ////////
	void setTxPacketId( byte id );
	void setTxPacketInstruction( byte instruction );
	void setTxPacketParameter( byte index, byte value );
	void setTxPacketLength( byte length );
	byte txrxPacket(void);
	int getRxPacketParameter( int index );
	int getRxPacketLength(void);


	//Easy Functions for DXL
	word getModelNumber(byte bID);

	void setID(byte current_ID, byte new_ID);
	void setBaud(byte bID, byte baud_num);

	void returnLevel(byte bID, byte level);
	byte returnLevel(byte bID);

	void returnDelayTime(byte bID, byte time);
	byte returnDelayTime(byte bID);

	void alarmShutdown(byte bID,byte option);
	byte alarmShutdown(byte bID);

	void controlMode(byte bID, byte mode); // change wheel, joint
	byte controlMode(byte bID); // return current mode

	void jointMode(byte bID);
	void wheelMode(byte bID);

	void maxTorque(byte bID, word value);
	word maxTorque(byte bID);

	void maxVolt(byte bID, byte value);
	byte maxVolt(byte bID);

	void minVolt(byte bID, byte value);
	byte minVolt(byte bID);

	void maxTemperature(byte bID, byte temp);
	byte maxTemperature(byte bID);

	void torqueEnable(byte bID);
	void torqueDisable(byte bID);

	void cwAngleLimit(byte bID, word angle);
	word cwAngleLimit(byte bID);
	void ccwAngleLimit(byte bID, word angle);
	word ccwAngleLimit(byte bID);

	void goalPosition(byte bID, int position);
	void goalSpeed(byte bID, int speed);
	void goalTorque(byte bID, int torque);
    
    void punch(byte bID, int kick);

	int getPosition(byte bID);
	int getSpeed(byte bID);
	int getLoad(byte bID);
	int getVolt(byte bID);
	byte getTemperature(byte bID);
	byte isMoving(byte bID);

	void ledOn(byte bID);
	void ledOn(byte bID, byte option);  //for XL-320 , DXL PRO
	void ledOff(byte bID);

	void setPID(byte bID, byte propotional, byte integral, byte derivative);

	void cwTurn(byte bID, word speed); //cwTurn()���� ����
	void ccwTurn(byte bID, word speed);//ccwTurn()
    
    //PACKET CONSTRUCTION
	void initPacket(byte bID, byte bInst);
	void pushByte(byte value);
	void pushParam(int value);
	void pushParam(byte value);
	void flushPacket(void);

	/*
	 * Utility methods for Dynamixel
	 */

	/*byte getLowByte( word wData ); //can be replaced by DXL_LOBYTE(w)
	byte getHighByte( word wData );//can be replaced by DXL_HIBYTE(w)
	word makeWord( byte lowbyte, byte highbyte ); //can be replaced by DXL_MAKEWORD(w)*/

private:
	void printBuffer(byte *bpPrintBuffer, byte bLength);

	byte checkPacketType(void);

	uint8_t setDxlLibStatRtnLvl(uint8_t); // inspired by NaN (robotsource.org)
	uint8_t setDxlLibNumTries(uint8_t); // inspired by NaN (robotsource.org)

	HardwareSerial *mDxlUsart;  /*< USART Device*/

	uint8_t mDirPin;

	uint8_t mRxBuffer[DXL_RX_BUF_SIZE];
	uint8_t mTxBuffer[DXL_RX_BUF_SIZE];
	uint8_t mParamBuffer[DXL_PARAMETER_BUF_SIZE];
	uint8_t mBusUsed;
	uint8_t mRxLength;  // the length of the received data from dynamixel bus

	// additions to return proper COMM_* status
	uint8_t mDXLtxrxStatus;  // inspired by NaN (robotsource.org)
	// additions to permit non-default Status Return Level settings without returning errors
	uint8_t gbDXLStatusReturnLevel;
	// additions to adjust number of txrx attempts
	uint8_t gbDXLNumberTxRxAttempts;

	uint8_t mPacketType;  //2014-04-02

	byte mPktIdIndex;
	byte mPktLengthIndex;
	byte mPktInstIndex;
	byte mPktErrorIndex;

	byte mbLengthForPacketMaking;
	byte mbIDForPacketMaking;
	byte mbInstructionForPacketMaking;
	byte mCommStatus;

	byte SmartDelayFlag;
};


#endif /* DYNAMIXEL_H_ */
