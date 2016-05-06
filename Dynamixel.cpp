/*
 * Dynamixel.cpp
 *
 *  Created on: 2013. 11. 8.
 *      Author: in2storm
 */
#include "Dynamixel.h"

Dynamixel::Dynamixel(HardwareSerial *serial) {
    mDxlUsart = serial;
    SmartDelayFlag = 1;
    debugFlag = 0;
}

Dynamixel::~Dynamixel() {
    // TODO Auto-generated destructor stub
}
void Dynamixel::begin(int baud, uint8_t DE_pin){
    
    mDxlUsart->begin(baud, SERIAL_8N1);
    mDirPin = DE_pin;
    mDxlUsart->transmitterEnable(mDirPin);
    
    delay(100);
    mDXLtxrxStatus = 0;
    mBusUsed = 0;// only 1 when tx/rx is operated
    
    this->setLibStatusReturnLevel(2);
    this->setLibNumberTxRxAttempts(1);
    
    mDxlUsart->clear();
    
    mPktIdIndex = 4;
    mPktLengthIndex = 5;
    mPktInstIndex = 7;
    mPktErrorIndex = 8;
}

void Dynamixel::addDebugStream(HardwareSerial *serial)
{
    debugFlag = 1;
    mDebugUsart = serial;
}

byte Dynamixel::readRaw(void){
    return mDxlUsart->read();
}

void Dynamixel::writeRaw(uint8_t value){
    mDxlUsart->write(value);
}

/*
 * @brief : if data coming from dxl bus, returns 1, or if not, returns 0.
 *
 */
byte Dynamixel::available(void){
    return mDxlUsart->available();
}

byte Dynamixel::getPacketType(void){
    return mPacketType;
}

byte Dynamixel::setLibStatusReturnLevel(byte num)
{
    gbDXLStatusReturnLevel = num;
    return gbDXLStatusReturnLevel;
}

byte Dynamixel::setLibNumberTxRxAttempts(byte num)
{
    gbDXLNumberTxRxAttempts = num;
    return gbDXLNumberTxRxAttempts;
}
/*
 * return value for getTxRxStatus(), getResult();
 #define	COMM_TXSUCCESS		(0)
 #define COMM_RXSUCCESS		(1)
 #define COMM_TXFAIL			(2)
 #define COMM_RXFAIL			(3)
 #define COMM_TXERROR		(4)
 #define COMM_RXWAITING		(5)
 #define COMM_RXTIMEOUT		(6)
 #define COMM_RXCORRUPT		(7)
 */
byte Dynamixel::getTxRxStatus(void) // made by NaN (Robotsource.org)
{
    return mDXLtxrxStatus;
}
/*
 * Use getTxRxStatus() instead of getResult()
 * */
byte  Dynamixel::getResult(void){
    //	return mCommStatus;
    return this->getTxRxStatus();
}
/*
 *  ERROR Bit table is below.
 
 //DXL 1.0 protocol
 #define ERRBIT_VOLTAGE		(1)
 #define ERRBIT_ANGLE		(2)
 #define ERRBIT_OVERHEAT		(4)
 #define ERRBIT_RANGE		(8)
 #define ERRBIT_CHECKSUM		(16)
 #define ERRBIT_OVERLOAD		(32)
 #define ERRBIT_INSTRUCTION	(64)
 
 //DXL 2.0 protocol
 #define ERRBIT_RESULT_FAIL	(1)
 #define ERRBIT_INST_ERROR	(2)
 #define ERRBIT_CRC			(4)
 #define ERRBIT_DATA_RANGE	(8)
 #define ERRBIT_DATA_LENGTH	(16)
 #define ERRBIT_DATA_LIMIT	(32)
 #define ERRBIT_ACCESS		(64)
 * */
byte Dynamixel::getError( byte errbit )
{
    //return dxl_get_rxpacket_error( errbit );
    if( mRxBuffer[8] & errbit ) return 1;
    else return 0;
}

byte Dynamixel::txPacket(byte bID, byte bInstruction, int bParameterLength)
{
    uint16_t bCount,bCheckSum,bPacketLength;
    byte offsetParamIndex;
    
    //dxl protocol 2.0
    mTxBuffer[0] = 0xff;
    mTxBuffer[1] = 0xff;
    mTxBuffer[2] = 0xfd;
    mTxBuffer[3] = 0x00;
    mTxBuffer[4] = bID;
    //get parameter length
    mTxBuffer[5] = DXL_LOBYTE(bParameterLength+3);// 3(byte) <- instruction(1byte) + checksum(2byte)
    mTxBuffer[6] = DXL_HIBYTE(bParameterLength+3);
    mTxBuffer[7] = bInstruction;
    
    offsetParamIndex = 8;
    bPacketLength = bParameterLength+3+7; //parameter length 3bytes, 7bytes =  packet header 4bytes, ID 1byte,  length 2bytes
    
    //copy parameters from mParamBuffer to mTxBuffer
    for(bCount = 0; bCount < bParameterLength; bCount++)
    {
        mTxBuffer[bCount+offsetParamIndex] = mParamBuffer[bCount];
    }
    
    bCheckSum = update_crc(0, mTxBuffer, bPacketLength-2);  // -2 : except CRC16
    
    mTxBuffer[bPacketLength-2] = DXL_LOBYTE(bCheckSum);     // last - 2
    mTxBuffer[bPacketLength-1] = DXL_HIBYTE(bCheckSum);     // last - 1
    
    //the old code used to have a DE pin toggle, this is now ommitted since the Teensy will take care of this
    uint16_t p_size = mDxlUsart->write(mTxBuffer, bPacketLength);
    
    return(p_size); // return packet length
}

//TODO: there is still an issue with multiple calls following each other up.
byte Dynamixel::rxPacket(int bRxLength)
{
    unsigned long ulCounter, ulTimeLimit;
    word bCount, bLength, bChecksum;
    byte bTimeout;
    
    bTimeout = 0;
    if(bRxLength == 255 || bRxLength == 0xffff) //2014-04-03
        ulTimeLimit = RX_TIMEOUT_COUNT1;
    else
        ulTimeLimit = RX_TIMEOUT_COUNT2;
    for(bCount = 0; bCount < bRxLength; bCount++)
    {
        ulCounter = 0;
        while(mDxlUsart->available() == 0)
        {
            delayMicroseconds(12); //[ROBOTIS] porting ydh //it was in nano's for some reason, 12000nS
            if(ulCounter++ > ulTimeLimit)
            {
                bTimeout = 1;
                break;
            }
        }
        
        if(bTimeout) break;
        mRxBuffer[bCount] = mDxlUsart->read(); // get one byte from USART device
    }
    
    bLength = bCount;
    bChecksum = 0;
    if( mTxBuffer[mPktIdIndex] != BROADCAST_ID )
    {
        if(bTimeout && bRxLength != 255)
        {
            mDXLtxrxStatus |= (1<<COMM_RXTIMEOUT);
            mDxlUsart->clear();
            
            return 0;
        }
        if(bLength > 3) //checking available length.
        {
            
            // Dxl 2.0 header check
            if(mRxBuffer[0] != 0xff || mRxBuffer[1] != 0xff || mRxBuffer[2] != 0xfd)
            {
                if(debugFlag) mDebugUsart->println("Wrong Header");//[Wrong Header]
                mDXLtxrxStatus |= (1<<COMM_RXCORRUPT);//RXHEADER);
                mDxlUsart->clear();
                return 0;
            }
            // Dxl match send and recv id's
            if(mRxBuffer[mPktIdIndex] != mTxBuffer[mPktIdIndex] )  //id check
            {
                if(debugFlag) mDebugUsart->println("[Error:TxID != RxID]");
                mDXLtxrxStatus |= (1<<COMM_RXCORRUPT);//RXID);
                mDxlUsart->clear();
                return 0;
            }
            
            //Dxl check status packet length
            if(mRxBuffer[mPktLengthIndex] != bLength-mPktInstIndex) // status packet length check
            {
                if(debugFlag) mDebugUsart->println("RxLength Error");
                mDXLtxrxStatus |= (1<<COMM_RXCORRUPT);//RXLENGTH);
                mDxlUsart->clear();
                return 0;
            }
            
            int bTryCount = 0;
            for(bTryCount = 1; bTryCount<= 7; bTryCount++){
                if((mRxBuffer[mPktErrorIndex] && debugFlag) == bTryCount){
                    switch(bTryCount){
                        case 1:
                            mDebugUsart->println("Result Fail");
                            break;
                        case 2:
                            mDebugUsart->println("Instruction Error");
                            break;
                        case 3:
                            mDebugUsart->println("CRC Error");
                            break;
                        case 4:
                            mDebugUsart->println("DataRange Error");
                            break;
                        case 5:
                            mDebugUsart->println("DataLength Error");
                            break;
                        case 6:
                            mDebugUsart->println("DataLimit Error");
                            break;
                        case 7:
                            mDebugUsart->println("Accrss Error");
                            break;
                    }
                }
            }
            
            
            // Dxl 2.0 checksum CRC16
            bChecksum = DXL_MAKEWORD(mRxBuffer[bRxLength-2], mRxBuffer[bRxLength-1]);
            if(update_crc(0, mRxBuffer, bRxLength-2) == bChecksum){
                return bLength;
            }
            else{
                if(debugFlag) mDebugUsart->println("CRC-16 Error\r\n");
                return 0;
            }
        }//(bLength > 3)
    }//end of Rx status packet check
    
    return bLength;
}

void Dynamixel::printBuffer(byte *bpPrintBuffer, byte bLength)
{
    if(debugFlag == 0) return;
    
    byte bCount;
    if(bLength == 0)
    {
        if(mTxBuffer[2] == BROADCAST_ID)
        {
            mDebugUsart->println("No Data[at Broadcast ID 0xFE]");
        }
        else
        {
            mDebugUsart->println("No Data(Check ID, Operating Mode, Baud rate)");
        }
    }
    for(bCount = 0; bCount < bLength; bCount++)
    {
        mDebugUsart->print(bpPrintBuffer[bCount]);
        mDebugUsart->print(' ');
    }
    mDebugUsart->print(" LEN:");//("(LEN:")
    mDebugUsart->println(bLength);
}

byte Dynamixel::txRxPacket(byte bID, byte bInst, int bTxParaLen)
{
    mDXLtxrxStatus = 0;
    
    uint16_t bTxLen, bRxLenEx, bTryCount;
    bRxLenEx = bTxLen = 0;
    
    uint8_t mBusUsed = 1;
    uint8_t mRxLength = 0;
    
    //retries are currently down to one...
    //this for-loop is useless since there is no break if succesfull the first time
    for(bTryCount = 0; bTryCount < gbDXLNumberTxRxAttempts; bTryCount++)
    {
        mDxlUsart->clear();
        
        //TX
        bTxLen = txPacket(bID, bInst, bTxParaLen);
        
        //Dxl 2.0 Tx success?
        if (bTxLen == (bTxParaLen+3+7))	mDXLtxrxStatus = (1<<COMM_TXSUCCESS);
        
        //determine returned packet length
        if(bInst == INST_PING){
            //Dxl 2.0
            if(bID == BROADCAST_ID)	mRxLength = bRxLenEx = 0xffff;
            else mRxLength = bRxLenEx = 14;
            
        }
        else if(bInst == INST_READ){
            if (gbDXLStatusReturnLevel > 0){
                mRxLength = bRxLenEx = 11+DXL_MAKEWORD(mParamBuffer[2], mParamBuffer[3]);
            }
            else{
                mRxLength = bRxLenEx = 0;
            }
            
        }
        else if( bID == BROADCAST_ID ){
            if(bInst == INST_SYNC_READ || bInst == INST_BULK_READ) mRxLength = bRxLenEx = 0xffff;
            else mRxLength = bRxLenEx = 0; // no response packet
        }
        else{
            if (gbDXLStatusReturnLevel>1){
                mRxLength = bRxLenEx = 11;
            }
            else{
                mRxLength = bRxLenEx = 0;
            }
        }
        
        if(bRxLenEx){
            //unsure if this delay is actually "smart"
            if(SmartDelayFlag == 1)
                delayMicroseconds(110);
            //RX
            mRxLength = rxPacket(bRxLenEx);
            //TODO: find out more about this bug
            //there is a problem with calls following each other up, for now a 400-500uS delay seems to do the trick
            delayMicroseconds(400);
        }
    }
    
    if((mRxLength != bRxLenEx) && (mTxBuffer[mPktIdIndex] != BROADCAST_ID))
    {
        return 0;
    }else if((mRxLength == 0) && (mTxBuffer[mPktInstIndex] == INST_PING)){
        return 0;
    }
    
    mDXLtxrxStatus = (1<<COMM_RXSUCCESS);
    
    return 1;
}

word  Dynamixel::ping(byte  bID )
{
    if(this->txRxPacket(bID, INST_PING, 0))
        return DXL_MAKEWORD(mRxBuffer[9],mRxBuffer[10]); //return product code when 2.0
    
    return 0xffff;  //no dxl in bus.
}

/*
 * Broadcast ping for DXL 2.0 protocol
 * return : bit set each dynaxmel on bus. but it is limit to 32 DXLs
 * */
uint32_t Dynamixel::ping(void)
{
    int i=0;
    uint32_t result=0;
    
    if(this->txRxPacket(BROADCAST_ID, INST_PING, 0)){
        for(i=0; i < 32*14; i+=14){
            if(mRxBuffer[i] == 0xFF && mRxBuffer[i+1] == 0xFF && mRxBuffer[i+2] == 0xFD){
                result |= 1UL << mRxBuffer[i+4];
            }
        }
        return result;
    }
    
    return 0xFFFFFFFF;  //no dxl in bus.
}

byte  Dynamixel::writeByte(byte bID, word bAddress, byte bData)
{
    byte param_length = 0;
    
    //insert wAddress to parameter bucket
    mParamBuffer[0]	= (unsigned char)DXL_LOBYTE(bAddress);
    mParamBuffer[1]	= (unsigned char)DXL_HIBYTE(bAddress);
    //insert data to parameter bucket
    mParamBuffer[2]	= bData;
    param_length = 3;
    
    return this->txRxPacket(bID, INST_WRITE, param_length);
}

byte Dynamixel::readByte(byte bID, word bAddress)
{
    mDxlUsart->clear();
    
    mParamBuffer[0]	= (unsigned char)DXL_LOBYTE(bAddress);
    mParamBuffer[1]	= (unsigned char)DXL_HIBYTE(bAddress);
    mParamBuffer[2]	= 1; //1byte
    mParamBuffer[3]	= 0;
    if( this->txRxPacket(bID, INST_READ, 4 )){
        return(mRxBuffer[9]);//refer to 2.0 packet structure
    }
    
    return 0xff;
}



byte Dynamixel::writeWord(byte bID, word bAddress, word wData)
{
    byte param_length = 0;
    mDxlUsart->clear();
    
    mParamBuffer[0]	= (unsigned char)DXL_LOBYTE(bAddress);
    mParamBuffer[1]	= (unsigned char)DXL_HIBYTE(bAddress);
    //insert data to parameter bucket
    mParamBuffer[2]	= DXL_LOBYTE(wData);
    mParamBuffer[3]	= DXL_HIBYTE(wData);
    param_length = 4;
    
    return this->txRxPacket(bID, INST_WRITE, param_length);
}



word Dynamixel::readWord(byte bID, word bAddress)
{
    mDxlUsart->clear();
    
    mParamBuffer[0]	= (unsigned char)DXL_LOBYTE(bAddress);
    mParamBuffer[1]	= (unsigned char)DXL_HIBYTE(bAddress);
    mParamBuffer[2]	= 2; //2byte
    mParamBuffer[3]	= 0;
    if(this->txRxPacket(bID, INST_READ, 4)){
        return(DXL_MAKEWORD(mRxBuffer[9], mRxBuffer[10]));
    }
    
    return 0xffff;
}


byte Dynamixel::writeDword( byte bID, word wAddress, uint32_t value )
{
    mDxlUsart->clear();
    
    //insert wAddress to parameter bucket
    mParamBuffer[0]	= (unsigned char)DXL_LOBYTE(wAddress);
    mParamBuffer[1]	= (unsigned char)DXL_HIBYTE(wAddress);
    //insert data to parameter bucket
    mParamBuffer[2]	= DXL_LOBYTE(DXL_LOWORD(value));
    mParamBuffer[3]	= DXL_HIBYTE(DXL_LOWORD(value));
    mParamBuffer[4]	= DXL_LOBYTE(DXL_HIWORD(value));
    mParamBuffer[5]	= DXL_HIBYTE(DXL_HIWORD(value));
    
    return this->txRxPacket(bID, INST_WRITE, 6); //// parameter length 4 = 2(address)+2(data)
}

uint32_t Dynamixel::readDword( byte bID, word wAddress )
{
    mParamBuffer[0]	= (unsigned char)DXL_LOBYTE(wAddress);
    mParamBuffer[1]	= (unsigned char)DXL_HIBYTE(wAddress);
    mParamBuffer[2]	= 4; //4byte
    mParamBuffer[3]	= 0;
    if(this->txRxPacket(bID, INST_READ, 4)){
        return DXL_MAKEDWORD( DXL_MAKEWORD( mRxBuffer[9], mRxBuffer[10]),
                             DXL_MAKEWORD( mRxBuffer[11], mRxBuffer[12]));
    }
    
    return 0xFFFFFFFF;
}

/*
 * @brief Sets the target position and speed of the specified servo
 * @author Made by Martin S. Mason(Professor @Mt. San Antonio College)
 * @change 2013-04-17 changed by ROBOTIS,.LTD.
 * */
byte Dynamixel::setPosition(byte ServoID, int Position, int Speed)
{
    byte param_length = 0;
    
    mParamBuffer[0]	= 30;
    mParamBuffer[1]	= 0;
    //insert data to parameter bucket
    mParamBuffer[2] = (unsigned char)DXL_LOBYTE(Position);
    mParamBuffer[3] = (unsigned char)DXL_HIBYTE(Position);
    mParamBuffer[4] = (unsigned char)DXL_LOBYTE(Speed);
    mParamBuffer[5] = (unsigned char)DXL_HIBYTE(Speed);
    param_length = 6;
    
    return (this->txRxPacket(ServoID, INST_WRITE, param_length));
    
}
byte Dynamixel::syncWrite(int start_addr, int data_length, word *param, int param_length)
{
    int i=0, j=0, k=0, num=0;
    mDxlUsart->clear();
    
    num = param_length/(data_length + 1); //ID+DATA1+DATA2..
    
    mParamBuffer[0]   = DXL_LOBYTE(start_addr);
    mParamBuffer[1]   = DXL_HIBYTE(start_addr);
    mParamBuffer[2]   = DXL_LOBYTE(data_length*2);
    mParamBuffer[3]   = DXL_HIBYTE(data_length*2);
    
    for(i=4; i < (4+num*(1+data_length*2)); i+=(1+data_length*2) ){
        mParamBuffer[i]   = (byte)param[k++]; //ID
        for(j=0; j < (data_length*2); j+=2){
            mParamBuffer[i+j+1] = DXL_LOBYTE(param[k]); //low byte
            mParamBuffer[i+j+2] = DXL_HIBYTE(param[k]); //high byte
            k++;
        }
    }
    
    return this->txRxPacket(BROADCAST_ID, INST_SYNC_WRITE, i);
}

byte Dynamixel::syncWrite(int start_addr, byte data_length, int *param, int param_length){
    int i=0, j=0, k=0, num=0;
    
    mDxlUsart->clear();
    
    num = param_length / (data_length + 1);
    
    mParamBuffer[0]   = DXL_LOBYTE(start_addr);
    mParamBuffer[1]   = DXL_HIBYTE(start_addr);
    mParamBuffer[2]   = DXL_LOBYTE(data_length*4);
    mParamBuffer[3]   = DXL_HIBYTE(data_length*4);
    
    for(i=4; i < (4+num*(1+data_length*4)); i+=(1+data_length*4) ){
        mParamBuffer[i]   = (byte)param[k++]; //ID
        for(j=0; j < (data_length*4); j+=4){
            mParamBuffer[i+j+1] = DXL_LOBYTE(DXL_LOWORD(param[k])); //data
            mParamBuffer[i+j+2] = DXL_HIBYTE(DXL_LOWORD(param[k]));
            mParamBuffer[i+j+3] = DXL_LOBYTE(DXL_HIWORD(param[k]));
            mParamBuffer[i+j+4] = DXL_HIBYTE(DXL_HIWORD(param[k]));
            k++;
        }
        
    }
    return this->txRxPacket(BROADCAST_ID, INST_SYNC_WRITE, 4+i);
}

void Dynamixel::setTxPacketId(byte id)
{
    mbIDForPacketMaking = id;
}

void Dynamixel::setTxPacketInstruction(byte instruction)
{
    mbInstructionForPacketMaking = instruction;
}

void Dynamixel::setTxPacketParameter( byte index, byte value )
{
    mParamBuffer[index] = value;
}

void Dynamixel::setTxPacketLength( byte length )
{
    mbLengthForPacketMaking = length;
}

byte Dynamixel::txrxPacket(void)
{
    mCommStatus = this->txRxPacket(mbIDForPacketMaking, mbInstructionForPacketMaking, mbLengthForPacketMaking);
    return mCommStatus;
}

int Dynamixel::getRxPacketParameter( int index )
{
    return mRxBuffer[5 + index];
}

int Dynamixel::getRxPacketLength(void)
{
    return mRxBuffer[3]; //length index is 3 in status packet
}

word Dynamixel::getModelNumber(byte bID)
{
    return this->readWord(bID, 0);
}

void Dynamixel::setID(byte current_ID, byte new_ID)
{
    this->writeByte(current_ID, 3, new_ID);
}

void Dynamixel::setBaud(byte bID, byte baud_num)
{
    this->writeByte(bID, 4, baud_num);
}

void Dynamixel::returnLevel(byte bID, byte level)
{
    this->writeByte(bID, 17, level);
}

byte Dynamixel::returnLevel(byte bID)
{
    return this->readByte(bID, 17);
}

void Dynamixel::returnDelayTime(byte bID, byte time)
{
    this->writeByte(bID, 5, time);
}

byte Dynamixel::returnDelayTime(byte bID)
{
    return this->readByte(bID, 5);
}

void Dynamixel::alarmShutdown(byte bID,byte option)
{
    this->writeByte(bID, 18, option);
}

byte Dynamixel::alarmShutdown(byte bID)
{
    return this->readByte(bID, 18);
}

void Dynamixel::controlMode(byte bID, byte mode)
{
    word model=0;
    this->writeByte(bID, 24, 0);
    this->writeByte(bID, 11, mode);
    this->writeByte(bID, 24, 1);
}

byte Dynamixel::controlMode(byte bID)
{
    return this->readByte(bID, 11);
}

void Dynamixel::wheelMode(byte bID)
{
    this->writeByte(bID, 24, 0);
    this->writeByte(bID, 11, 1);
    this->writeByte(bID, 24, 1);
}

void Dynamixel::jointMode(byte bID)
{
    word model=0;
    this->writeByte(bID, 24, 0);
    this->writeByte(bID, 11, 2);
    this->writeByte(bID, 24, 1);
}

void Dynamixel::maxTorque(byte bID, word value)
{
    this->writeWord(bID, 15, value);
}

word Dynamixel::maxTorque(byte bID)
{
    return this->readWord(bID, 15);
}

void Dynamixel::maxVolt(byte bID, byte value)
{
    this->writeByte(bID, 14, value);
}

byte Dynamixel::maxVolt(byte bID)
{
    return this->readByte(bID, 14);
}

void Dynamixel::minVolt(byte bID, byte value)
{
    this->writeByte(bID, 13, value);
}

byte Dynamixel::minVolt(byte bID)
{
    return this->readByte(bID, 13);
}

void Dynamixel::maxTemperature(byte bID, byte temp)
{
    this->writeByte(bID, 12, temp);
}

byte Dynamixel::maxTemperature(byte bID)
{
    return this->readByte(bID, 12);
}

void Dynamixel::torqueEnable(byte bID)
{
    this->writeByte(bID, 24, 1);
}

void Dynamixel::torqueDisable(byte bID)
{
    this->writeByte(bID, 24, 0);
}

void Dynamixel::cwAngleLimit(byte bID, word angle)
{
    this->writeWord(bID, 6, angle);
}

word Dynamixel::cwAngleLimit(byte bID)
{
    return this->readWord(bID, 6);
}

void Dynamixel::ccwAngleLimit(byte bID, word angle)
{
    this->writeWord(bID, 8, angle);
}

word Dynamixel::ccwAngleLimit(byte bID)
{
    return this->readWord(bID, 8);
}

void Dynamixel::goalPosition(byte bID, int position)
{
    this->writeWord(bID, 30, position);
}

void Dynamixel::goalSpeed(byte bID, int speed)
{
    this->writeWord(bID, 32, speed);
}

void Dynamixel::goalTorque(byte bID, int torque)
{
    this->writeWord(bID, 35, torque);
}

int Dynamixel::getPosition(byte bID)
{
    return this->readWord(bID, 37);
}

int Dynamixel::getSpeed(byte bID)
{
    return this->readWord(bID, 39);
}

int Dynamixel::getLoad(byte bID)
{
    return this->readWord(bID, 41);
}

int Dynamixel::getVolt(byte bID)
{
    return this->readByte(bID, 45);
}

byte Dynamixel::getTemperature(byte bID)
{
    return this->readByte(bID, 46);
}

byte Dynamixel::isMoving(byte bID)
{
    return this->readByte(bID, 49);
}

void Dynamixel::ledOn(byte bID)
{
    this->writeByte(bID, 25, 1);
}

void Dynamixel::punch(byte bID, int kick)
{
    this->writeWord(bID, 51, kick);
}

/**
 Turn on/off the internal led on the servo (XL-320)
 
 OPTIONS
 000 = off = 0
 001 = red = 1
 010 = green = 2
 011 = yellow = 3
 100 = blue = 4
 101 = pink = 5
 110 = turqouise = 6
 111 = white = 7
 **/
void Dynamixel::ledOn(byte bID, byte option)
{
    this->writeByte(bID, 25, option);
}

void Dynamixel::ledOff(byte bID){
    this->writeByte(bID, 25, 0);
}

void Dynamixel::setPID(byte bID, byte propotional, byte integral, byte derivative)
{
    this->writeByte(bID, 27, derivative);
    this->writeByte(bID, 28, integral);
    this->writeByte(bID, 29, propotional);
}

void Dynamixel::cwTurn(byte bID, word speed)
{
    //check if servo is in wheelmode
    word mode=0;
    mode = this->readByte(bID, 11);
    
    //if not set it in wheelmode
    if(mode != 1) this->writeByte(bID, 11, 1);
    
    this->writeWord(bID, 32, speed+1023);
}

void Dynamixel::ccwTurn(byte bID, word speed)
{
    //check if servo is in wheelmode
    word mode=0;
    mode = this->readByte(bID, 11);
    
    //if not set it in wheelmode
    if(mode != 1) this->writeByte(bID, 11, 1);
    
    this->writeWord(bID, 32, speed);
}

/*
 * @brief initialize parameter and get ID, instruction for making packet
 * */
void Dynamixel::initPacket(byte bID, byte bInst)
{
    mbLengthForPacketMaking = 0;
    mbIDForPacketMaking = bID;
    mbInstructionForPacketMaking = bInst;
    mCommStatus = 0;
}

/*
 * @brief just push parameters, individual ID or moving data, individual data length
 * */
void Dynamixel::pushByte(byte value)
{
    //packet length is not above the maximum 143 bytes because size of buffer receiver has only 143 bytes capacity.
    //please refer to ROBOTIS e-manual (support.robotis.com)
    if(mbLengthForPacketMaking > 140)//prevent violation of memory access
        return;
    mParamBuffer[mbLengthForPacketMaking++] = value;
}

void Dynamixel::pushParam(byte value)
{
    if(mbLengthForPacketMaking > 140)//prevent violation of memory access
        return;
    mParamBuffer[mbLengthForPacketMaking++] = value;
}

void Dynamixel::pushParam(int value)
{
    if(mbLengthForPacketMaking > 140)//prevent violation of memory access
        return;
    mParamBuffer[mbLengthForPacketMaking++] = (unsigned char)DXL_LOBYTE(value);
    mParamBuffer[mbLengthForPacketMaking++] = (unsigned char)DXL_HIBYTE(value);
}

/*
 * @brief transfers packets to dynamixel bus
 * */
void Dynamixel::flushPacket(void){
    return mDxlUsart->flush();
}
