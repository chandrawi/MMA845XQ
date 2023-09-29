#include <MMA845xQ.h>

// Constructor

MMA845xQ::MMA845xQ(TwoWire &wireObject)
{
    _deviceAddress = MMA845XQ_I2C_ADDR;
    _wire = &wireObject;
}

MMA845xQ::MMA845xQ(uint8_t deviceAddress, TwoWire &wireObject)
{
    _deviceAddress = deviceAddress;
    _wire = &wireObject;
}

// Public

bool MMA845xQ::begin()
{
    _wire->begin();
    
    // Check device ID
    _deviceId = _readByte(MMA845XQ_REG_WHO_AM_I);
    if (_deviceId == MMA8451Q_ID) _dataBits = 14;      // Set read data bits to 14 for MMA8451Q
    else if (_deviceId == MMA8452Q_ID) _dataBits = 12; // Set read data bits to 12 for MMA8452Q
    else if (_deviceId == MMA8453Q_ID) _dataBits = 10; // Set read data bits to 10 for MMA8453Q
    else return false;
    
    // Reset MMA845XQ
    _writeByte(MMA845XQ_REG_CTRL_REG2, 0b01000000);
    while (_readByte(MMA845XQ_REG_CTRL_REG2) == 0b01000000) yield();
    
    // Set MOD to high resolution and SMOD to normal
    setOverSamplingMode(MMA845XQ_OVER_SAMPLING_HIGH_RESOLUTION, MMA845XQ_OVER_SAMPLING_NORMAL);
    // Set accelerometer range +/- 2g
    setRange(MMA845XQ_RANGE_2);
    // Set acceleration unit to g
    _unit = MMA845XQ_UNIT_G;
    
    // Set to active mode
    active();
    
    return true;
}

bool MMA845xQ::begin(uint32_t clock)
{
    if (!begin()) return false;
    _wire->setClock(clock);
    return true;
}

uint8_t MMA845xQ::deviceId()
{
    return _deviceId;
}

void MMA845xQ::standby()
{
    _writeBit(MMA845XQ_REG_CTRL_REG1, 0b00000000, 0); // Switch to standby mode (bit0 to 0)
}

void MMA845xQ::active()
{
    _writeBit(MMA845XQ_REG_CTRL_REG1, 0b00000001, 0); // Switch to active mode (bit0 to 1)
}

uint8_t MMA845xQ::getMode()
{
    return _readBits(MMA845XQ_REG_SYSMOD, 0, 2);
}

void MMA845xQ::setReadMode(uint8_t readMode)
{
    setFifoEnable(false);
    standby();
    uint8_t bitData = 0b00000000;
    if (_deviceId == MMA8451Q_ID) _dataBits = 14;      // Set read data bits to 14 for MMA8451Q
    else if (_deviceId == MMA8452Q_ID) _dataBits = 12; // Set read data bits to 12 for MMA8452Q
    else if (_deviceId == MMA8453Q_ID) _dataBits = 10; // Set read data bits to 10 for MMA8453Q
    if (readMode == MMA845XQ_READ_FAST){
        bitData = 0b00000010;
        _dataBits = 8; // Set number of read data bit to 8
    }
    _writeBit(MMA845XQ_REG_CTRL_REG1, bitData, 1); // Set F_READ bit
    active();
    setFifoEnable(true);
}

void MMA845xQ::_readData()
{
    uint8_t status = 0x00;
    _readBytes(MMA845XQ_REG_STATUS, &status, 1); // Check for new data
    if (!status) return; // Not updating _dataX,Y,Z when no new data
    _dataFlag = 0b00000000; // Reset flag to indicate data ready to read
    
    if (_dataBits == 8) _readDataFast(); // Read acceleration data from register for fast mode
    else _readDataNormal();              // Read acceleration data from register for normal mode
}

void MMA845xQ::_readDataFast()
{
    uint8_t data[3];
    _readBytes(MMA845XQ_REG_OUT_X_MSB, data, 3); // Read MSB acceleration X, Y, Z data
    // Update cached acceleration X, Y, and Z data from data register
    _dataX = ((int16_t) data[0]);
    _dataY = ((int16_t) data[1]);
    _dataZ = ((int16_t) data[2]);
    if (data[0] > 0x7F) _dataX |= 0xFF00; // Set 8 bits MSB to 1 if data is negative
    if (data[1] > 0x7F) _dataY |= 0xFF00;
    if (data[2] > 0x7F) _dataZ |= 0xFF00;
}

void MMA845xQ::_readDataNormal()
{
    uint8_t data[6];
    _readBytes(MMA845XQ_REG_OUT_X_MSB, data, 6); // Read all acceleration data
    uint8_t shiftBits = 16 - _dataBits;
    // Update cached acceleration X, Y, and Z data from data register
    _dataX = ((((int16_t) data[0]) << 8) | data[1]) >> shiftBits; // Shift 14/12/10-bit 2'complement to 16-bit integer
    _dataY = ((((int16_t) data[2]) << 8) | data[3]) >> shiftBits;
    _dataZ = ((((int16_t) data[4]) << 8) | data[5]) >> shiftBits;
    uint16_t maskBits = 0xFFFF << _dataBits;
    if (data[0] > 0x7F) _dataX |= maskBits; // Set (16 - n) bits MSB to 1 if data is negative
    if (data[2] > 0x7F) _dataY |= maskBits;
    if (data[4] > 0x7F) _dataZ |= maskBits;
}

void MMA845xQ::read(int16_t* rawX, int16_t* rawY, int16_t* rawZ)
{
    _readData(); // Read data from sensor
    _dataFlag = 0b00000111; // Mark the flag to indicate all data has been read
    *rawX = _dataX;
    *rawY = _dataY;
    *rawZ = _dataZ;
}

int16_t MMA845xQ::readX()
{
    // Read data from sensor when all previous data has been read or acceleration-X data requested consecutively
    if (_dataFlag & 0b00000001) _readData();
    _dataFlag |= 0b00000001; // Mark acceleration-X data has been read
    return _dataX;
}

int16_t MMA845xQ::readY()
{
    // Read data from sensor when all previous data has been read or acceleration-Y data requested consecutively
    if (_dataFlag & 0b00000010) _readData();
    _dataFlag |= 0b00000010; // Mark acceleration-Y data has been read
    return _dataY;
}

int16_t MMA845xQ::readZ()
{
    // Read data from sensor when all previous data has been read or acceleration-Z data requested consecutively
    if (_dataFlag & 0b00000100) _readData();
    _dataFlag |= 0b00000100; // Mark acceleration-Z data has been read
    return _dataZ;
}

void MMA845xQ::readNew(int16_t* rawX, int16_t* rawY, int16_t* rawZ)
{
    uint8_t status = 0x00;
    while (status == 0x00) _readBytes(MMA845XQ_REG_STATUS, &status, 1); // Wait for new data
    _readData(); // Read data from sensor
    *rawX = _dataX;
    *rawY = _dataY;
    *rawZ = _dataZ;
}

void MMA845xQ::readInst(int16_t* rawX, int16_t* rawY, int16_t* rawZ)
{
    if (_dataBits == 8) _readDataFast(); // Read acceleration data from register for fast mode
    else _readDataNormal();              // Read acceleration data from register for normal mode
    *rawX = _dataX;
    *rawY = _dataY;
    *rawZ = _dataZ;
}

int16_t MMA845xQ::_readInst(uint8_t registry, bool mode)
{
    int16_t dataInst;
    if (mode) {
        uint8_t data;
        _readBytes(registry, &data, 1); // Read an acceleration data
        dataInst = data;
        if (data > 0x7F) dataInst |= 0xFF00; // Set 8 bits MSB to 1 if data is negative
    } else {
        uint8_t data[2];
        _readBytes(registry, data, 2); // Read an acceleration data
        uint8_t shiftBits = 16 - _dataBits;
        dataInst = ((((int16_t) data[0]) << 8) | data[1]) >> shiftBits; // Shift 14/12/10-bit 2'complement to 16-bit integer
        uint8_t maskBits = 0xFFFF << _dataBits;
        if (data[0] > 0x7F) dataInst |= 0xFF00; // Set 8 bits MSB to 1 if data is negative
    }
    return dataInst;
}

int16_t MMA845xQ::readInstX()
{
    if (_dataBits == 8) return _readInst(MMA845XQ_REG_OUT_X_FAST, true); // Read acceleration data for fast mode
    else return _readInst(MMA845XQ_REG_OUT_X_MSB, false);                // Read acceleration data for normal mode
}

int16_t MMA845xQ::readInstY()
{
    if (_dataBits == 8) return _readInst(MMA845XQ_REG_OUT_Y_FAST, true); // Read acceleration data for fast mode
    else return _readInst(MMA845XQ_REG_OUT_Y_MSB, false);                // Read acceleration data for normal mode
}

int16_t MMA845xQ::readInstZ()
{
    if (_dataBits == 8) return _readInst(MMA845XQ_REG_OUT_Z_FAST, true); // Read acceleration data for fast mode
    else return _readInst(MMA845XQ_REG_OUT_Z_MSB, false);                // Read acceleration data for normal mode
}

void MMA845xQ::acceleration(double* accelX, double* accelY, double* accelZ)
{
    _readData(); // Read data from sensor
    _dataFlag = 0b00000111; // Mark the flag to indicate all data has been read
    *accelX = _dataX * _unit / _range;
    *accelY = _dataY * _unit / _range;
    *accelZ = _dataZ * _unit / _range;
}

double MMA845xQ::accelerationX()
{
    return readX() * _unit / _range;
}

double MMA845xQ::accelerationY()
{
    return readY() * _unit / _range;
}

double MMA845xQ::accelerationZ()
{
    return readZ() * _unit / _range;
}

void MMA845xQ::accelerationNew(double* accelX, double* accelY, double* accelZ)
{
    uint8_t status = 0x00;
    while (status == 0x00) _readBytes(MMA845XQ_REG_STATUS, &status, 1); // Wait for new data
    _readData(); // Read data from sensor
    *accelX = _dataX * _unit / _range;
    *accelY = _dataY * _unit / _range;
    *accelZ = _dataZ * _unit / _range;
}

uint8_t MMA845xQ::getRange()
{
    return _readBits(MMA845XQ_REG_XYZ_DATA_CFG, 0, 2);
}

void MMA845xQ::setRange(uint8_t range)
{
    if (range > 0b00000010) return; // Data valid 0b00000000 - 0b00000010
    
    standby();
    _writeBits(MMA845XQ_REG_XYZ_DATA_CFG, range, 0, 2); // Set accelerometer range
    if (range < 0b00000010) _writeBit(MMA845XQ_REG_CTRL_REG1, 0b00000100, 2); // Set low noise mode for range +/- 2g and +/- 4g
    else _writeBit(MMA845XQ_REG_CTRL_REG1, 0b00000000, 2);
    active();
    
    // Set 1 g data reading raw data value
    switch (range){
        case MMA845XQ_RANGE_2: _range = 4096; break;
        case MMA845XQ_RANGE_4: _range = 2048; break;
        case MMA845XQ_RANGE_8: _range = 1024; break;
    }
    if (_dataBits == 8) _range = _range >> 6; 
    else if (_deviceId == MMA8452Q_ID) _range = _range >> 2;
    else if (_deviceId == MMA8453Q_ID) _range = _range >> 4;
    
    // Read data for the first time
    readNew(&_dataX, &_dataY, &_dataZ);
}

void MMA845xQ::setHpfEnable(bool enable)
{
    standby();
    uint8_t xyz_data_cfg = 0b00000000;
    if (enable) xyz_data_cfg = 0b00010000;
    _writeBit(MMA845XQ_REG_XYZ_DATA_CFG, xyz_data_cfg, 4); // Enable or disable high pass filter in bit4
    active();
}

void MMA845xQ::setHpf(uint8_t select)
{
    standby();
    if (select > 3) return;
    _writeBits(MMA845XQ_REG_HP_FILTER_CUTOFF, select, 0, 2); // Enable or disable high pass filter in bit4
    active();
}

double MMA845xQ::getUnit()
{
    return _unit;
}

void MMA845xQ::setUnit(double unit)
{
    _unit = unit;
}

int8_t MMA845xQ::_dataToOffset(int16_t data)
{
    if (_range > 512){
        uint8_t divider = _range / 512;
        int8_t offset = data / divider;
        if (data >= 0){
            if ((data % divider) > (divider / 2)) offset++;
        }
        else {
            if (((0 - data) % divider) > (divider / 2)) offset--;
        }
        return -offset;
    }
    else {
        int16_t offset = data * (512 / _range);
        return (int8_t) -offset;
    }
}

int16_t MMA845xQ::_offsetToData(int8_t offset)
{
    if (_range > 512){
        return -offset * (_range / 512);
    }
    else {
        uint8_t divider = 512 / _range;
        int16_t data = offset / divider;
        if (offset >= 0){
            if ((offset % divider) > (divider / 2)) data++;
        }
        else {
            if (((0 - offset) % divider) > (divider / 2)) data--;
        }
        return -data;
    }
}

int16_t MMA845xQ::getOffsetX()
{
    uint8_t offset = _readByte(MMA845XQ_REG_OFF_X);
    return _offsetToData(offset);
}

int16_t MMA845xQ::getOffsetY()
{
    int8_t offset = _readByte(MMA845XQ_REG_OFF_Y);
    return _offsetToData(offset);
}

int16_t MMA845xQ::getOffsetZ()
{
    int8_t offset = _readByte(MMA845XQ_REG_OFF_Z);
    return _offsetToData(offset);
}

void MMA845xQ::setOffset(int16_t offsetX, int16_t offsetY, int16_t offsetZ)
{
    setOffsetX(offsetX);
    setOffsetY(offsetY);
    setOffsetZ(offsetZ);
}

void MMA845xQ::setOffsetX(int16_t offsetX)
{
    uint8_t offset = _dataToOffset(offsetX);
    standby();
    _writeByte(MMA845XQ_REG_OFF_X, offset);
    active();
}

void MMA845xQ::setOffsetY(int16_t offsetY)
{
    uint8_t offset = _dataToOffset(offsetY);
    standby();
    _writeByte(MMA845XQ_REG_OFF_Y, offset);
    active();
}

void MMA845xQ::setOffsetZ(int16_t offsetZ)
{
    uint8_t offset = _dataToOffset(offsetZ);
    standby();
    _writeByte(MMA845XQ_REG_OFF_Z, offset);
    active();
}

void MMA845xQ::setSleepEnable(bool enable)
{
    standby();
    uint8_t ctrl_reg2 = 0b00000000;
    if (enable) ctrl_reg2 = 0b00000100;
    _writeBit(MMA845XQ_REG_CTRL_REG2, 0b00000100, 2); // Switch to auto sleep enable or disable (bit2)
    active();
}

uint8_t MMA845xQ::getOverSamplingWakeMode()
{
    return _readBits(MMA845XQ_REG_CTRL_REG2, 0, 2); // Select bit0 - bit1
}

uint8_t MMA845xQ::getOverSamplingSleepMode()
{
    return _readBits(MMA845XQ_REG_CTRL_REG2, 3, 2) >> 3; // Select bit3 - bit4 and shift bit
}

void MMA845xQ::setOverSamplingMode(uint8_t wakeMode, uint8_t sleepMode)
{
    if (wakeMode > 0b00000011) return; // Data valid 0b00000000 - 0b00000011
    if (sleepMode > 0b00000011) return;
    sleepMode = sleepMode << 3; // Shift to match bit position in registry
    
    standby();
    _writeBits(MMA845XQ_REG_CTRL_REG2, wakeMode, 0, 2); // Set wake oversampling mode in bit0 - bit1
    _writeBits(MMA845XQ_REG_CTRL_REG2, sleepMode, 3, 2); // Set sleep oversampling mode in bit3 - bit4
    active();
}

uint8_t MMA845xQ::getDataRateWake()
{
    return _readBits(MMA845XQ_REG_CTRL_REG1, 3, 3) >> 3; // Select bit3 - bit5
}

uint8_t MMA845xQ::getDataRateSleep()
{
    return _readBits(MMA845XQ_REG_CTRL_REG1, 6, 2) >> 6; // Select bit6 - bit7 and shift bit
}

void MMA845xQ::setDataRate(uint8_t dataRateWake, uint8_t dataRateSleep)
{
    if (dataRateWake > 0b00000111) return; // Data valid 0b00000000 - 0b00000111
    if (dataRateSleep > 0b00000111) return;
    dataRateWake = dataRateWake << 3; // Shift to match bit position in registry
    dataRateSleep = dataRateSleep << 6;
    
    standby();
    _writeBits(MMA845XQ_REG_CTRL_REG1, dataRateWake, 3, 3); // Set wake data rate in bit3 - bit5
    _writeBits(MMA845XQ_REG_CTRL_REG1, dataRateSleep, 6, 2); // Set sleep data rate in bit6 - bit7
    active();
}

uint8_t MMA845xQ::getWakeTimeMin()
{
    return _readByte(MMA845XQ_REG_ASLP_COUNT);
}

void MMA845xQ::setWakeTimeMin(uint8_t wakeTimeMin)
{
    standby();
    _writeByte(MMA845XQ_REG_ASLP_COUNT, wakeTimeMin);
    active();
}

uint8_t MMA845xQ::getFifoMode()
{
    return _fifoMode;
}

uint8_t MMA845xQ::getFifoWaterMark()
{
    return _readBits(MMA845XQ_REG_F_SETUP, 0, 6);
}

uint8_t MMA845xQ::getFifoCount()
{
    uint8_t status;
    _readBytes(MMA845XQ_REG_STATUS, &status, 1);
    return status & 0b00111111;
}

void MMA845xQ::setFifoEnable(bool enable)
{
    uint8_t mode = _fifoMode;
    if (enable){
        _writeBits(MMA845XQ_REG_F_SETUP, mode, 6, 2); // Enable FIFO with predefined mode
    }
    else {
        _writeBits(MMA845XQ_REG_F_SETUP, 0b00000000, 6, 2); // Disable FIFO
    }
}

void MMA845xQ::setFifo(uint8_t fifoMode, uint8_t fifoWaterMark)
{
    if (fifoWaterMark > 32) return; // Valid FIFO watermark 0-32
    _fifoMode = fifoMode; // Define FIFO mode
    standby();
    _writeBits(MMA845XQ_REG_F_SETUP, fifoWaterMark, 0, 6); // Set watermark
    if(fifoMode > 0b11000000) _writeBits(MMA845XQ_REG_TRIG_CFG, fifoMode, 2, 4); // Set fifo trigger source
    active();
}

uint8_t MMA845xQ::getOrientationThreshold()
{
    return _readBits(MMA845XQ_REG_PL_THS_REG, 3, 5);
}

uint8_t MMA845xQ::getOrientationHysterisis()
{
    return _readBits(MMA845XQ_REG_PL_THS_REG, 0, 3);
}

uint8_t MMA845xQ::getOrientationBackFront()
{
    return _readBits(MMA845XQ_REG_PL_BF_ZCOMP, 6, 2);
}

uint8_t MMA845xQ::getOrientationZlock()
{
    return _readBits(MMA845XQ_REG_PL_BF_ZCOMP, 0, 3);
}

uint8_t MMA845xQ::getOrientationCounter()
{
    return _readByte(MMA845XQ_REG_PL_COUNT);
}

void MMA845xQ::setOrientationEnable(bool enable)
{
    standby();
    uint8_t bitData = 0b00000000;
    if (enable) bitData = 0b01000000;
    _writeBit(MMA845XQ_REG_PL_CFG, bitData, 6); // Enable orientation detection
    active();
}

void MMA845xQ::setOrientation(uint8_t threshold, uint8_t hysterisis, uint8_t backFront, uint8_t zLock, uint8_t counter, bool debounceMode)
{
    if (threshold > 9) return;
    if (hysterisis > 7) return;
    if (backFront > 3) return;
    if (zLock > 7) return;
    
    standby();
    _writeBits(MMA845XQ_REG_PL_THS_REG, (threshold << 3), 3, 5);  // Set portrait/landscape angle threshold
    _writeBits(MMA845XQ_REG_PL_THS_REG, hysterisis, 0, 3);        // Set portrait/landscape angle hysterisis
    _writeBits(MMA845XQ_REG_PL_BF_ZCOMP, (backFront << 6), 6, 2); // Set back/front angle threshold
    _writeBits(MMA845XQ_REG_PL_BF_ZCOMP, zLock, 0, 3);            // Set z-lock angle threshold
    _writeByte(MMA845XQ_REG_PL_COUNT, counter);                   // Set debounce counter for orientation detection
    uint8_t bitData = 0b00000000;
    if (debounceMode) bitData = 0b10000000;
    _writeBit(MMA845XQ_REG_PL_CFG, bitData, 7);                   // Set debounce counter mode to increment/decrement or clear
    active();
}

bool MMA845xQ::checkOrientation()
{
    _checkOrientation = _readByte(MMA845XQ_REG_PL_STATUS);
    
    return _checkOrientation & 0b10000000;
}

uint8_t MMA845xQ::orientationStatus()
{
    return _checkOrientation & 0b00000111;
}

bool MMA845xQ::orientationZlock()
{
    return _checkOrientation & 0b01000000;
}

uint8_t MMA845xQ::getMotionMode()
{
    uint8_t oae = _readBit(MMA845XQ_REG_FF_MT_CFG, 6); // Select bit6
    uint8_t ele = _readBit(MMA845XQ_REG_FF_MT_CFG, 7); // Select bit7
    if (_readBit(MMA845XQ_REG_FF_MT_THS, 7)) return oae | ele | 0b00100000;
    return oae | ele;
}

uint8_t MMA845xQ::getMotionAxis()
{
    return _readBits(MMA845XQ_REG_FF_MT_CFG, 3, 3); // Select bit3 - bit5
}

uint8_t MMA845xQ::getMotionThreshold()
{
    return _readBits(MMA845XQ_REG_FF_MT_THS, 0, 6); // Select bit0 - bit6
}

uint8_t MMA845xQ::getMotionCounter()
{
    return _readByte(MMA845XQ_REG_FF_MT_COUNT);
}

void MMA845xQ::setMotionEnable(bool enable)
{
    standby();
    uint8_t ff_mt_cfg = 0b00000000;
    if (enable) ff_mt_cfg = _axisMotion;
    _writeBits(MMA845XQ_REG_FF_MT_CFG, ff_mt_cfg, 3, 3); // Enable or disable free fall / motion event detection with predefined axis
    active();
}

void MMA845xQ::setMotion(uint8_t mode, uint8_t axis, uint8_t threshold, uint8_t counter, bool debounceMode)
{
    if (axis > 7) return;
    standby();
    _writeBit(MMA845XQ_REG_FF_MT_CFG, mode, 6);          // Set free fall or motion event detection mode
    _writeBit(MMA845XQ_REG_FF_MT_CFG, mode, 7);          // Set event to latch or not latch
    _axisMotion = axis << 3;                             // Define axis for free fall / motion event detection
    _writeBits(MMA845XQ_REG_FF_MT_THS, threshold, 0, 6); // Set motion detection threshold [0,0625 LSB/g]
    _writeByte(MMA845XQ_REG_FF_MT_COUNT, counter);       // Set debounce counter for motion detection
    uint8_t bitData = 0b00000000;
    if (debounceMode) bitData = 0b10000000;
    _writeBit(MMA845XQ_REG_FF_MT_THS, bitData, 7);       // Set debounce counter mode to increment/decrement or clear
    active();
}

bool MMA845xQ::checkMotion()
{
    _checkMotion = _readByte(MMA845XQ_REG_FF_MT_SRC);

    return _checkMotion & 0b10000000; // Select event flag in bit7
}

int8_t MMA845xQ::motionAxisX()
{
    if (_checkMotion & 0b00000010){ // Select X-axis motion detection flag
        if (_checkMotion & 0b00000001) return -1; // Asign value -1 if polarity negative
        else return 1;
    }
    return 0;
}

int8_t MMA845xQ::motionAxisY()
{
    if (_checkMotion & 0b00001000){ // Select Y-axis motion detection flag
        if (_checkMotion & 0b00000100) return -1; // Asign value -1 if polarity negative
        else return 1;
    }
    return 0;
}

int8_t MMA845xQ::motionAxisZ()
{
    if (_checkMotion & 0b00100000){ // Select Y-axis motion detection flag
        if (_checkMotion & 0b00010000) return -1; // Asign value -1 if polarity negative
        else return 1;
    }
    return 0;
}

uint8_t MMA845xQ::getTransientMode()
{
    uint8_t hpf = _readBit(MMA845XQ_REG_TRANSIENT_CFG, 0); // Select bit0
    uint8_t ele = _readBit(MMA845XQ_REG_TRANSIENT_CFG, 4); // Select bit4
    if (_readBit(MMA845XQ_REG_TRANSIENT_THS, 7)) return hpf | ele | 0b00100000;
    return hpf | ele;
}

uint8_t MMA845xQ::getTransientAxis()
{
    return _readBits(MMA845XQ_REG_TRANSIENT_CFG, 1, 3); // Select bit1 - bit3
}

uint8_t MMA845xQ::getTransientThreshold()
{
    return _readBits(MMA845XQ_REG_TRANSIENT_THS, 0, 6); // Select bit0 - bit6
}

uint8_t MMA845xQ::getTransientCounter()
{
    return _readByte(MMA845XQ_REG_TRANSIENT_COUNT);
}

void MMA845xQ::setTransientEnable(bool enable)
{
    standby();
    uint8_t transient_cfg = 0b00000000;
    if (enable) transient_cfg = _axisTransient;
    _writeBits(MMA845XQ_REG_TRANSIENT_CFG, transient_cfg, 1, 3); // Enable or disable transient event detection with predefined axis
    active();
}

void MMA845xQ::setTransient(uint8_t mode, uint8_t axis, uint8_t threshold, uint8_t counter, bool debounceMode)
{
    if (axis > 7) return;
    standby();
    _writeBit(MMA845XQ_REG_TRANSIENT_CFG, mode, 0);          // Set transient event detection mode
    _writeBit(MMA845XQ_REG_TRANSIENT_CFG, mode, 4);          // Set event to latch or not latch
    _axisTransient = axis << 1;                              // Define axis for transient event detection
    _writeBits(MMA845XQ_REG_TRANSIENT_THS, threshold, 0, 6); // Set transient detection threshold [0,0625 LSB/g]
    _writeByte(MMA845XQ_REG_TRANSIENT_COUNT, counter);       // Set debounce counter for transient detection
    uint8_t bitData = 0b00000000;
    if (debounceMode) bitData = 0b10000000;
    _writeBit(MMA845XQ_REG_TRANSIENT_THS, bitData, 7);       // Set debounce counter mode to increment/decrement or clear
    active();
}

bool MMA845xQ::checkTransient()
{
    _checkTransient = _readByte(MMA845XQ_REG_TRANSIENT_SRC);

    return _checkTransient & 0b01000000; // Select event flag in bit6
}

int8_t MMA845xQ::transientAxisX()
{
    if (_checkTransient & 0b00000010){ // Select X-axis transient detection flag
        if (_checkTransient & 0b00000001) return -1; // Asign value -1 if polarity negative
        else return 1;
    }
    return 0;
}

int8_t MMA845xQ::transientAxisY()
{
    if (_checkTransient & 0b00001000){ // Select Y-axis transient detection flag
        if (_checkTransient & 0b00000100) return -1; // Asign value -1 if polarity negative
        else return 1;
    }
    return 0;
}

int8_t MMA845xQ::transientAxisZ()
{
    if (_checkTransient & 0b00100000){ // Select Y-axis transient detection flag
        if (_checkTransient & 0b00010000) return -1; // Asign value -1 if polarity negative
        else return 1;
    }
    return 0;
}

uint8_t MMA845xQ::getPulseMode()
{
    return _readByte(MMA845XQ_REG_PULSE_CFG);
}

uint8_t MMA845xQ::getPulseThresholdX()
{
    return _readByte(MMA845XQ_REG_PULSE_THSX);
}

uint8_t MMA845xQ::getPulseThresholdY()
{
    return _readByte(MMA845XQ_REG_PULSE_THSY);
}

uint8_t MMA845xQ::getPulseThresholdZ()
{
    return _readByte(MMA845XQ_REG_PULSE_THSZ);
}

uint8_t MMA845xQ::getPulseTimeLimit()
{
    return _readByte(MMA845XQ_REG_PULSE_TMLT);
}

uint8_t MMA845xQ::getPulseTimeLatency()
{
    return _readByte(MMA845XQ_REG_PULSE_LTCY);
}

uint8_t MMA845xQ::getPulseTimeWindow()
{
    return _readByte(MMA845XQ_REG_PULSE_WIND);
}

void MMA845xQ::setPulseEnable(bool enable)
{
    standby();
    uint8_t pulse_cfg = 0b00000000;
    if (enable) pulse_cfg = _axisPulse;
    _writeBits(MMA845XQ_REG_PULSE_CFG, pulse_cfg, 0, 6); // Enable or disable pulse single, double, or both event detection with predefined axis
    active();
}

void MMA845xQ::setPulse(uint8_t mode, uint8_t axis, uint8_t thresholdX, uint8_t thresholdY, uint8_t thresholdZ, uint8_t timeLimit, uint8_t timeLatency, uint8_t timeWindow)
{
    standby();
    _writeBits(MMA845XQ_REG_PULSE_CFG, mode, 6, 2); // Set latch and DPA bit
    _axisPulse = mode;
    if (!(axis & MMA845XQ_AXIS_X)) _axisPulse &= 0b11111100;  // Clear X-axis bit for single and double pulse if X-axis not used
    if (!(axis & MMA845XQ_AXIS_Y)) _axisPulse &= 0b11110011;  // Clear Y-axis bit for single and double pulse if Y-axis not used
    if (!(axis & MMA845XQ_AXIS_Z)) _axisPulse &= 0b11001111;  // Clear Z-axis bit for single and double pulse if Z-axis not used
    _writeByte(MMA845XQ_REG_PULSE_THSX, thresholdX);   // Set X-axis threshold for pulse detection
    _writeByte(MMA845XQ_REG_PULSE_THSY, thresholdY);   // Set Y-axis threshold for pulse detection
    _writeByte(MMA845XQ_REG_PULSE_THSZ, thresholdZ);   // Set Z-axis threshold for pulse detection
    _writeByte(MMA845XQ_REG_PULSE_TMLT, timeLimit);    // Set time limit for pulse detection
    _writeByte(MMA845XQ_REG_PULSE_LTCY, timeLimit);    // Set time latency for pulse detection
    _writeByte(MMA845XQ_REG_PULSE_WIND, timeLimit);    // Set time window for pulse detection
    active();
}

bool MMA845xQ::checkPulse()
{
    _checkPulse = _readByte(MMA845XQ_REG_PULSE_SRC);

    return _checkPulse & 0b10000000; // Select event flag in bit7
}

int8_t MMA845xQ::pulseAxisX()
{
    uint8_t singleDouble = 1; // Single pulse detected
    if (_checkPulse & 0b00001000) singleDouble = 2; // Double pulse detected
    if (_checkPulse & 0b00010000){ // Select X-axis pulse detection flag
        if (_checkPulse & 0b00000001) return 0 - singleDouble; // Asign value -1 if polarity negative
        else return singleDouble;
    }
    return 0;
}

int8_t MMA845xQ::pulseAxisY()
{
    uint8_t singleDouble = 1; // Single pulse detected
    if (_checkPulse & 0b00001000) singleDouble = 2; // Double pulse detected
    if (_checkPulse & 0b00100000){ // Select Y-axis pulse detection flag
        if (_checkPulse & 0b00000010) return 0 - singleDouble; // Asign value -1 if polarity negative
        else return singleDouble;
    }
    return 0;
}

int8_t MMA845xQ::pulseAxisZ()
{
    uint8_t singleDouble = 1; // Single pulse detected
    if (_checkPulse & 0b00001000) singleDouble = 2; // Double pulse detected
    if (_checkPulse & 0b01000000){ // Select Y-axis pulse detection flag
        if (_checkPulse & 0b00000100) return 0 - singleDouble; // Asign value -1 if polarity negative
        else return singleDouble;
    }
    return 0;
}

void MMA845xQ::requestInterrupt(uint8_t interruptPin, uint8_t interruptEvent, uint8_t interruptMode)
{
    requestInterrupt(interruptPin, interruptEvent);
    setInterruptMode(interruptMode);
}

void MMA845xQ::requestInterrupt(uint8_t interruptPin, uint8_t interruptEvent)
{
    standby();
    uint8_t bitPosition = interruptEvent >> 5;
    uint8_t bitData = 0b00000000; // interrupt pin is INT2
    if (interruptPin == MMA845XQ_INT_PIN_1) bitData = 0b11111111; // interrupt pin is INT1
    _writeBit(MMA845XQ_REG_CTRL_REG4, 0b11111111, bitPosition); // Enable an interrupt event
    _writeBit(MMA845XQ_REG_CTRL_REG5, bitData, bitPosition); // Set interrupt pin to INT1 or INT2
    
    bitPosition = (interruptEvent >> 2) & 0b00000111;
    if (interruptEvent & 0b00000010) bitData = 0b11111111; // interrupt event is waking device
    else bitData = 0b00000000; // interrupt event is not waking device
    _writeBit(MMA845XQ_REG_CTRL_REG3, bitData, bitPosition); // Set an interrupt event to wake or don't wake device
    active();
}

void MMA845xQ::setInterruptMode(uint8_t interruptMode)
{
    standby();
    if (interruptMode > 0b00000011) return; // Data valid 0b00000000 - 0b00000011
    _writeBits(MMA845XQ_REG_CTRL_REG3, interruptMode, 0, 2); // Set interrupt mode in bit0 - bit1
    active();
}

void MMA845xQ::resetInterrupt(uint8_t interruptPinEvent)
{
    standby();
    if (interruptPinEvent < 0b00000010){
        uint8_t ctrl_reg5 = _readByte(MMA845XQ_REG_CTRL_REG5);
        uint8_t pinBit;
        for (uint8_t i=0; i<8; i++){
            pinBit = (ctrl_reg5 >> i) & 0b00000001;
            if (pinBit == interruptPinEvent){ // Check interrupt event coresponding with interrupt pin
                _writeBit(MMA845XQ_REG_CTRL_REG4, 0b00000000, i); // Disable an interrupt event
            }
        }
    }
    else {
        uint8_t bitPosition = interruptPinEvent >> 5;
        _writeBit(MMA845XQ_REG_CTRL_REG4, 0b00000000, bitPosition); // Disable an interrupt event
        bitPosition = (interruptPinEvent >> 2) & 0b00000111;
        if (interruptPinEvent & 0b00000010) _writeBit(MMA845XQ_REG_CTRL_REG3, 0b00000000, bitPosition); // Set an interrupt event to don't wake device
    }
    active();
}

void MMA845xQ::resetInterrupt()
{
    standby();
    _writeByte(MMA845XQ_REG_CTRL_REG3, 0b00000000); // Reset interrupt mode and interrupt wake config
    _writeByte(MMA845XQ_REG_CTRL_REG4, 0b00000000); // Reset interrupt enable
    _writeByte(MMA845XQ_REG_CTRL_REG5, 0b00000000); // Reset interrupt pin
    active();
}

uint8_t MMA845xQ::checkInterrupt()
{
    _checkInterrupt = _readByte(MMA845XQ_REG_INT_SOURCE); // Checking interrupt source
    // Check fifo interrupt from data status register
    if (_readBits(MMA845XQ_REG_STATUS, 6, 2) && _readBit(MMA845XQ_REG_CTRL_REG4, 6)) _checkInterrupt |= 0b01000000;

    return _checkInterrupt;
}

bool MMA845xQ::checkInterruptDataReady()
{
    if (_checkInterrupt & 0b00000001) {               // Clear interruptDataReady flag and return true
        _checkInterrupt &= 0b11111110;
        return true;
    } else {                                          // Check interrupt register for data ready
        uint8_t flag = checkInterrupt() & 0b00000001;
        if (flag) _checkInterrupt &= 0b11111110;
        return flag;
    }
}

bool MMA845xQ::checkInterruptMotion()
{
    if (_checkInterrupt & 0b00000100) {               // Clear interruptMotion flag and return true
        _checkInterrupt &= 0b11111011;
        return true;
    } else {                                          // Check interrupt register for motion
        uint8_t flag = checkInterrupt() & 0b00000100;
        if (flag) _checkInterrupt &= 0b11111011;
        return flag;
    }
}

bool MMA845xQ::checkInterruptPulse()
{
    if (_checkInterrupt & 0b00001000) {               // Clear interruptPulse flag and return true
        _checkInterrupt &= 0b11110111;
        return true;
    } else {                                          // Check interrupt register for pulse
        uint8_t flag = checkInterrupt() & 0b00001000;
        if (flag) _checkInterrupt &= 0b11110111;
        return flag;
    }
}

bool MMA845xQ::checkInterruptOrientation()
{
    if (_checkInterrupt & 0b00010000) {               // Clear interruptOrientation flag and return true
        _checkInterrupt &= 0b11101111;
        return true;
    } else {                                          // Check interrupt register for orientation
        uint8_t flag = checkInterrupt() & 0b00010000;
        if (flag) _checkInterrupt &= 0b11101111;
        return flag;
    }
}

bool MMA845xQ::checkInterruptTransient()
{
    if (_checkInterrupt & 0b00100000) {               // Clear interruptTransient flag and return true
        _checkInterrupt &= 0b11011111;
        return true;
    } else {                                          // Check interrupt register for transient
        uint8_t flag = checkInterrupt() & 0b00100000;
        if (flag) _checkInterrupt &= 0b11011111;
        return flag;
    }
}

bool MMA845xQ::checkInterruptFifo()
{
    if (_checkInterrupt & 0b01000000) {               // Clear interruptFifo flag and return true
        _checkInterrupt &= 0b10111111;
        return true;
    } else {                                          // Check interrupt register for FIFO
        uint8_t flag = checkInterrupt() & 0b01000000;
        if (flag) _checkInterrupt &= 0b10111111;
        return flag;
    }
}

bool MMA845xQ::checkInterruptAutoSleep()
{
    if (_checkInterrupt & 0b10000000) {               // Clear interruptAutoSleep flag and return true
        _checkInterrupt &= 0b01111111;
        return true;
    } else {                                          // Check interrupt register for auto sleep
        uint8_t flag = checkInterrupt() & 0b10000000;
        if (flag) _checkInterrupt &= 0b01111111;
        return flag;
    }
}

// Private

uint8_t MMA845xQ::_readBit(uint8_t address, uint8_t startBit)
{
    uint8_t data;
    _readBytes(address, &data, 1);
    uint8_t mask = (0b00000001 << startBit);
    return data & mask;
}

uint8_t MMA845xQ::_readBits(uint8_t address, uint8_t startBit, uint8_t nBits)
{
    uint8_t data;
    _readBytes(address, &data, 1);
    uint8_t mask = (0b11111111 << startBit) & (0b11111111 >> (8 - (nBits + startBit)));
    return data & mask;
}

uint8_t MMA845xQ::_readByte(uint8_t address)
{
    uint8_t data;
    _readBytes(address, &data, 1);
    return data;
}

bool MMA845xQ::_writeBit(uint8_t address, uint8_t data, uint8_t startBit)
{
    uint8_t dataRead;
    _readBytes(address, &dataRead, 1);
    uint8_t mask = (0b00000001 << startBit);
    uint8_t dataWrite = (mask & data) | (~mask & dataRead);
    return _writeBytes(address, &dataWrite, 1);
}

bool MMA845xQ::_writeBits(uint8_t address, uint8_t data, uint8_t startBit, uint8_t nBits)
{
    uint8_t dataRead;
    _readBytes(address, &dataRead, 1);
    uint8_t mask = (0b11111111 << startBit) & (0b11111111 >> (8 - (nBits + startBit)));
    uint8_t dataWrite = (mask & data) | (~mask & dataRead);
    return _writeBytes(address, &dataWrite, 1);
}

bool MMA845xQ::_writeByte(uint8_t address, uint8_t data)
{
    return _writeBytes(address, &data, 1);
}

uint8_t MMA845xQ::_readBytes(uint8_t address, uint8_t* data, uint8_t nBytes)
{
    uint8_t i = 0;
    
    _wire->beginTransmission(_deviceAddress);
    _wire->write(address);
    _wire->endTransmission(false);
    
    _wire->requestFrom(_deviceAddress, nBytes);
    while(_wire->available()) data[i++] = _wire->read();
    
    return i;
}

bool MMA845xQ::_writeBytes(uint8_t address, uint8_t* data, uint8_t nBytes)
{
    _wire->beginTransmission(_deviceAddress);
    _wire->write(address);
    _wire->write(data, nBytes);
    int8_t stat = _wire->endTransmission();

    if (stat == 0) return true;
    return false;
}

bool MMA845xQ::isConnected()
{
    _wire->beginTransmission(_deviceAddress);
    return _wire->endTransmission() == 0;
}
