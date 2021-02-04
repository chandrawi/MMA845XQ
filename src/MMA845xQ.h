#ifndef _MMA845XQ_H_
#define _MMA845XQ_H_

#if (ARDUINO >= 100)
#include <Arduino.h>
#else
#include <WProgram.h>
#endif

#include <Wire.h>

// Device I2C address and ID
#define MMA845XQ_I2C_ADDR                           0x1C
#define MMA8451Q_ID                                 0x1A
#define MMA8452Q_ID                                 0x2A
#define MMA8453Q_ID                                 0x3A

// Mode definition
#define MMA845XQ_MODE_STANDBY                       0b00000000
#define MMA845XQ_MODE_WAKE                          0b00000001
#define MMA845XQ_MODE_SLEEP                         0b00000010

// Accelerometer range (+/-2g, +/-4g, +/-8g)
#define MMA845XQ_RANGE_2                            0b00000000
#define MMA845XQ_RANGE_4                            0b00000001
#define MMA845XQ_RANGE_8                            0b00000010

// Accelerometer unit (g, m/s^2, ft/s^2)
#define MMA845XQ_UNIT_G                             1.0
#define MMA845XQ_UNIT_M_S2                          9.81
#define MMA845XQ_UNIT_FT_S2                         386.22

// Read mode definition
#define MMA845XQ_READ_NORMAL                        0b00000000
#define MMA845XQ_READ_FAST                          0b00000010

// FIFO mode definition
#define MMA845XQ_FIFO_CIRCULAR                      0b01000000
#define MMA845XQ_FIFO_FILL                          0b10000000
#define MMA845XQ_FIFO_TRIGGER_MOTION                0b11000100
#define MMA845XQ_FIFO_TRIGGER_PULSE                 0b11001000
#define MMA845XQ_FIFO_TRIGGER_ORIENTATION           0b11010000
#define MMA845XQ_FIFO_TRIGGER_TRANSIENT             0b11100000

// Output data rate (800 Hz, 400 Hz, 200 Hz, 100 Hz, 50 Hz, 12.5 Hz, 6.25 Hz, 1.56 Hz)
#define MMA845XQ_DATA_RATE_800                      0b00000000
#define MMA845XQ_DATA_RATE_400                      0b00000001
#define MMA845XQ_DATA_RATE_200                      0b00000010
#define MMA845XQ_DATA_RATE_100                      0b00000011
#define MMA845XQ_DATA_RATE_50                       0b00000100
#define MMA845XQ_DATA_RATE_12_5                     0b00000101
#define MMA845XQ_DATA_RATE_6_25                     0b00000110
#define MMA845XQ_DATA_RATE_1_56                     0b00000111

// Sleep and wake oversampling mode
#define MMA845XQ_OVER_SAMPLING_NORMAL               0b00000000
#define MMA845XQ_OVER_SAMPLING_LOW_NOISE            0b00000001
#define MMA845XQ_OVER_SAMPLING_HIGH_RESOLUTION      0b00000010
#define MMA845XQ_OVER_SAMPLING_LOW_POWER            0b00000011

// High pass filter (HPF) selection
#define MMA845XQ_OVER_HPF_DEF                       0b00000000
#define MMA845XQ_OVER_HPF_1                         0b00000001
#define MMA845XQ_OVER_HPF_2                         0b00000010
#define MMA845XQ_OVER_HPF_3                         0b00000011

// Default orientation setting
#define MMA845XQ_ORIENTATION_DEF_THRESHOLD          0b00010000
#define MMA845XQ_ORIENTATION_DEF_HYSTERISIS         0b00000100
#define MMA845XQ_ORIENTATION_DEF_BACKFRONT          0b00000001
#define MMA845XQ_ORIENTATION_DEF_ZLOCK              0b00000100
#define MMA845XQ_ORIENTATION_DEF_COUNTER            0b00000000

// Orientation definition
#define MMA845XQ_PORTRAIT_UP_FRONT                  0b00000000
#define MMA845XQ_PORTRAIT_UP_BACK                   0b00000001
#define MMA845XQ_PORTRAIT_DOWN_FRONT                0b00000010
#define MMA845XQ_PORTRAIT_DOWN_BACK                 0b00000011
#define MMA845XQ_LANDSCAPE_RIGHT_FRONT              0b00000100
#define MMA845XQ_LANDSCAPE_RIGHT_BACK               0b00000101 
#define MMA845XQ_LANDSCAPE_LEFT_FRONT               0b00000110
#define MMA845XQ_LANDSCAPE_LEFT_BACK                0b00000111

// Axis definition
#define MMA845XQ_AXIS_X                             0b00000001
#define MMA845XQ_AXIS_Y                             0b00000010
#define MMA845XQ_AXIS_XY                            0b00000011
#define MMA845XQ_AXIS_Z                             0b00000100
#define MMA845XQ_AXIS_XZ                            0b00000101
#define MMA845XQ_AXIS_YZ                            0b00000110
#define MMA845XQ_AXIS_XYZ                           0b00000111

// Motion, transient, and pulse event modes definition
#define MMA845XQ_EVENT_FREEFALL                     0b00000000
#define MMA845XQ_EVENT_FREEFALL_LATCH               0b10000000
#define MMA845XQ_EVENT_MOTION                       0b01000000
#define MMA845XQ_EVENT_MOTION_LATCH                 0b11000000
#define MMA845XQ_EVENT_TRANSIENT_HPF                0b00000000
#define MMA845XQ_EVENT_TRANSIENT_HPF_LATCH          0b00010000
#define MMA845XQ_EVENT_TRANSIENT                    0b00000001
#define MMA845XQ_EVENT_TRANSIENT_LATCH              0b00010001
#define MMA845XQ_EVENT_PULSE_SINGLE                 0b00010101
#define MMA845XQ_EVENT_PULSE_SINGLE_LATCH           0b01010101
#define MMA845XQ_EVENT_PULSE_DOUBLE                 0b00101010
#define MMA845XQ_EVENT_PULSE_DOUBLE_SUSPEND         0b10101010
#define MMA845XQ_EVENT_PULSE_DOUBLE_LATCH           0b01101010
#define MMA845XQ_EVENT_PULSE_DOUBLE_SUSPEND_LATCH   0b11101010
#define MMA845XQ_EVENT_PULSE_BOTH                   0b00111111
#define MMA845XQ_EVENT_PULSE_BOTH_SUSPEND           0b10111111
#define MMA845XQ_EVENT_PULSE_BOTH_LATCH             0b01111111
#define MMA845XQ_EVENT_PULSE_BOTH_SUSPEND_LATCH     0b11111111

// Interrupt pin and event definition
#define MMA845XQ_INT_PIN_1                          0b00000001
#define MMA845XQ_INT_PIN_2                          0b00000000
#define MMA845XQ_INT_EVENT_DATA_READY               0b00000000
#define MMA845XQ_INT_EVENT_MOTION                   0b01000000
#define MMA845XQ_INT_EVENT_PULSE                    0b01100000
#define MMA845XQ_INT_EVENT_ORIENTATION              0b10000000
#define MMA845XQ_INT_EVENT_TRANSIENT                0b10100000
#define MMA845XQ_INT_EVENT_FIFO                     0b11000000
#define MMA845XQ_INT_EVENT_AUTO_SLEEP               0b11100000
#define MMA845XQ_INT_EVENT_MOTION_WAKE              0b01001110
#define MMA845XQ_INT_EVENT_PULSE_WAKE               0b01110010
#define MMA845XQ_INT_EVENT_ORIENTATION_WAKE         0b10010110
#define MMA845XQ_INT_EVENT_TRANSIENT_WAKE           0b10111010
#define MMA845XQ_INT_EVENT_FIFO_GATE                0b11011110

// Interrupt mode definition
#define MMA845XQ_INT_MODE_FALLING                   0b00000000
#define MMA845XQ_INT_MODE_FALLING_OD                0b00000001
#define MMA845XQ_INT_MODE_RISING                    0b00000010
#define MMA845XQ_INT_MODE_RISING_OD                 0b00000011

// Register address
#define MMA845XQ_REG_STATUS                 0x00
#define MMA845XQ_REG_OUT_X_MSB              0x01
#define MMA845XQ_REG_OUT_Y_MSB              0x03
#define MMA845XQ_REG_OUT_Z_MSB              0x05
#define MMA845XQ_REG_OUT_X_FAST             0x01
#define MMA845XQ_REG_OUT_Y_FAST             0x02
#define MMA845XQ_REG_OUT_Z_FAST             0x03
#define MMA845XQ_REG_F_SETUP                0x09
#define MMA845XQ_REG_TRIG_CFG               0x0A
#define MMA845XQ_REG_SYSMOD                 0x0B
#define MMA845XQ_REG_INT_SOURCE             0x0C
#define MMA845XQ_REG_WHO_AM_I               0x0D
#define MMA845XQ_REG_XYZ_DATA_CFG           0x0E
#define MMA845XQ_REG_HP_FILTER_CUTOFF       0x0F
#define MMA845XQ_REG_PL_STATUS              0x10
#define MMA845XQ_REG_PL_CFG                 0x11
#define MMA845XQ_REG_PL_COUNT               0x12
#define MMA845XQ_REG_PL_BF_ZCOMP            0x13
#define MMA845XQ_REG_PL_THS_REG             0x14
#define MMA845XQ_REG_FF_MT_CFG              0x15
#define MMA845XQ_REG_FF_MT_SRC              0x16
#define MMA845XQ_REG_FF_MT_THS              0x17
#define MMA845XQ_REG_FF_MT_COUNT            0x18
#define MMA845XQ_REG_TRANSIENT_CFG          0x1D
#define MMA845XQ_REG_TRANSIENT_SRC          0x1E
#define MMA845XQ_REG_TRANSIENT_THS          0x1F
#define MMA845XQ_REG_TRANSIENT_COUNT        0x20
#define MMA845XQ_REG_PULSE_CFG              0x21
#define MMA845XQ_REG_PULSE_SRC              0x22
#define MMA845XQ_REG_PULSE_THSX             0x23
#define MMA845XQ_REG_PULSE_THSY             0x24
#define MMA845XQ_REG_PULSE_THSZ             0x25
#define MMA845XQ_REG_PULSE_TMLT             0x26
#define MMA845XQ_REG_PULSE_LTCY             0x27
#define MMA845XQ_REG_PULSE_WIND             0x28
#define MMA845XQ_REG_ASLP_COUNT             0x29
#define MMA845XQ_REG_CTRL_REG1              0x2A
#define MMA845XQ_REG_CTRL_REG2              0x2B
#define MMA845XQ_REG_CTRL_REG3              0x2C
#define MMA845XQ_REG_CTRL_REG4              0x2D
#define MMA845XQ_REG_CTRL_REG5              0x2E
#define MMA845XQ_REG_OFF_X                  0x2F
#define MMA845XQ_REG_OFF_Y                  0x30
#define MMA845XQ_REG_OFF_Z                  0x31

class MMA845xQ
{
    public:

        MMA845xQ();
        MMA845xQ(uint8_t deviceAddress);

        bool begin();
        bool begin(uint32_t clock);
        uint8_t deviceId();

        void standby();
        void active();
        uint8_t getMode();
        void setReadMode(uint8_t readMode);

        void read(int16_t* rawX, int16_t* rawY, int16_t* rawZ);
        int16_t readX();
        int16_t readY();
        int16_t readZ();
        void readNew(int16_t* rawX, int16_t* rawY, int16_t* rawZ);
        void readInst(int16_t* rawX, int16_t* rawY, int16_t* rawZ);
        int16_t readInstX();
        int16_t readInstY();
        int16_t readInstZ();

        void acceleration(double* accelX, double* accelY, double* accelZ);
        double accelerationX();
        double accelerationY();
        double accelerationZ();
        void accelerationNew(double* accelX, double* accelY, double* accelZ);

        uint8_t getRange();
        void setRange(uint8_t range);
        double getUnit();
        void setUnit(double unit);
        void setHpfEnable(bool enable=true);
        void setHpf(uint8_t select);

        int16_t getOffsetX();
        int16_t getOffsetY();
        int16_t getOffsetZ();
        void setOffset(int16_t offsetX, int16_t offsetY, int16_t offsetZ);
        void setOffsetX(int16_t offsetX);
        void setOffsetY(int16_t offsetY);
        void setOffsetZ(int16_t offsetZ);

        void setSleepEnable(bool enable=true);
        uint8_t getOverSamplingWakeMode();
        uint8_t getOverSamplingSleepMode();
        void setOverSamplingMode(uint8_t wakeMode, uint8_t sleepMode);
        uint8_t getDataRateWake();
        uint8_t getDataRateSleep();
        void setDataRate(uint8_t dataRateWake, uint8_t dataRateSleep);
        uint8_t getWakeTimeMin();
        void setWakeTimeMin(uint8_t wakeTimeMin);

        uint8_t getFifoMode();
        uint8_t getFifoWaterMark();
        uint8_t getFifoCount();
        void setFifoEnable(bool enable=true);
        void setFifo(uint8_t fifoMode, uint8_t fifoWaterMark);

        uint8_t getOrientationThreshold();
        uint8_t getOrientationHysterisis();
        uint8_t getOrientationBackFront();
        uint8_t getOrientationZlock();
        uint8_t getOrientationCounter();
        void setOrientationEnable(bool enable=true);
        void setOrientation(uint8_t threshold, uint8_t hysterisis, uint8_t backFront, uint8_t zLock, uint8_t counter=0, bool debounceMode=false);
        bool checkOrientation();
        uint8_t orientationStatus();
        bool orientationZlock();

        uint8_t getMotionMode();
        uint8_t getMotionAxis();
        uint8_t getMotionThreshold();
        uint8_t getMotionCounter();
        void setMotionEnable(bool enable=true);
        void setMotion(uint8_t mode, uint8_t axis, uint8_t threshold, uint8_t counter=0, bool debounceMode=false);
        bool checkMotion();
        int8_t motionAxisX();
        int8_t motionAxisY();
        int8_t motionAxisZ();

        uint8_t getTransientMode();
        uint8_t getTransientAxis();
        uint8_t getTransientThreshold();
        uint8_t getTransientCounter();
        void setTransientEnable(bool enable=true);
        void setTransient(uint8_t mode, uint8_t axis, uint8_t threshold, uint8_t counter=0, bool debounceMode=false);
        bool checkTransient();
        int8_t transientAxisX();
        int8_t transientAxisY();
        int8_t transientAxisZ();

        uint8_t getPulseMode();
        uint8_t getPulseThresholdX();
        uint8_t getPulseThresholdY();
        uint8_t getPulseThresholdZ();
        uint8_t getPulseTimeLimit();
        uint8_t getPulseTimeLatency();
        uint8_t getPulseTimeWindow();
        void setPulseEnable(bool enable=true);
        void setPulse(uint8_t mode, uint8_t axis, uint8_t thresholdX, uint8_t thresholdY, uint8_t thresholdZ, uint8_t timeLimit, uint8_t timeLatency, uint8_t timeWindow);
        bool checkPulse();
        int8_t pulseAxisX();
        int8_t pulseAxisY();
        int8_t pulseAxisZ();

        void requestInterrupt(uint8_t interruptPin, uint8_t interruptEvent, uint8_t interruptMode);
        void requestInterrupt(uint8_t interruptPin, uint8_t interruptEvent);
        void setInterruptMode(uint8_t interruptMode);
        void resetInterrupt(uint8_t interruptPinEvent);
        void resetInterrupt();
        uint8_t checkInterrupt();
        bool checkInterruptDataReady();
        bool checkInterruptMotion();
        bool checkInterruptPulse();
        bool checkInterruptOrientation();
        bool checkInterruptTransient();
        bool checkInterruptFifo();
        bool checkInterruptAutoSleep();

    // uint8_t peek();

    private:

        uint8_t _deviceAddress;
        uint8_t _deviceId;
        int16_t _dataX, _dataY, _dataZ;
        uint8_t _dataBits;
        uint8_t _dataFlag;
        uint16_t _range;
        double _unit;
        uint8_t _fifoMode;
        uint8_t _axisMotion;
        uint8_t _axisTransient;
        uint8_t _axisPulse;

        uint8_t _checkOrientation;
        uint8_t _checkMotion;
        uint8_t _checkTransient;
        uint8_t _checkPulse;
        uint8_t _checkInterrupt;

        void _readData();
        void _readDataFast();
        void _readDataNormal();
        int16_t _readInst(uint8_t registry, bool mode);

        int8_t _dataToOffset(int16_t data);
        int16_t _offsetToData(int8_t offset);

        uint8_t _readBit(uint8_t address, uint8_t startBit);
        uint8_t _readBits(uint8_t address, uint8_t startBit, uint8_t nBits);
        uint8_t _readByte(uint8_t address);
        uint8_t _readBytes(uint8_t address, uint8_t* data, uint8_t nBytes);
        bool _writeBit(uint8_t address, uint8_t data, uint8_t startBit);
        bool _writeBits(uint8_t address, uint8_t data, uint8_t startBit, uint8_t nBits);
        bool _writeByte(uint8_t address, uint8_t data);
        bool _writeBytes(uint8_t address, uint8_t* data, uint8_t nBytes);

};

#endif
