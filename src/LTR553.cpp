/*
*****************************************************************************
@file         Deneyap_MesafeOlcerIsikAlgilayici.cpp
@mainpage     Deneyap Proximity&Light Sensor LTR553 Arduino library source file
@maintainer   RFtek Electronics <techsupport@rftek.com.tr>
@version      v1.0.0
@date         June 24, 2022
@brief        Includes functions to control Deneyap Proximity&Light Sensor LTR553
              Arduino library

Library includes:
--> Configuration functions
--> Data manipulation functions
--> I2C communication functions
*****************************************************************************
*/

#include "LTR553.h"
#include "soc/soc.h"

LTR553::LTR553()
    : LTR553(LTR553_ADDR) {}

LTR553::LTR553(uint8_t i2c_address)
    : I2CDevice(i2c_address)
{
    _address = i2c_address;
}

/**
 * @brief  I2C initialization and read check
 * @param  adress: Device adress
 * @retval None
 **/
bool LTR553::begin(void)
{
    bool deviceExists = false;

    Wire.begin();
    Wire.beginTransmission(_address);
    if (Wire.endTransmission() == 0)
    {
        deviceExists = true;

        /* Registers are at their reset defaults (0x00) after this write, so the later
           mode writes need no read-modify-write. */
        write_register(LTR553_ALS_CONTR_REG, LTR553_ALS_SOFTWARE_RESET_ENABLE);
        delay(LTR553_RESET_SETTLE_MS);
        setPSledPulse(LTR553_PS_LED_PULSES_15);
        write_register(LTR553_ALS_CONTR_REG, LTR553_ALS_ACTIVE_MODE);
        write_register(LTR553_PS_CONTR_REG, LTR553_PS_ACTIVE_MODE);
    }
    return deviceExists;
}

/**
 * @brief
 * @param
 * @retval
 **/
void LTR553::setALSmode(uint8_t mode)
{
    uint8_t readValue = read8(LTR553_ALS_CONTR_REG);
    readValue &= (~LTR553_ALS_MODE_MASK);
    readValue |= mode;
    write_register(LTR553_ALS_CONTR_REG, readValue);
}

/**
 * @brief
 * @param
 * @retval
 **/
void LTR553::setPSmode(uint8_t mode)
{
    uint8_t readValue = read8(LTR553_PS_CONTR_REG);
    readValue &= (~LTR553_PS_MODE_MASK);
    readValue |= mode;
    write_register(LTR553_PS_CONTR_REG, readValue);
}

/**
 * @brief
 * @param
 * @retval
 **/
void LTR553::softwareReset(void)
{
    uint8_t readValue = read8(LTR553_ALS_CONTR_REG);
    readValue &= (~LTR553_ALS_SOFTWARE_RESET_MASK);
    readValue |= 0x02;
    write_register(LTR553_ALS_CONTR_REG, readValue);
}

/**
 * @brief
 * @param
 * @retval
 **/
void LTR553::setALSgain(uint8_t gain)
{
    uint8_t readValue = read8(LTR553_ALS_CONTR_REG);
    readValue &= (~LTR553_ALS_GAIN_MASK);
    readValue |= gain;
    write_register(LTR553_ALS_CONTR_REG, readValue);
}

/**
 * @brief
 * @param
 * @retval
 **/
uint8_t LTR553::getALSgain(void)
{
    uint8_t readValue = read8(LTR553_ALS_CONTR_REG);
    readValue &= LTR553_ALS_GAIN_MASK;
    return readValue;
}

/**
 * @brief
 * @param
 * @retval
 **/
void LTR553::setPSsaturationIndicator(uint8_t strIndicator)
{
    uint8_t readValue = read8(LTR553_PS_CONTR_REG);
    readValue &= (~LTR553_PS_SATURATION_INDICATOR_MASK);
    readValue |= strIndicator;
    write_register(LTR553_PS_CONTR_REG, readValue);
}

/**
 * @brief
 * @param
 * @retval
 **/
void LTR553::setPSledPulseFreq(uint8_t ledPulseFreq)
{
    uint8_t readValue = read8(LTR553_PS_LED_REG);
    readValue &= (~LTR553_PS_LED_PULSE_FREQUENCY_MASK);
    readValue |= ledPulseFreq;
    write_register(LTR553_PS_LED_REG, readValue);
}

/**
 * @brief
 * @param
 * @retval
 **/
void LTR553::setPSledDutyCycle(uint8_t ledDutyCycle)
{
    uint8_t readValue = read8(LTR553_PS_LED_REG);
    readValue &= (~LTR553_PS_LED_DUTY_CYCLE_MASK);
    readValue |= ledDutyCycle;
    write_register(LTR553_PS_LED_REG, readValue);
}

/**
 * @brief
 * @param
 * @retval
 **/
void LTR553::setPSledPeakCurrent(uint8_t ledPeakCurrent)
{
    uint8_t readValue = read8(LTR553_PS_LED_REG);
    readValue &= (~LTR553_PS_LED_PEAK_CURRENT_MASK);
    readValue |= ledPeakCurrent;
    write_register(LTR553_PS_LED_REG, readValue);
}

/**
 * @brief
 * @param
 * @retval
 **/
void LTR553::setPSledPulse(uint8_t ledPulse)
{
    write_register(LTR553_PS_N_PULSES_REG, ledPulse);
}

/**
 * @brief
 * @param
 * @retval
 **/
void LTR553::setPSmeasurementRate(uint8_t measRate)
{
    write_register(LTR553_PS_MEAS_RATE_REG, measRate);
}

/**
 * @brief
 * @param
 * @retval
 **/
void LTR553::setALSintegrationTime(uint8_t intTime)
{
    uint8_t readValue = read8(LTR553_ALS_MEAS_PATE_REG);
    readValue &= (~LTR553_ALS_INTEGRATION_TIME_MASK);
    readValue |= intTime;
    write_register(LTR553_ALS_MEAS_PATE_REG, readValue);
}

/**
 * @brief
 * @param
 * @retval
 **/
uint8_t LTR553::getALSintegrationTime(void)
{
    uint8_t readValue = read8(LTR553_ALS_MEAS_PATE_REG);
    readValue &= LTR553_ALS_INTEGRATION_TIME_MASK;
    return readValue;
}

/**
 * @brief
 * @param
 * @retval
 **/
void LTR553::setALSmeasurementRate(uint8_t measRate)
{
    uint8_t readValue = read8(LTR553_ALS_MEAS_PATE_REG);
    readValue &= (~LTR553_ALS_MEAS_REPEAT_RATE_MASK);
    readValue |= measRate;
    write_register(LTR553_ALS_MEAS_PATE_REG, readValue);
}

/**
 * @brief
 * @param
 * @retval
 **/
uint8_t LTR553::getPartNumberID(void)
{
    uint8_t readValue = read8(LTR553_PART_ID_REG);
    readValue &= LTR553_PART_NUMBER_ID_MASK;
    return readValue >> 4;
}

/**
 * @brief
 * @param
 * @retval
 **/
uint8_t LTR553::getRevisionID(void)
{
    uint8_t readValue = read8(LTR553_PART_ID_REG);
    readValue &= LTR553_REVISION_ID_MASK;
    return readValue;
}

/**
 * @brief
 * @param
 * @retval
 **/
uint8_t LTR553::getManufacturerID(void)
{
    return read8(LTR553_MANUFAC_ID_REG);
}

/**
 * @brief
 * @param
 * @retval
 **/
uint16_t LTR553::getPSvalue(void)
{
    uint8_t buffer[LTR553_PS_DATA_LEN] = {0};
    readRegisters(LTR553_PS_DATA_0_REG, LTR553_PS_DATA_LEN, buffer);
    buffer[0] &= LTR553_PS_DATA_LOW_MASK;
    buffer[1] &= LTR553_PS_DATA_HIGH_MASK;
    return (buffer[1] << 8) | buffer[0];
}

/**
 * @brief
 * @param
 * @retval
 **/
uint16_t LTR553::getALSCH0value(void)
{
    uint8_t buffer[2] = {0};
    readRegisters(LTR553_ALS_DATA_CH0_0_REG, 2, buffer);
    return (buffer[1] << 8) | buffer[0];
}

/**
 * @brief
 * @param
 * @retval
 **/
uint16_t LTR553::getALSCH1value(void)
{
    uint8_t buffer[2] = {0};
    readRegisters(LTR553_ALS_DATA_CH1_0_REG, 2, buffer);
    return (buffer[1] << 8) | buffer[0];
}

/**
 * @brief  Lux from one 4-byte burst of ALS_DATA_CH1_0..CH0_1 (0x88..0x8B)
 * @retval lux (same piecewise formula and values as the two-read version)
 **/
float LTR553::getLuxValue(void)
{
    uint8_t buffer[LTR553_ALS_DATA_LEN] = {0};
    readRegisters(LTR553_ALS_DATA_CH1_0_REG, LTR553_ALS_DATA_LEN, buffer);
    const uint16_t CH1 = (buffer[LTR553_ALS_DATA_CH1_HIGH_IDX] << 8) | buffer[LTR553_ALS_DATA_CH1_LOW_IDX];
    const uint16_t CH0 = (buffer[LTR553_ALS_DATA_CH0_HIGH_IDX] << 8) | buffer[LTR553_ALS_DATA_CH0_LOW_IDX];

    const float ratio = CH1 / ((float)CH1 + CH0);

    if (ratio <= LTR553_LUX_RATIO_BAND0_MAX)
        return ((float)1.7743 * CH0) + ((float)1.1059 * CH1);

    else if (ratio <= LTR553_LUX_RATIO_BAND1_MAX)
        return ((float)4.2785 * CH0) - ((float)1.9548 * CH1);

    else if (ratio < LTR553_LUX_RATIO_BAND2_LIMIT)
        return ((float)0.5926 * CH0) + ((float)0.1185 * CH1);

    else
        return 0;
}

/**
 * @brief  Register-pointer write + data read as one repeated-START transaction
 *         (one driver call instead of two). Same semantics as I2CDevice::read_register
 *         but drains the receive buffer with Wire.read() instead of Stream::readBytes(),
 *         which polls millis() per byte and blocks up to 1 s on a short read.
 **/
size_t LTR553::readRegisters(uint8_t reg_addr, size_t length, uint8_t *data)
{
    Wire.beginTransmission(_address);
    Wire.write(reg_addr);
    Wire.endTransmission(false);

    const size_t receivedBytes = Wire.requestFrom(_address, length);
    for (size_t i = 0; i < receivedBytes && i < length; i++)
        data[i] = (uint8_t)Wire.read();
    return receivedBytes;
}
