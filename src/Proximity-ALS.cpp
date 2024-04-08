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

#include "Proximity-ALS.h"
#include "soc/soc.h"

ProximityAL::ProximityAL()
    : ProximityAL(LTR553_ADDR) {}

ProximityAL::ProximityAL(uint8_t i2c_address)
    : I2CDevice(i2c_address)
{
    _address = i2c_address;
}

/**
 * @brief  I2C initialization and read check
 * @param  adress: Device adress
 * @retval None
 **/
bool ProximityAL::begin(void)
{
    bool deviceExists = false;

    Wire.begin();
    Wire.beginTransmission(_address);
    if (Wire.endTransmission() == 0)
    {
        deviceExists = true;

        softwareReset();
        setPSledPulse(PS_LED_PULSES_15);
        setALSmode(ALS_ACTIVE_MODE);
        setPSmode(PS_ACTIVE_MODE);
    }
    return deviceExists;
}

/**
 * @brief
 * @param
 * @retval
 **/
void ProximityAL::setALSmode(uint8_t mode)
{
    uint8_t readValue = read8(LTR553_ALS_CONTR_REG);
    readValue &= (~ALS_MODE_MASK);
    readValue |= mode;
    write_register(LTR553_ALS_CONTR_REG, readValue);
}

/**
 * @brief
 * @param
 * @retval
 **/
void ProximityAL::setPSmode(uint8_t mode)
{
    uint8_t readValue = read8(LTR553_PS_CONTR_REG);
    readValue &= (~PS_MODE_MASK);
    readValue |= mode;
    write_register(LTR553_PS_CONTR_REG, readValue);
}

/**
 * @brief
 * @param
 * @retval
 **/
void ProximityAL::softwareReset(void)
{
    uint8_t readValue = read8(LTR553_ALS_CONTR_REG);
    readValue &= (~ALS_SOFTWARE_RESET_MASK);
    readValue |= 0x02;
    write_register(LTR553_ALS_CONTR_REG, readValue);
}

/**
 * @brief
 * @param
 * @retval
 **/
void ProximityAL::setALSgain(uint8_t gain)
{
    uint8_t readValue = read8(LTR553_ALS_CONTR_REG);
    readValue &= (~ALS_GAIN_MASK);
    readValue |= gain;
    write_register(LTR553_ALS_CONTR_REG, readValue);
}

/**
 * @brief
 * @param
 * @retval
 **/
uint8_t ProximityAL::getALSgain(void)
{
    uint8_t readValue = read8(LTR553_ALS_CONTR_REG);
    readValue &= ALS_GAIN_MASK;
    return readValue;
}

/**
 * @brief
 * @param
 * @retval
 **/
void ProximityAL::setPSsaturationIndicator(uint8_t strIndicator)
{
    uint8_t readValue = read8(LTR553_PS_CONTR_REG);
    readValue &= (~PS_SATURATION_INDICATOR_MASK);
    readValue |= strIndicator;
    write_register(LTR553_PS_CONTR_REG, readValue);
}

/**
 * @brief
 * @param
 * @retval
 **/
void ProximityAL::setPSledPulseFreq(uint8_t ledPulseFreq)
{
    uint8_t readValue = read8(LTR553_PS_LED_REG);
    readValue &= (~PS_LED_PULSE_FREQUENCY_MASK);
    readValue |= ledPulseFreq;
    write_register(LTR553_PS_LED_REG, readValue);
}

/**
 * @brief
 * @param
 * @retval
 **/
void ProximityAL::setPSledDutyCycle(uint8_t ledDutyCycle)
{
    uint8_t readValue = read8(LTR553_PS_LED_REG);
    readValue &= (~PS_LED_DUTY_CYCLE_MASK);
    readValue |= ledDutyCycle;
    write_register(LTR553_PS_LED_REG, readValue);
}

/**
 * @brief
 * @param
 * @retval
 **/
void ProximityAL::setPSledPeakCurrent(uint8_t ledPeakCurrent)
{
    uint8_t readValue = read8(LTR553_PS_LED_REG);
    readValue &= (~PS_LED_PEAK_CURRENT_MASK);
    readValue |= ledPeakCurrent;
    write_register(LTR553_PS_LED_REG, readValue);
}

/**
 * @brief
 * @param
 * @retval
 **/
void ProximityAL::setPSledPulse(uint8_t ledPulse)
{
    write_register(LTR553_PS_N_PULSES_REG, ledPulse);
}

/**
 * @brief
 * @param
 * @retval
 **/
void ProximityAL::setPSmeasurementRate(uint8_t measRate)
{
    write_register(LTR553_PS_MEAS_RATE_REG, measRate);
}

/**
 * @brief
 * @param
 * @retval
 **/
void ProximityAL::setALSintegrationTime(uint8_t intTime)
{
    uint8_t readValue = read8(LTR553_ALS_MEAS_PATE_REG);
    readValue &= (~ALS_INTEGRATION_TIME_MASK);
    readValue |= intTime;
    write_register(LTR553_ALS_MEAS_PATE_REG, readValue);
}

/**
 * @brief
 * @param
 * @retval
 **/
uint8_t ProximityAL::getALSintegrationTime(void)
{
    uint8_t readValue = read8(LTR553_ALS_MEAS_PATE_REG);
    readValue &= ALS_INTEGRATION_TIME_MASK;
    return readValue;
}

/**
 * @brief
 * @param
 * @retval
 **/
void ProximityAL::setALSmeasurementRate(uint8_t measRate)
{
    uint8_t readValue = read8(LTR553_ALS_MEAS_PATE_REG);
    readValue &= (~ALS_MEAS_REPEAT_RATE_MASK);
    readValue |= measRate;
    write_register(LTR553_ALS_MEAS_PATE_REG, readValue);
}

/**
 * @brief
 * @param
 * @retval
 **/
uint8_t ProximityAL::getPartNumberID(void)
{
    uint8_t readValue = read8(LTR553_PART_ID_REG);
    readValue &= PART_NUMBER_ID_MASK;
    return readValue >> 4;
}

/**
 * @brief
 * @param
 * @retval
 **/
uint8_t ProximityAL::getRevisionID(void)
{
    uint8_t readValue = read8(LTR553_PART_ID_REG);
    readValue &= REVISION_ID_MASK;
    return readValue;
}

/**
 * @brief
 * @param
 * @retval
 **/
uint8_t ProximityAL::getManufacturerID(void)
{
    return read8(LTR553_MANUFAC_ID_REG);
}

/**
 * @brief
 * @param
 * @retval
 **/
uint16_t ProximityAL::getPSvalue(void)
{
    uint8_t buffer[2];
    read_register(LTR553_PS_DATA_0_REG, 2, &buffer[0]);
    buffer[0] &= PS_DATA_LOW_MASK;
    buffer[1] &= PS_DATA_HIGH_MASK;
    return (buffer[1] << 8) | buffer[0];
}

/**
 * @brief
 * @param
 * @retval
 **/
uint16_t ProximityAL::getALSCH0value(void)
{
    uint8_t buffer[2];
    read_register(LTR553_ALS_DATA_CH0_0_REG, 2, &buffer[0]);
    return (buffer[1] << 8) | buffer[0];
}

/**
 * @brief
 * @param
 * @retval
 **/
uint16_t ProximityAL::getALSCH1value(void)
{
    uint8_t buffer[2];
    read_register(LTR553_ALS_DATA_CH1_0_REG, 2, &buffer[0]);
    return (buffer[1] << 8) | buffer[0];
}

/**
 * @brief
 * @param
 * @retval
 **/
float ProximityAL::getLuxValue(void)
{
    uint16_t CH0, CH1;
    float ratio;
    CH1 = getALSCH1value();
    CH0 = getALSCH0value();

    ratio = ((float)(CH1 / ((float)CH1 + CH0)));

    if (ratio < 0.45)
        return ((float)1.7743 * CH0) + ((float)1.1059 * CH1);

    else if (0.45 <= ratio && ratio < 0.64)
        return ((float)4.2785 * CH0) - ((float)1.9548 * CH1);

    else if (0.64 <= ratio && ratio < 0.85)
        return ((float)0.5926 * CH0) + ((float)0.1185 * CH1);

    else
        return 0;
}
