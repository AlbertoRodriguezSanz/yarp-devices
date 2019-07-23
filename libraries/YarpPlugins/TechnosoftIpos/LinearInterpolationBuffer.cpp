// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "LinearInterpolationBuffer.hpp"

#include <ColorDebug.h>

using namespace roboticslab;

LinearInterpolationBuffer::LinearInterpolationBuffer(double _periodMs, double _bufferSize, TechnosoftIpos * _technosoftIpos)
    : technosoftIpos(_technosoftIpos),
      periodMs(_periodMs),
      bufferSize(_bufferSize),
      lastSentTarget(0.0),
      lastReceivedTarget(0.0)
{}

LinearInterpolationBuffer * LinearInterpolationBuffer::createBuffer(yarp::os::Searchable& config, TechnosoftIpos * technosoftIpos)
{
    int linInterpPeriodMs = config.check("linInterpPeriodMs", yarp::os::Value(0),
            "linear interpolation mode period (ms)").asInt32();
    int linInterpBufferSize = config.check("linInterpBufferSize", yarp::os::Value(DEFAULT_LIN_INTERP_BUFFER_SIZE),
            "linear interpolation mode buffer size").asInt32();
    std::string linInterpMode = config.check("linInterpMode", yarp::os::Value(DEFAULT_LIN_INTERP_MODE),
            "linear interpolation mode (PT/PVT)").asString();

    if (linInterpPeriodMs <= 0)
    {
        CD_ERROR("Invalid linear interpolation mode period: %d.\n", linInterpPeriodMs);
        return 0;
    }

    if (linInterpBufferSize <= 0)
    {
        CD_ERROR("Invalid linear interpolation mode buffer size: %d.\n", linInterpBufferSize);
        return 0;
    }

    if (linInterpMode == "pt")
    {
        if (linInterpBufferSize > PT_BUFFER_MAX_SIZE)
        {
            CD_ERROR("Invalid PT mode buffer size: %d > %d.\n", linInterpBufferSize, PT_BUFFER_MAX_SIZE);
            return 0;
        }

        return new PtBuffer(linInterpPeriodMs, linInterpBufferSize, technosoftIpos);
    }
    else if (linInterpMode == "pvt")
    {
        if (linInterpBufferSize > PVT_BUFFER_MAX_SIZE)
        {
            CD_ERROR("Invalid PVT mode buffer size: %d > %d.\n", linInterpBufferSize, PVT_BUFFER_MAX_SIZE);
            return 0;
        }

        return new PvtBuffer(linInterpPeriodMs, linInterpBufferSize, technosoftIpos);
    }
    else
    {
        CD_ERROR("Unsupported linear interpolation mode: %s.\n", linInterpMode.c_str());
        return 0;
    }
}

void LinearInterpolationBuffer::setInitialReference(double target)
{
    lastSentTarget = target;
}

void LinearInterpolationBuffer::updateTarget(double target)
{
    mutex.lock();
    lastReceivedTarget = target;
    mutex.unlock();
}

void LinearInterpolationBuffer::setBufferSize(uint8_t * msg)
{
    std::memcpy(msg + 4, &bufferSize, 2);
}

PtBuffer::PtBuffer(double periodMs, double bufferSize, TechnosoftIpos * technosoftIpos)
    : LinearInterpolationBuffer(periodMs, bufferSize, technosoftIpos)
{}

void PtBuffer::setSubMode(uint8_t * msg)
{
    uint16_t submode = 0;
    std::memcpy(msg + 4, &submode, 2);
}

void PtBuffer::createMessage(uint8_t * msg)
{
    //*************************************************************
    //-- 14. Send the 1 st PT point.
    //-- Position= 20000 IU (0x00004E20) 1IU = 1 encoder pulse
    //-- Time = 1000 IU (0x03E8)
    //-- IC = 1 (0x01)
    //-- 1IU = 1 control loop = 1ms by default
    //-- IC=Integrity Counter
    //-- The drive motor will do 10 rotations (20000 counts) in 1000 milliseconds.
    //-- Send the following message:
    //uint8_t ptpoint1[]={0x20,0x4E,0x00,0x00,0xE8,0x03,0x00,0x02};

    mutex.lock();
    double p = lastReceivedTarget;
    lastSentTarget = lastReceivedTarget;
    mutex.unlock();

    int t = periodMs;
    const double factor = technosoftIpos->tr * (technosoftIpos->encoderPulses / 360.0);

    int32_t position = p * factor;  // Apply tr & convert units to encoder increments
    std::memcpy(msg, &position, 4);

    int16_t time = t;
    std::memcpy(msg + 4, &time, 2);

    uint8_t ic = (++technosoftIpos->integrityCounter) << 1;
    std::memcpy(msg + 7, &ic, 1);

    CD_DEBUG("Sending to canId %d: pos %f, time %d, ic %d.\n",
                technosoftIpos->canId, p, t, technosoftIpos->integrityCounter);
}

PvtBuffer::PvtBuffer(double periodMs, double bufferSize, TechnosoftIpos * technosoftIpos)
    : LinearInterpolationBuffer(periodMs, bufferSize, technosoftIpos)
{}

void PvtBuffer::setSubMode(uint8_t * msg)
{
    uint16_t submode = -1;
    std::memcpy(msg + 4, &submode, 2);
}


void PvtBuffer::createMessage(uint8_t * msg)
{
    //*************************************************************
    //-- 13. Send the 1 st PT point.
    //-- Position = 88 IU (0x000058) 1IU = 1 encoder pulse
    //-- Velocity = 3.33 IU (0x000354) 1IU = 1 encoder pulse/ 1 control loop
    //-- Time = 55 IU (0x37) 1IU = 1 control loop = 1ms by default
    //-- IC = 0 (0x00) IC=Integrity Counter
    //-- Send the following message:
    //uint8_t ptpoint1[]={0x58,0x00,0x54,0x00,0x03,0x00,0x37,0x00};

    mutex.lock();
    double p = lastReceivedTarget;
    double previousTarget = lastSentTarget;
    lastSentTarget = lastReceivedTarget;
    mutex.unlock();

    double v = (lastReceivedTarget - previousTarget) / (periodMs / 1000.0);
    int t = periodMs;
    const double factor = technosoftIpos->tr * (technosoftIpos->encoderPulses / 360.0);

    int32_t position = p * factor;  // Apply tr & convert units to encoder increments
    int16_t positionLSB = (int32_t)(position << 16) >> 16;
    int8_t positionMSB = (int32_t)(position << 8) >> 24;
    std::memcpy(msg, &positionLSB, 2);
    std::memcpy(msg + 3, &positionMSB, 1);

    double velocity = v * factor * 0.001;
    int16_t velocityInt, velocityFrac;
    TechnosoftIpos::encodeFixedPoint(velocity, &velocityInt, &velocityFrac);
    std::memcpy(msg + 2, &velocityFrac, 1);
    std::memcpy(msg + 4, &velocityInt, 2);

    int16_t time = (int16_t)(t << 7) >> 7;
    uint8_t ic = (++technosoftIpos->integrityCounter) << 1;
    uint16_t timeAndIc = time + ic;
    std::memcpy(msg + 6, &timeAndIc, 2);

    CD_DEBUG("Sending to canId %d: pos %f, vel %f, time %d, ic %d.\n",
                technosoftIpos->canId, p, v, t, technosoftIpos->integrityCounter);
}
