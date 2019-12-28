// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "TechnosoftIpos.hpp"

#include <ColorDebug.h>

#include "CanUtils.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

namespace
{
    inline std::uint8_t getByte(std::uint32_t number, int n)
    {
        // https://stackoverflow.com/a/7787433
        return (number >> (8 * n)) & 0xFF;
    }
}

// -----------------------------------------------------------------------------

unsigned int TechnosoftIpos::getId()
{
    return can->getId();
}

// -----------------------------------------------------------------------------

std::vector<unsigned int> TechnosoftIpos::getAdditionalIds()
{
    if (iExternalEncoderCanBusSharer)
    {
        return {iExternalEncoderCanBusSharer->getId()};
    }

    return {};
}

// -----------------------------------------------------------------------------

bool TechnosoftIpos::registerSender(CanSenderDelegate * sender)
{
    can->configureSender(sender);
    return iExternalEncoderCanBusSharer && iExternalEncoderCanBusSharer->registerSender(sender);
}

// -----------------------------------------------------------------------------

bool TechnosoftIpos::initialize()
{
    if (iExternalEncoderCanBusSharer && !iExternalEncoderCanBusSharer->initialize())
    {
        CD_ERROR("Unable to initialize external encoder device.\n");
        return false;
    }

    std::uint32_t data;

    if (!can->sdo()->upload("Device type", &data, 0x1000))
    {
        return false;
    }

    CD_INFO("CiA standard: %d.\n", data & 0xFFFF);

    if (!can->sdo()->upload("Supported drive modes", &data, 0x6502))
    {
        return false;
    }

    interpretSupportedDriveModes(data);

    std::string firmware;

    if (!can->sdo()->upload("Manufacturer software version", firmware, 0x100A))
    {
        return false;
    }

    CD_INFO("Firmware version: %s.\n", firmware.c_str());

    can->sdo()->upload("Identity Object: Vendor ID", &data, 0x1018, 0x01);

    if (!can->sdo()->upload("Identity Object: Product Code", &data, 0x1018, 0x02))
    {
        return false;
    }

    CD_INFO("Retrieved product code: P%03d.%03d.E%03d.\n", data / 1000000, (data / 1000) % 1000, data % 1000);

    if (!can->sdo()->upload("Identity Object: Revision number", &data, 0x1018, 0x03))
    {
        return false;
    }

    CD_INFO("Revision number: %c%c%c%c.\n", getByte(data, 3), getByte(data, 2), getByte(data, 1), getByte(data, 0));

    if (!can->sdo()->upload("Identity Object: Serial number", &data, 0x1018, 0x04))
    {
        return false;
    }

    CD_INFO("Serial number: %c%c%02x%02x.\n", getByte(data, 3), getByte(data, 2), getByte(data, 1), getByte(data, 0));

    if (!setLimitsRaw(0, vars.min, vars.max))
    {
        CD_ERROR("Unable to set software limits.\n");
        return false;
    }

    if (!setRefSpeedRaw(0, vars.refSpeed))
    {
        CD_ERROR("Unable to set reference speed.\n");
        return false;
    }

    if (!setRefAccelerationRaw(0, vars.refAcceleration))
    {
        CD_ERROR("Unable to set reference acceleration.\n");
        return false;
    }

    if (iEncodersTimedRawExternal)
    {
        // synchronize absolute (master) and relative (slave) encoders
        double extEnc;

        if (!iEncodersTimedRawExternal->getEncodersRaw(&extEnc))
        {
            return false;
        }

        CD_INFO("External absolute encoder read %f.\n", extEnc);

        if (!setEncoderRaw(0, extEnc))
        {
            return false;
        }
    }

    if (!can->tpdo1()->configure(vars.tpdo1Conf))
    {
        CD_ERROR("Unable to configure TPDO1.\n");
        return false;
    }

    if (!can->tpdo2()->configure(vars.tpdo2Conf))
    {
        CD_ERROR("Unable to configure TPDO2.\n");
        return false;
    }

    if (!can->tpdo3()->configure(vars.tpdo3Conf))
    {
        CD_ERROR("Unable to configure TPDO3.\n");
        return false;
    }

    vars.actualControlMode = VOCAB_CM_CONFIGURED;

    if (!can->nmt()->issueServiceCommand(NmtService::START_REMOTE_NODE))
    {
        return false;
    }

    if (can->driveStatus()->getCurrentState() == DriveState::NOT_READY_TO_SWITCH_ON &&
        !can->driveStatus()->awaitState(DriveState::SWITCH_ON_DISABLED))
    {
        CD_ERROR("SWITCH_ON_DISABLED state check failed.\n");
        return false;
    }

    if (!can->driveStatus()->requestState(DriveState::SWITCHED_ON))
    {
        return false;
    }

    return true;
}

// -----------------------------------------------------------------------------

bool TechnosoftIpos::finalize()
{
    bool ok = true;

    if (!can->driveStatus()->requestState(DriveState::SWITCH_ON_DISABLED))
    {
        CD_WARNING("SWITCH_ON_DISABLED transition failed.\n");
        ok = false;
    }
    else
    {
        vars.actualControlMode = VOCAB_CM_CONFIGURED;
    }

    if (!can->nmt()->issueServiceCommand(NmtService::RESET_NODE))
    {
        CD_WARNING("Reset node NMT service failed.\n");
        ok = false;
    }
    else
    {
        vars.actualControlMode = VOCAB_CM_NOT_CONFIGURED;
    }

    return ok;
}

// -----------------------------------------------------------------------------

bool TechnosoftIpos::interpretMessage(const yarp::dev::CanMessage & message)
{
    if (iExternalEncoderCanBusSharer && iExternalEncoderCanBusSharer->getId() == (message.getId() & 0x7F))
    {
        return iExternalEncoderCanBusSharer->interpretMessage(message);
    }

    if (!can->consumeMessage(message.getId(), message.getData(), message.getLen()))
    {
        CD_WARNING("Unknown message: %s\n", CanUtils::msgToStr(message.getId(), message.getLen(), message.getData()).c_str());
        return false;
    }

    return true;
}

// -----------------------------------------------------------------------------
