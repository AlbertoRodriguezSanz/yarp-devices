// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "TechnosoftIpos.hpp"

#include <bitset>

#include "CanUtils.hpp"

// -----------------------------------------------------------------------------

namespace
{
    bool retrieveDrivePeakCurrent(uint32_t productCode, double *peakCurrent)
    {
        switch (productCode)
        {
            case 24300101: // iPOS2401 MX-CAN
            case 24200121: // iPOS2401 MX-CAT
                *peakCurrent = 0.9;
                break;
            case 28001001: // iPOS3602 VX-CAN
            case 28001021: // iPOS3602 VX-CAT
            case 28001101: // iPOS3602 MX-CAN
            case 28001201: // iPOS3602 BX-CAN
            case 28001501: // iPOS3602 HX-CAN
                *peakCurrent = 3.2;
                break;
            case 28002001: // iPOS3604 VX-CAN
            case 28002021: // iPOS3604 VX-CAT
            case 28002101: // iPOS3604 MX-CAN
            case 28002201: // iPOS3604 BX-CAN
            case 28002501: // iPOS3604 HX-CAN
                *peakCurrent = 10.0;
                break;
            case 27014001: // iPOS4808 VX-CAN
            case 27014101: // iPOS4808 MX-CAN
            case 27014121: // iPOS4808 MX-CAT
            case 27414101: // iPOS4808 MY-CAN (standard)
            case 27424101: // iPOS4808 MY-CAN (extended)
            case 27314111: // iPOS4808 MY-CAN-STO (standard)
            case 27324111: // iPOS4808 MY-CAN-STO (extended)
            case 27314121: // iPOS4808 MY-CAT-STO (standard)
            case 27324121: // iPOS4808 MY-CAT-STO (extended)
            case 27014201: // iPOS4808 BX-CAN
            case 27214201: // iPOS4808 BX-CAN (standard)
            case 27214701: // iPOS4808 BX-CAN (hall)
            case 27214221: // iPOS4808 BX-CAT (standard)
            case 27214721: // iPOS4808 BX-CAT (hall)
            case 27314221: // iPOS4808 BX-CAT-STO (standard)
            case 27314721: // iPOS4808 BX-CAT-STO (hall)
            case 29025201: // iPOS8010 BX-CAN
            case 29025221: // iPOS8010 BX-CAT
            case 29025202: // iPOS8010 BA-CAN
            case 29025222: // iPOS8010 BA-CAT
                *peakCurrent = 20.0;
                break;
            case 29026201: // iPOS8020 BX-CAN
            case 29026221: // iPOS8020 BX-CAT
            case 29026202: // iPOS8020 BA-CAN
            case 29026222: // iPOS8020 BA-CAT
                *peakCurrent = 40.0;
                break;
            default:
                return false;
        }

        return true;
    }

    void interpretSupportedDriveModes(uint32_t data)
    {
        std::bitset<32> bits(data);

        if (bits.test(0))
        {
            CD_INFO("\t*profiled position (pp)\n");
        }
        if (bits.test(1))
        {
            CD_INFO("\t*velocity (vl)\n");
        }
        if (bits.test(2))
        {
            CD_INFO("\t*profiled velocity (pv)\n");
        }
        if (bits.test(3))
        {
            CD_INFO("\t*profiled torque (tq)\n");
        }
        if (bits.test(5))
        {
            CD_INFO("\t*homing (hm)\n");
        }
        if (bits.test(6))
        {
            CD_INFO("\t*interpolated position (ip)\n");
        }
        if (bits.test(7))
        {
            CD_INFO("\t*cyclic synchronous position\n");
        }
        if (bits.test(8))
        {
            CD_INFO("\t*cyclic synchronous velocity\n");
        }
        if (bits.test(9))
        {
            CD_INFO("\t*cyclic synchronous torque\n");
        }
        if (bits.test(16))
        {
            CD_INFO("\t*electronic camming position (manufacturer specific)\n");
        }
        if (bits.test(17))
        {
            CD_INFO("\t*electronic gearing position (manufacturer specific)\n");
        }
        if (bits.test(18))
        {
            CD_INFO("\t*external reference position (manufacturer specific)\n");
        }
        if (bits.test(19))
        {
            CD_INFO("\t*external reference speed (manufacturer specific)\n");
        }
        if (bits.test(20))
        {
            CD_INFO("\t*external reference torque (manufacturer specific)\n");
        }
    }

    inline char getByte(uint32_t number, int n)
    {
        // https://stackoverflow.com/a/7787433
        return (number >> (8 * n)) & 0xFF;
    }
}

// -----------------------------------------------------------------------------

bool roboticslab::TechnosoftIpos::registerSender(CanSenderDelegate * sender)
{
    this->sender = sender;
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::TechnosoftIpos::setIEncodersTimedRawExternal(IEncodersTimedRaw * iEncodersTimedRaw)
{
    double v;
    this->iEncodersTimedRawExternal = iEncodersTimedRaw;

    CD_SUCCESS("Ok pointer to external encoder interface %p (%d). Updating with latest external...\n",iEncodersTimedRaw,canId);

    CD_INFO("canId(%d) wait to get external encoder value...\n",this->canId);
    while( !iEncodersTimedRawExternal->getEncoderRaw(0,&v) )  //-- loop while v is still a NaN.
    {
        //CD_INFO("Wait to get external encoder value...\n"); //\todo{activate these lines if blocking is too much}
        //Time::delay(0.2);
    }
    this->setEncoderRaw(0,v);  //-- Forces the relative encoder to this value.

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::TechnosoftIpos::initialize()
{
    uint32_t data;

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

    if (!can->sdo()->upload("Manufacturer software version", &firmware, 0x100A))
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

    if (!retrieveDrivePeakCurrent(data, &drivePeakCurrent))
    {
        CD_ERROR("Unhandled iPOS model %d, unable to retrieve drive peak current.\n", data);
        return false;
    }

    CD_SUCCESS("Retrieved drive peak current: %f A.\n", drivePeakCurrent);

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

    return can->sdo()->download<int16_t>("Quick stop option code", 6, 0x605A);
}

// -----------------------------------------------------------------------------
/** -- Start Remote Node: Used to change NMT state of one or all NMT slaves to Operational.
 PDO communication will beallowed. */

bool roboticslab::TechnosoftIpos::start()
{
    // NMT Start Remote Node (to operational, Fig 4.1)
    uint8_t msg_start[] = {0x01, (uint8_t)canId};
    return sender->prepareMessage(message_builder(0, 2, msg_start));
}

// -----------------------------------------------------------------------------

bool roboticslab::TechnosoftIpos::readyToSwitchOn()
{
    return can->rpdo1()->write<uint16_t>(0x0006);
}

// -----------------------------------------------------------------------------

bool roboticslab::TechnosoftIpos::switchOn()
{
    return can->rpdo1()->write<uint16_t>(0x0007);
}

// -----------------------------------------------------------------------------

bool roboticslab::TechnosoftIpos::enable()
{
    return can->rpdo1()->write<uint16_t>(0x000F);
}

// -----------------------------------------------------------------------------

bool roboticslab::TechnosoftIpos::recoverFromError()
{
    //*************************************************************
    //j//uint8_t msg_recover[]={0x23,0xFF}; // Control word 6040

    //j//if( ! send(0x200, 2, msg_recover)){
    //j//    CD_ERROR("Sent \"recover\". %s\n", msgToStr(0x200, 2, msg_recover).c_str() );
    //j//    return false;
    //j//}
    //j//CD_SUCCESS("Sent \"recover\". %s\n", msgToStr(0x200, 2, msg_recover).c_str() );
    //*************************************************************

    return true;
}

/** Manual: 4.1.2. Device control
    Reset Node: The NMT master sets the state of the selected NMT slave to the reset application sub-state.
    In this state the drives perform a software reset and enter the pre-operational state.
 **/

bool roboticslab::TechnosoftIpos::resetNodes()
{
    // NMT Reset Node (Manual 4.1.2.3)
    uint8_t msg_resetNodes[] = {0x81,0x00};
    return sender->prepareMessage(message_builder(0, 2, msg_resetNodes));
}

/** Manual: 4.1.2. Device control
 * The NMT master sets the state of the selected NMT slave to the “reset communication” sub-state.
 * In this state the drives resets their communication and enter the pre-operational state.
 */

bool roboticslab::TechnosoftIpos::resetCommunication()
{
    return can->rpdo1()->write<uint16_t>(0x0002);
}


/** Manual: 4.1.2. Device control
    Reset Node: The NMT master sets the state of the selected NMT slave to the reset application sub-state.
    In this state the drives perform a software reset and enter the pre-operational state.
 **/

bool roboticslab::TechnosoftIpos::resetNode(int id)
{
    // NMT Reset Node (Manual 4.1.2.3)
    uint8_t msg_resetNode[] = {0x81, (uint8_t)id};
    return sender->prepareMessage(message_builder(id, 2, msg_resetNode));
}


// -----------------------------------------------------------------------------

bool roboticslab::TechnosoftIpos::interpretMessage(const yarp::dev::CanMessage & message)
{
    uint16_t op = message.getId() - canId;

    switch (op)
    {
    case 0x580: // SDO
        can->sdo()->notify(message.getData());
        return true;
    case 0x180: // PDO1
        return can->tpdo1()->accept(message.getData(), message.getLen());
    case 0x80: // EMCY
        can->emcy()->accept(message.getData());
        return true;
    }

    CD_ERROR("Unknown message: %s\n", CanUtils::msgToStr(message).c_str());
    return false;

}

// -----------------------------------------------------------------------------
