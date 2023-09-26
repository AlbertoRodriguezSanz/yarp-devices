// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "Jr3Mbed.hpp"

#include <cstdint>
#include <cstring>

#include <algorithm> // std::copy
#include <tuple>

#include <yarp/os/LogStream.h>
#include <yarp/os/SystemClock.h>

#include "LogComponent.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

namespace
{
    std::tuple<std::array<std::int16_t, 3>, std::uint16_t> parseData(const can_message & message)
    {
        std::uint64_t data;
        std::memcpy(&data, message.data, message.len);

        return {{
            static_cast<std::int16_t>(data & 0x000000000000FFFF),
            static_cast<std::int16_t>((data & 0x00000000FFFF0000) >> 16),
            static_cast<std::int16_t>((data & 0x0000FFFF00000000) >> 32)
        }, static_cast<std::uint16_t>((data & 0xFFFF000000000000) >> 48)};
    }
}

// -----------------------------------------------------------------------------

unsigned int Jr3Mbed::getId()
{
    return canId;
}

// -----------------------------------------------------------------------------

bool Jr3Mbed::initialize()
{
    switch (mode)
    {
    case SYNC:
        return sendStartSyncCommand(filter);
    case ASYNC:
        return sendStartAsyncCommand(filter, asyncPeriod);
    default:
        yCIError(JR3M, id()) << "Unknown mode:" << static_cast<int>(mode);
        return false;
    }
}

// -----------------------------------------------------------------------------

bool Jr3Mbed::finalize()
{
    return sendStopCommand();
}

// -----------------------------------------------------------------------------

bool Jr3Mbed::notifyMessage(const can_message & message)
{
    const unsigned int op = (message.id - canId) >> 7;

    switch (op)
    {
    case JR3_BOOTUP:
    {
        yCIInfo(JR3M, id()) << "Bootup message received";
        isBooting = true;
        // can't block here, let the monitor thread call the initialization routine
        return true;
    }
    case JR3_ACK:
        return ackStateObserver->notify();
    case JR3_GET_FORCES:
    {
        auto [forces, counter] = parseData(message);
        std::lock_guard lock(mtx);
        buffer = forces;
        integrityCounter = counter;
        return true;
    }
    case JR3_GET_MOMENTS:
    {
        auto [moments, counter] = parseData(message);

        if (std::lock_guard lock(mtx); counter == integrityCounter)
        {
            std::copy(buffer.cbegin(), buffer.cend(), raw.begin());
            std::copy(moments.cbegin(), moments.cend(), raw.begin() + 3);
            timestamp = yarp::os::SystemClock::nowSystem();
        }

        return true;
    }
    default:
        yCIWarning(JR3M, id()) << "Unsupported operation:" << op;
        return false;
    }
}

// -----------------------------------------------------------------------------

bool Jr3Mbed::registerSender(ICanSenderDelegate * sender)
{
    this->sender = sender;
    return true;
}

// -----------------------------------------------------------------------------

bool Jr3Mbed::synchronize(double timestamp)
{
    return true;
}

// -----------------------------------------------------------------------------
