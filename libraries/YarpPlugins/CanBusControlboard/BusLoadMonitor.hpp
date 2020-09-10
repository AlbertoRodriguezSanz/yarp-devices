// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __BUS_LOAD_MONITOR_HPP__
#define __BUS_LOAD_MONITOR_HPP__

#include <atomic>

#include <yarp/os/Bottle.h>
#include <yarp/os/PeriodicThread.h>
#include <yarp/os/PortWriterBuffer.h>

#include "CanMessageNotifier.hpp"

namespace roboticslab
{

/**
 * @ingroup CanBusControlboard
 * @brief Registers load statistics per single CAN bus.
 */
class OneWayMonitor : public CanMessageNotifier
{
public:
    //! Tell observers a new CAN message has arrived.
    virtual bool notifyMessage(const can_message & msg) override;

    //! Return stored value and clear counter.
    unsigned int reset();

private:
    std::atomic<unsigned int> bits;
};

/**
 * @ingroup CanBusControlboard
 * @brief Periodically sends CAN bus load stats through a YARP port.
 */
class BusLoadMonitor final : public yarp::os::PeriodicThread,
                             public yarp::os::PortWriterBuffer<yarp::os::Bottle>
{
public:
    //! Constructor.
    BusLoadMonitor(double period) : yarp::os::PeriodicThread(period), bitrate(1.0)
    { }

    void setBitrate(unsigned int bitrate)
    { this->bitrate = bitrate; }

    CanMessageNotifier * getReadMonitor()
    { return &readMonitor; }

    CanMessageNotifier * getWriteMonitor()
    { return &writeMonitor; }

protected:
    //! The thread will invoke this periodically.
    virtual void run() override;

private:
    unsigned int bitrate;

    OneWayMonitor readMonitor;
    OneWayMonitor writeMonitor;
};

} // namespace roboticslab

#endif // __BUS_LOAD_MONITOR_HPP__
