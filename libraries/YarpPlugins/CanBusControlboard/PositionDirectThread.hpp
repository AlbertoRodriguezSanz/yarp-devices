// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __POSITION_DIRECT_THREAD_HPP__
#define __POSITION_DIRECT_THREAD_HPP__

#include <map>
#include <mutex>
#include <set>
#include <vector>

#include <yarp/os/PeriodicThread.h>
#include <yarp/os/Searchable.h>

#include <yarp/dev/IRemoteVariables.h>

#include "DeviceMapper.hpp"

namespace roboticslab
{

class PositionDirectThread : public yarp::os::PeriodicThread
{
public:
    PositionDirectThread(const DeviceMapper & deviceMapper);
    bool configure(const yarp::os::Searchable & config);
    bool updateControlModeRegister(const std::map<int, bool> & ctrl);

protected:
    virtual bool threadInit() override;
    virtual void run() override;

private:
    std::vector<yarp::dev::IRemoteVariablesRaw *> handles;
    std::set<int> activeIds;
    mutable std::mutex mutex;
};

} // namespace roboticslab

#endif // __POSITION_DIRECT_THREAD_HPP__
