// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusControlBoard.hpp"

#include <yarp/os/Log.h>

#include "LogComponent.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

bool CanBusControlBoard::setPosition(int j, double ref)
{
    yCTrace(CBCB, "%d %f", j, ref);
    CHECK_JOINT(j);
    return deviceMapper.mapSingleJoint(&yarp::dev::IPositionDirectRaw::setPositionRaw, j, ref);
}

// -----------------------------------------------------------------------------

bool CanBusControlBoard::setPositions(const double * refs)
{
    yCTrace(CBCB, "");
    return deviceMapper.mapAllJoints(&yarp::dev::IPositionDirectRaw::setPositionsRaw, refs);
}

// -----------------------------------------------------------------------------

bool CanBusControlBoard::setPositions(int n_joint, const int * joints, const double * refs)
{
    yCTrace(CBCB, "%d", n_joint);
    return deviceMapper.mapJointGroup(&yarp::dev::IPositionDirectRaw::setPositionsRaw, n_joint, joints, refs);
}

// -----------------------------------------------------------------------------

bool CanBusControlBoard::getRefPosition(int joint, double * ref)
{
    yCTrace(CBCB, "%d", joint);
    CHECK_JOINT(joint);
    return deviceMapper.mapSingleJoint(&yarp::dev::IPositionDirectRaw::getRefPositionRaw, joint, ref);
}

// -----------------------------------------------------------------------------

bool CanBusControlBoard::getRefPositions(double * refs)
{
    yCTrace(CBCB, "");
    return deviceMapper.mapAllJoints(&yarp::dev::IPositionDirectRaw::getRefPositionsRaw, refs);
}

// -----------------------------------------------------------------------------

bool CanBusControlBoard::getRefPositions(int n_joint, const int * joints, double * refs)
{
    yCTrace(CBCB, "%d", n_joint);
    return deviceMapper.mapJointGroup(&yarp::dev::IPositionDirectRaw::getRefPositionsRaw, n_joint, joints, refs);
}

// -----------------------------------------------------------------------------