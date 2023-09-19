// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusControlBoard.hpp"

#include <yarp/os/Log.h>

#include "LogComponent.hpp"

using namespace roboticslab;
using raw_t = yarp::dev::IVelocityControlRaw;

// -----------------------------------------------------------------------------

bool CanBusControlBoard::velocityMove(int j, double spd)
{
    yCTrace(CBCB, "%d %f", j, spd);
    CHECK_JOINT(j);
    return deviceMapper.mapSingleJoint<raw_t, double>(&yarp::dev::IVelocityControlRaw::velocityMoveRaw, j, spd);
}

// -----------------------------------------------------------------------------

bool CanBusControlBoard::velocityMove(const double * spds)
{
    yCTrace(CBCB, "");
    return deviceMapper.mapAllJoints(&yarp::dev::IVelocityControlRaw::velocityMoveRaw, spds);
}

// -----------------------------------------------------------------------------

bool CanBusControlBoard::velocityMove(int n_joint, const int * joints, const double * spds)
{
    yCTrace(CBCB, "%d", n_joint);
    return deviceMapper.mapJointGroup(&yarp::dev::IVelocityControlRaw::velocityMoveRaw, n_joint, joints, spds);
}

// -----------------------------------------------------------------------------

bool CanBusControlBoard::getRefVelocity(int joint, double * vel)
{
    yCTrace(CBCB, "%d", joint);
    CHECK_JOINT(joint);
    return deviceMapper.mapSingleJoint(&yarp::dev::IVelocityControlRaw::getRefVelocityRaw, joint, vel);
}

// ------------------------------------------------------------------------------

bool CanBusControlBoard::getRefVelocities(double * vels)
{
    yCTrace(CBCB, "");
    return deviceMapper.mapAllJoints(&yarp::dev::IVelocityControlRaw::getRefVelocitiesRaw, vels);
}

// -----------------------------------------------------------------------------

bool CanBusControlBoard::getRefVelocities(int n_joint, const int * joints, double * vels)
{
    yCTrace(CBCB, "%d", n_joint);
    return deviceMapper.mapJointGroup(&yarp::dev::IVelocityControlRaw::getRefVelocitiesRaw, n_joint, joints, vels);
}

// -----------------------------------------------------------------------------