// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "TechnosoftIpos.hpp"

#include <yarp/conf/version.h>
#include <yarp/os/Log.h>

#include "LogComponent.hpp"

using namespace roboticslab;

// -------------------------------------------------------------------------------------

bool TechnosoftIpos::getRefTorqueRaw(int j, double * t)
{
#if defined(YARP_VERSION_COMPARE) // >= 3.6.0
    yCITrace(IPOS, id(), "%d", j);
#else
    yCTrace(IPOS, "%d", j);
#endif
    CHECK_JOINT(j);
    CHECK_MODE(VOCAB_CM_TORQUE);
    *t = vars.synchronousCommandTarget;
    return true;
}

// -------------------------------------------------------------------------------------

bool TechnosoftIpos::getRefTorquesRaw(double * t)
{
    return getRefTorqueRaw(0, &t[0]);
}

// -------------------------------------------------------------------------------------

bool TechnosoftIpos::setRefTorqueRaw(int j, double t)
{
#if defined(YARP_VERSION_COMPARE) // >= 3.6.0
    yCITrace(IPOS, id(), "%d %f", j, t);
#else
    yCTrace(IPOS, "%d %f", j, t);
#endif
    CHECK_JOINT(j);
    CHECK_MODE(VOCAB_CM_TORQUE);
    vars.synchronousCommandTarget = t;
    return true;
}

// -------------------------------------------------------------------------------------

bool TechnosoftIpos::setRefTorquesRaw(const double * t)
{
    return setRefTorqueRaw(0, t[0]);
}

// -------------------------------------------------------------------------------------

bool TechnosoftIpos::getTorqueRaw(int j, double * t)
{
#if defined(YARP_VERSION_COMPARE) // >= 3.6.0
    yCITrace(IPOS, id(), "%d", j);
#else
    yCTrace(IPOS, "%d", j);
#endif
    CHECK_JOINT(j);
    std::int16_t temp = vars.lastCurrentRead;
    double curr = vars.internalUnitsToCurrent(temp);
    *t = vars.currentToTorque(curr);
    return true;
}

// -------------------------------------------------------------------------------------

bool TechnosoftIpos::getTorquesRaw(double * t)
{
    return getTorqueRaw(0, &t[0]);
}

// -------------------------------------------------------------------------------------

bool TechnosoftIpos::getTorqueRangeRaw(int j, double * min, double * max)
{
#if defined(YARP_VERSION_COMPARE) // >= 3.6.0
    yCITrace(IPOS, id(), "%d", j);
#else
    yCTrace(IPOS, "%d", j);
#endif
    CHECK_JOINT(j);

    return can->sdo()->upload<std::uint16_t>("Current limit", [this, min, max](auto data)
        { double temp = vars.internalUnitsToPeakCurrent(data);
          *max = vars.currentToTorque(temp);
          *min = -(*max); },
        0x207F);
}

// -------------------------------------------------------------------------------------

bool TechnosoftIpos::getTorqueRangesRaw(double * min, double * max)
{
    return getTorqueRangeRaw(0, &min[0], &max[0]);
}

// -------------------------------------------------------------------------------------

bool TechnosoftIpos::getMotorTorqueParamsRaw(int j, yarp::dev::MotorTorqueParameters * params)
{
#if defined(YARP_VERSION_COMPARE) // >= 3.6.0
    yCITrace(IPOS, id(), "%d", j);
#else
    yCTrace(IPOS, "%d", j);
#endif
    CHECK_JOINT(j);

    params->bemf = 0.0;
    params->bemf_scale = 0.0;
    params->ktau = vars.k;
    params->ktau_scale = 0.0;
#if defined(YARP_VERSION_COMPARE) && (YARP_VERSION_MAJOR > 3 || YARP_VERSION_MINOR > 6) // >= 3.7.0
    params->viscousPos = 0.0;
    params->viscousNeg = 0.0;
    params->coulombPos = 0.0;
    params->coulombNeg = 0.0;
#endif

    return true;
}

// -------------------------------------------------------------------------------------

bool TechnosoftIpos::setMotorTorqueParamsRaw(int j, const yarp::dev::MotorTorqueParameters params)
{
#if defined(YARP_VERSION_COMPARE) // >= 3.6.0
    yCITrace(IPOS, id(), "%d", j);
#else
    yCTrace(IPOS, "%d", j);
#endif
    CHECK_JOINT(j);
    vars.k = params.ktau;
    return true;
}

// -------------------------------------------------------------------------------------
