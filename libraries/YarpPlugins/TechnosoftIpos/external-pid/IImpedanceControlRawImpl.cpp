// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "external-pid/TechnosoftIposExternal.hpp"

#include <yarp/os/LogStream.h>

#include "LogComponent.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

bool TechnosoftIposExternal::getImpedanceRaw(int j, double * stiffness, double * damping)
{
    yCITrace(IPOS, id(), "%d", j);
    CHECK_JOINT(j);

    *stiffness = impedanceStiffness;
    *damping = impedanceDamping;

    return true;
}

// -----------------------------------------------------------------------------

bool TechnosoftIposExternal::setImpedanceRaw(int j, double stiffness, double damping)
{
    yCITrace(IPOS, id(), "%d %f %f", j, stiffness, damping);
    CHECK_JOINT(j);

    if (stiffness < minImpedanceStiffness || stiffness > maxImpedanceStiffness)
    {
        yCIError(IPOS, id(), "Invalid stiffness: %f (not in [%f, %f])", stiffness, minImpedanceStiffness, maxImpedanceStiffness);
        return false;
    }

    if (damping < minImpedanceDamping || damping > maxImpedanceDamping)
    {
        yCIError(IPOS, id(), "Invalid damping: %f (not in [%f, %f])", damping, minImpedanceDamping, maxImpedanceDamping);
        return false;
    }

    impedanceStiffness = stiffness;
    impedanceDamping = damping;
    return true;
}

// -----------------------------------------------------------------------------

bool TechnosoftIposExternal::setImpedanceOffsetRaw(int j, double offset)
{
    yCITrace(IPOS, id(), "%d %f", j, offset);
    CHECK_JOINT(j);

    if (offset < 0.0)
    {
        yCIError(IPOS, id()) << "Invalid offset:" << offset;
        return false;
    }

    impedanceOffset = offset;
    return true;
}

// -----------------------------------------------------------------------------

bool TechnosoftIposExternal::getImpedanceOffsetRaw(int j, double * offset)
{
    yCITrace(IPOS, id(), "%d", j);
    CHECK_JOINT(j);
    *offset = impedanceOffset;
    return true;
}

// -----------------------------------------------------------------------------

bool TechnosoftIposExternal::getCurrentImpedanceLimitRaw(int j, double * min_stiff, double * max_stiff, double * min_damp, double * max_damp)
{
    yCITrace(IPOS, id(), "%d", j);
    CHECK_JOINT(j);

    *min_stiff = minImpedanceStiffness;
    *max_stiff = maxImpedanceStiffness;
    *min_damp = minImpedanceDamping;
    *max_damp = maxImpedanceDamping;

    return true;
}

// -----------------------------------------------------------------------------
