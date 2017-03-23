// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusControlboard.hpp"

// ------------------ IVelocityControl Related ----------------------------------------

bool teo::CanBusControlboard::setVelocityMode()
{
    CD_INFO("\n");

    bool ok = true;
    for(int j=0; j<nodes.size(); j++)
    {
        ok &= this->setVelocityMode(j);
    }
    return ok;
}

// -----------------------------------------------------------------------------

bool teo::CanBusControlboard::velocityMove(int j, double sp)
{
    CD_INFO("(%d), (%f)\n",j , sp);

    //-- Check index within range
    if ( ! this->indexWithinRange(j) ) return false;    

    return iVelocityControl2Raw[j]->velocityMoveRaw( 0, sp );
}

// -----------------------------------------------------------------------------

bool teo::CanBusControlboard::velocityMove(const double *sp)
{
    CD_INFO("\n");

    bool ok = true;
    for(int j=0; j<nodes.size(); j++)
    {
        ok &= this->velocityMove(j,sp[j]);
    }
    return ok;
}

// ----------------------------  IVelocityControl2 Related  --------------------

bool teo::CanBusControlboard::velocityMove(const int n_joint, const int *joints, const double *spds)
{
    CD_INFO("\n");

    bool ok = true;
    for(int j=0; j<nodes.size(); j++)
    {
        if( joints[j] )
        {
            ok &= this->velocityMove(j,spds[j]);
        }
    }
    return ok;
}

// -----------------------------------------------------------------------------

bool teo::CanBusControlboard::getRefVelocity(const int joint, double *vel)
{
    CD_INFO("%d\n",joint);

    //-- Check index within range
    if ( ! this->indexWithinRange(joint) ) return false;

    // -- Get the last reference speed set by velocityMove (see IVelocityControl2RawImpl.cpp contained in TechnosoftIpos ) for single joint
    return iVelocityControl2Raw[joint]->getRefVelocityRaw(0, vel); // -- It can be... getRefVelocityRaw(joint, vel)
}

// ------------------------------------------------------------------------------

bool teo::CanBusControlboard::getRefVelocities(double *vels)
{
    CD_INFO("\n");

    bool ok = true;
    for(unsigned int i=0; i<nodes.size(); i++)
    {
        ok &= getRefVelocity(i,&(vels[i]));
    }
    return ok;
}

// -----------------------------------------------------------------------------

bool teo::CanBusControlboard::getRefVelocities(const int n_joint, const int *joints, double *vels)
{
    CD_INFO("\n");

    bool ok = true;
    for(unsigned int i=0; i<nodes.size(); i++)
    {
        if( joints[i] )
        {
            ok &= getRefVelocity(i,&(vels[i]));
        }
    }
    return ok;
}

// -----------------------------------------------------------------------------

bool teo::CanBusControlboard::setVelPid(int j, const yarp::dev::Pid &pid)
{
    return true;
}

// -----------------------------------------------------------------------------

bool teo::CanBusControlboard::setVelPids(const yarp::dev::Pid *pids)
{
    return true;
}

// -----------------------------------------------------------------------------

bool teo::CanBusControlboard::getVelPid(int j, yarp::dev::Pid *pid)
{
    return true;
}

// -----------------------------------------------------------------------------

bool teo::CanBusControlboard::getVelPids(yarp::dev::Pid *pids)
{
    return true;
}
