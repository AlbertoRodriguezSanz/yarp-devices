// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
 * @ingroup yarp_devices_examples_cpp
 * @defgroup exampleRemoteControlBoard exampleRemoteControlBoard
 *
 * @brief This example connects to a remote control board device (e.g. @ref CanBusBroker).
 *
 * <b>Legal</b>
 *
 * Copyright: (C) 2010 Universidad Carlos III de Madrid;
 *            (C) 2010 RobotCub Consortium
 *
 * Author: Juan G Victores
 *
 * Contribs: Paul Fitzpatrick and Giacomo Spigler (YARP dev/motortest.cpp example)
 *
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see license/LGPL.TXT
 *
 * <b>Building</b>
\verbatim
cd repos/asibot-main/example/cpp
mkdir build; cd build; cmake ..
make -j3
\endverbatim
 *
 * Usage (showing default option values):
@verbatim
 [terminal 1] yarpserver --write
 [terminal 2] sudo ip link set can0 up txqueuelen 1000 type can bitrate 1000000
 [terminal 2] candump can0
 [terminal 3] repos/sofia-yarp-devices/share/contexts/launchCanBus or the directory where launchCanBus is located
 [terminal 3] YARP_ROBOT_NAME=teoSoftGripper launchCanBus --from softGripper.ini
 [terminal 4]-> [For controlling only a couple of joints]: exampleGripperGrasp --remote /teo/softGripper --joint "(5 6)" 
 [terminal 4]-> [For controlling all joints]: exampleGripperGrasp --remote /teo/softGripper  --period 50
@endverbatim
 *
 */

/**
 * @example{lineno} exampleGripperGrasp.cpp
 */

#include <cstdio>

#include <vector>

#include <yarp/os/Network.h>
#include <yarp/os/Property.h>
#include <yarp/os/Time.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/LogStream.h>

#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/IControlMode.h>
#include <yarp/dev/IEncodersTimed.h>
#include <yarp/dev/IPositionControl.h>
#include <yarp/dev/IVelocityControl.h>

constexpr auto DEFAULT_REMOTE = "/teo/softGripper";

int main(int argc, char *argv[])
{
    std::printf("INFO: requires a running robot counterpart.\n");
    yarp::os::Network yarp;

    if (!yarp::os::Network::checkNetwork())
    {
        std::printf("Please start a yarp name server first\n");
        return 1;
    }

    yarp::os::ResourceFinder rf;
    rf.configure(argc, argv);
    auto remote = rf.check("remote", yarp::os::Value(DEFAULT_REMOTE), "remote port").asString();

    yarp::os::Property options 
    {{"device", yarp::os::Value("remote_controlboard")}, //This is the client which connects to controlboard_nws 
     {"local", yarp::os::Value("/exampleGripperGrasp")},
     {"remote", yarp::os::Value(remote)}};

    yarp::dev::PolyDriver dd(options);

    if (!dd.isValid())
    {
        std::printf("Device not available.\n");
        return 1;
    }

    yarp::dev::IPositionControl *pos;
    yarp::dev::IVelocityControl *vel;
    yarp::dev::IEncodersTimed *enc;
    yarp::dev::IControlMode *mode;


    if (!dd.view(mode) || !dd.view(enc) || !dd.view(pos))   //vel is not implemented for this example
    {
        yError() << "Unable to acquire robot interfaces";
        return 1;
    }


    std::printf("SUCCESS: Acquired robot interface\n");


    if (!mode->setControlModes(std::vector(9, VOCAB_CM_POSITION).data()))
    {
        yError() << "Unable to set position control mode";
        return 1;
    }

    //Brute-force homing
    std::vector<double> q(9,0.0);
    q = {10.0, 10.0, 0.0, 10.0, 10.0, 0.0, 10.0, 10.0, 0.0};
    double *speed;
    pos->getRefSpeed(q[0], speed);
    std::printf("Speed: ", *speed);
    std::printf("homing done!\n");
    pos->positionMove( q.data() );
    std::printf("homing done!\n");
    
    bool done = false;
    /*
    do 
    {
        if (!pos->checkMotionDone(&done))
        {
            yError() << "checkMotionDone failed";
            break;
        }

        yarp::os::Time::delay(0.1);
    }
    while (!done);
    done = false;
    */

    yarp::os::Time::delay(3);
    yInfo() << "motion done";

    q = {2.0, 2.0, 0.0, 2.0, 2.0, 0.0, 2.0, 2.0, 0.0};
    pos->positionMove( q.data() );
    std::printf("homing done!\n");

    do 
    {
        if (!pos->checkMotionDone(&done))
        {
            yError() << "checkMotionDone failed";
            break;
        }

        yarp::os::Time::delay(0.1);
    }
    while (!done);
    done = false;
    yInfo() << "motion done";

    //Moving to one sideee
    if (!mode->setControlModes(std::vector(9, VOCAB_CM_IDLE ).data()))
    {
        yError() << "Unable to set idle control mode";
        return 1;
    }
    std::printf("Idle control mode set\n");
    if (!mode->setControlModes(std::vector(9, VOCAB_CM_POSITION ).data()))
    {
        yError() << "Unable to set position control mode";
        return 1;
    }
    std::printf("Position control mode set\n");
    
    q = {1.0, 1.0, 0.0, 1.0, 1.0, 0.0, 1.0, 1.0, 0.0};
    pos->positionMove( q.data() );
    std::printf("homing done!\n");


    do 
    {
        if (!pos->checkMotionDone(&done))
        {
            yError() << "checkMotionDone failed";
            break;
        }

        yarp::os::Time::delay(0.1);
    }
    while (!done);
    done = false;
    yInfo() << "motion done";

    q = {0.0, 130.0, 0.0, 0.0, 130.0, 0.0, 0.0, 130.0, 0.0};
    pos->positionMove( q.data() );
    std::printf("mooooving!\n");
    /*
    do 
    {
        if (!pos->checkMotionDone(&done))
        {
            yError() << "checkMotionDone failed";
            break;
        }

        yarp::os::Time::delay(0.1);
    }
    while (!done);
    done = false;
    yInfo() << "motion done\n";

    */

   
   yarp::os::Time::delay(10);

    //Mooooving back to the center
    q = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    pos->positionMove( q.data() );
    std::printf("mooooving back to the center!\n");

    do 
    {
        if (!pos->checkMotionDone(&done))
        {
            yError() << "checkMotionDone failed";
            break;//Moooving to the other side
        }
    }
    while (!done);
    done = false;
    yInfo() << "motion done";

    q = {130.0, 0.0, 0.0, 130.0, 0.0, 0.0, 130.0, 0.0, 0.0};
    pos->positionMove( q.data() );
    std::printf("mooooving!\n");
    /*
    do 
    {
        if (!pos->checkMotionDone(&done))
        {
            yError() << "checkMotionDone failed";
            break;
        }

        yarp::os::Time::delay(0.1);
    }
    while (!done);
    done = false;
    yInfo() << "motion done\n";

    */

   
   yarp::os::Time::delay(10);

    //Mooooving back to the center
    q = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    pos->positionMove( q.data() );
    std::printf("mooooving back to the center!\n");
    do 
    {
        if (!pos->checkMotionDone(&done))
        {
            yError() << "checkMotionDone failed";
            break;
        }

        yarp::os::Time::delay(0.1);
    }
    while (!done);
    done = false;
    yInfo() << "motion done\n";



    return 0;
}
