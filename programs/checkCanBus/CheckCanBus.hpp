// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __CHECK_CAN_BUS__
#define __CHECK_CAN_BUS__

#include <yarp/os/RFModule.h>
#include <yarp/os/Module.h>
#include <yarp/os/Network.h>
#include <yarp/os/Port.h>
#include <yarp/os/BufferedPort.h>

#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/Wrapper.h>

#include <string>
#include <stdlib.h>

#include "ICanBusSharer.h"
#include "ColorDebug.hpp"


namespace teo
{

/**
 * @ingroup dumpCanBus //-- MODIFICAR
 *
 * @brief Launches one CAN bus driver, dumps output.
 *
 */

class CheckCanBus : public yarp::os::RFModule, public yarp::os::Thread {
    public:
        CheckCanBus();
        bool configure(yarp::os::ResourceFinder &rf);

        // -- Nuevas variables:
        int timeOut;    // -- tiempo de espera para comprobar el ID (s)
        int canNumber;  // -- Can que utilizaremos (0 o 1)

    protected:

        yarp::dev::PolyDriver deviceDevCan0; // -- Dispositivo que se crea (¿A qué dispositivo hace referencia?)
        CanBusHico* iCanBus;

        /** A helper function to display CAN messages. */
        std::string msgToStr(can_msg* message); // -- Muestra los mensajes que vienen del CAN
        double lastNow; // -- Muestra el tiempo actual

        virtual double getPeriod() {return 3.0;}
        virtual bool updateModule();
        virtual bool close();

//        virtual bool interruptModule();
//        virtual int period;

    // -------- Thread declarations. Implementation in ThreadImpl.cpp --------

        /**
         * Main body of the new thread.
         * Override this method to do what you want.
         * After Thread::start is called, this
         * method will start running in a separate thread.
         * It is important that this method either keeps checking
         * Thread::isStopping to see if it should stop, or
         * you override the Thread::onStop method to interact
         * with it in some way to shut the new thread down.
         * There is no really reliable, portable way to stop
         * a thread cleanly unless that thread cooperates.
         */
        virtual void run();
};

}  // namespace teo

#endif  // __CHECK_CAN_BUS__

