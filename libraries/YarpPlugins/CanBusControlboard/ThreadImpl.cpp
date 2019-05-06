// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusControlboard.hpp"

#include <limits>

#include <yarp/os/Time.h>

// ------------------ Thread Related -----------------------------------------

void roboticslab::CanBusControlboard::run()
{
    CD_INFO("Started CanBusControlboard reading thread run.\n");

    while ( ! this->isStopping() )
    {
        //-- Lend CPU time to write threads.
        // https://github.com/roboticslab-uc3m/yarp-devices/issues/191
        yarp::os::Time::delay(std::numeric_limits<double>::min());

        unsigned int read;

        //-- Blocks with timeout until a message arrives, returns false on errors.
        bool ok = iCanBus->canRead(canInputBuffer, canRxBufferSize, &read, true);

        //-- All debugging messages should be contained in canRead, so just loop again.
        if( !ok || read == 0 ) continue;

        for (int i = 0; i < read; i++)
        {
            const yarp::dev::CanMessage &msg = canInputBuffer[i];
            int canId = msg.getId() & 0x7F;

            //-- Commenting next line as way too verbose, happens all the time.
            //CD_DEBUG("Read ok. %s\n", msgToStr(msg).c_str());

            std::map< int, int >::iterator idxFromCanIdFound = idxFromCanId.find(canId);

            if( idxFromCanIdFound == idxFromCanId.end() )  //-- Can ID not found
            {
                //-- Intercept 700h 0 msg that just indicates presence.
                if( msg.getId() - canId == 0x700 )
                {
                    CD_SUCCESS("Device indicating presence. %s\n", msgToStr(msg).c_str());
                    continue;
                }

                //CD_ERROR("Read CAN message from unknown device!!! %s\n", msgToStr(msg).c_str()); // --Commented this line to avoid filling the screen with error messages
                continue;
            }

            //CD_DEBUG("idxFromCanIdFound->second: %d\n",idxFromCanIdFound->second);
            iCanBusSharer[ idxFromCanIdFound->second ]->interpretMessage(msg);  //-- Check if false?
        }
    }  //-- ends: while ( ! this->isStopping() ).

    CD_INFO("Stopping CanBusControlboard reading thread run.\n");
}

// -----------------------------------------------------------------------------
