// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "PlaybackThread.hpp"

namespace teo {

// -----------------------------------------------------------------------------

void PlaybackThread::run()
{

    while ( ! this->isStopping() )
    {
        //CD_DEBUG("\n");
        if ( getState() == PLAYING )
        {
            //CD_DEBUG("PLAYING. %d\n", this->getIter() );
            std::vector<double> row;

            if( ! this->getNext(row) )
            {
                stopPlay();
                CD_INFO("End of rows, auto stopPlay()\n");
                continue;
            }

            if( initTime != initTime )
            {
                initTime = yarp::os::Time::now();
                initRow = row[timeIdx];
            }
            else
            {
                yarp::os::Time::delay( initTime + (row[timeIdx] - initRow) - yarp::os::Time::now() );
            }

            std::cout << "Row[" << this->getIter() << "]: ";
            for(int i=0;i<row.size();i++)
            {
                std::cout << row[i] << " ";
            }
            std::cout << std::endl;
        }  // if ( getState() == PLAYING )
    }  // while ( ! this->isStopping() )
}

// -----------------------------------------------------------------------------

void PlaybackThread::onStop()
{
    setState( NOT_PLAYING );
    //CD_DEBUG("Bye!\n");
}

// -----------------------------------------------------------------------------

}  // namespace teo
