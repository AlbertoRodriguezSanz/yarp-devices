// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "PlaybackThread.hpp"

// -----------------------------------------------------------------------------

void teo::PlaybackThread::run()
{
    int ret = ioctl(fd,IOCTL0_JR3_FILTER0,&fm);
    if ( ret != -1)
    {
        fmSemaphore.wait();

        f[0] = 100.0*(fm.f[0])*(fs.f[0])/16384.0;
        f[1] = 100.0*(fm.f[1])*(fs.f[1])/16384.0;
        f[2] = 100.0*(fm.f[2])*(fs.f[2])/16384.0;

        m[0] = 10.0*(fm.m[0])*(fs.m[0])/16384.0;
        m[1] = 10.0*(fm.m[1])*(fs.m[1])/16384.0;
        m[2] = 10.0*(fm.m[2])*(fs.m[2])/16384.0;

        fmSemaphore.post();
    }
}

// -----------------------------------------------------------------------------
