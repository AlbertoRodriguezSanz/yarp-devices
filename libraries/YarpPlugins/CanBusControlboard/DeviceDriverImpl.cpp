// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusControlboard.hpp"

#include <yarp/os/Property.h>
#include <yarp/dev/CanBusInterface.h>

#include <ColorDebug.h>

#include "ICanBusSharer.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

bool CanBusControlboard::open(yarp::os::Searchable & config)
{
    CD_DEBUG("%s\n", config.toString().c_str());

    if (!config.check("robotConfig") || !config.find("robotConfig").isBlob())
    {
        CD_ERROR("Missing \"robotConfig\" property or not a blob.\n");
        return false;
    }

    const auto * robotConfig = *reinterpret_cast<yarp::os::Property * const *>(config.find("robotConfig").asBlob());

    int parallelCanThreadLimit = config.check("parallelCanThreadLimit", yarp::os::Value(0), "parallel CAN TX thread limit").asInt32();

    if (parallelCanThreadLimit > 0)
    {
        deviceMapper.enableParallelization(parallelCanThreadLimit);
    }

    yarp::os::Bottle * canBuses = config.find("buses").asList();

    if (canBuses == nullptr)
    {
        CD_ERROR("Missing key \"buses\" or not a list.\n");
        return false;
    }

    canThreads.resize(canBuses->size());

    for (int i = 0; i < canBuses->size(); i++)
    {
        std::string canBus = canBuses->get(i).asString();
        yarp::os::Bottle & canBusGroup = robotConfig->findGroup(canBus);

        if (canBusGroup.isNull())
        {
            CD_ERROR("Missing CAN bus device group %s.\n", canBus.c_str());
            return false;
        }

        yarp::os::Property canBusOptions;
        canBusOptions.fromString(canBusGroup.toString());
        canBusOptions.put("robotConfig", config.find("robotConfig"));
        canBusOptions.put("canBlockingMode", false); // enforce non-blocking mode
        canBusOptions.put("canAllowPermissive", false); // always check usage requirements
        canBusOptions.setMonitor(config.getMonitor(), canBus.c_str());

        yarp::dev::PolyDriver * canBusDevice = new yarp::dev::PolyDriver;
        busDevices.push(canBusDevice, canBus.c_str());

        if (!canBusDevice->open(canBusOptions))
        {
            CD_ERROR("canBusDevice instantiation not worked.\n");
            return false;
        }

        yarp::dev::ICanBus * iCanBus;

        if (!canBusDevice->view(iCanBus))
        {
            CD_ERROR("Cannot view ICanBus interface in device: %s.\n", canBus.c_str());
            return false;
        }

        yarp::dev::ICanBufferFactory * iCanBufferFactory;

        if (!canBusDevice->view(iCanBufferFactory))
        {
            CD_ERROR("Cannot view ICanBufferFactory interface in device: %s.\n", canBus.c_str());
            return false;
        }

        int rxBufferSize = canBusGroup.check("rxBufferSize", yarp::os::Value(100), "CAN bus RX buffer size").asInt32();
        int txBufferSize = canBusGroup.check("txBufferSize", yarp::os::Value(100), "CAN bus TX buffer size").asInt32();
        double rxDelay = canBusGroup.check("rxDelay", yarp::os::Value(0.0), "CAN bus RX delay (seconds)").asFloat64();
        double txDelay = canBusGroup.check("txDelay", yarp::os::Value(0.0), "CAN bus TX delay (seconds)").asFloat64();

        CanReaderThread * reader = new CanReaderThread(canBus);
        reader->setCanHandles(iCanBus, iCanBufferFactory, rxBufferSize);
        reader->setDelay(rxDelay);

        CanWriterThread * writer = new CanWriterThread(canBus);
        writer->setCanHandles(iCanBus, iCanBufferFactory, txBufferSize);
        writer->setDelay(txDelay);

        canThreads[i].busName = canBus;
        canThreads[i].reader = reader;
        canThreads[i].writer = writer;

        yarp::os::Bottle * nodes = config.find(canBus).asList();

        if (nodes == nullptr)
        {
            CD_ERROR("Missing key \"%s\" or not a list.\n", canBus.c_str());
            return false;
        }

        for (int i = 0; i < nodes->size(); i++)
        {
            std::string node = nodes->get(i).asString();
            yarp::os::Bottle & nodeGroup = robotConfig->findGroup(node);

            if (nodeGroup.isNull())
            {
                CD_ERROR("Missing CAN node device group %s.\n", node.c_str());
                return false;
            }

            yarp::os::Property nodeOptions;
            nodeOptions.fromString(nodeGroup.toString());
            nodeOptions.put("robotConfig", config.find("robotConfig"));
            nodeOptions.setMonitor(config.getMonitor(), node.c_str());

            yarp::dev::PolyDriver * device = new yarp::dev::PolyDriver;
            nodeDevices.push(device, node.c_str());

            if (!device->open(nodeOptions))
            {
                CD_ERROR("CAN node device %s configuration failure.\n", node.c_str());
                return false;
            }

            if (!deviceMapper.registerDevice(device))
            {
                CD_ERROR("Unable to register device %s.\n", node.c_str());
                return false;
            }

            ICanBusSharer * iCanBusSharer;

            if (!device->view(iCanBusSharer))
            {
                CD_ERROR("Unable to view ICanBusSharer in %s.\n", node.c_str());
                return false;
            }

            reader->registerHandle(iCanBusSharer);
            iCanBusSharer->registerSender(writer->getDelegate());

            if (!iCanBus->canIdAdd(iCanBusSharer->getId()))
            {
                CD_ERROR("Unable to register acceptance filter in %s.\n", node.c_str());
                return false;
            }
        }
    }

    for (const auto & bundle : canThreads)
    {
        if (!bundle.reader->start())
        {
            CD_ERROR("Unable to start reader thread.\n");
            return false;
        }

        if (!bundle.writer->start())
        {
            CD_ERROR("Unable to start writer thread.\n");
            return false;
        }
    }

    for (int i = 0; i > nodeDevices.size(); i++)
    {
        ICanBusSharer * iCanBusSharer;
        nodeDevices[i]->poly->view(iCanBusSharer);

        if (!iCanBusSharer->initialize())
        {
            CD_ERROR("Node device %s could not initialize CAN comms.\n", nodeDevices[i]->key.c_str());
            return false;
        }
    }

    linInterpPeriodMs = config.check("linInterpPeriodMs", yarp::os::Value(DEFAULT_LIN_INTERP_PERIOD_MS), "linear interpolation mode period (milliseconds)").asInt32();
    linInterpBufferSize = config.check("linInterpBufferSize", yarp::os::Value(DEFAULT_LIN_INTERP_BUFFER_SIZE), "linear interpolation mode buffer size").asInt32();
    linInterpMode = config.check("linInterpMode", yarp::os::Value(DEFAULT_LIN_INTERP_MODE), "linear interpolation mode (pt/pvt)").asString();

    posdThread = new PositionDirectThread(deviceMapper, linInterpPeriodMs * 0.001);
    posdThread->start();

    return true;
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::close()
{
    bool ok = true;

    if (posdThread && posdThread->isRunning())
    {
        posdThread->stop();
    }

    delete posdThread;

    for (int i = 0; i < nodeDevices.size(); i++)
    {
        ICanBusSharer * iCanBusSharer;
        nodeDevices[i]->poly->view(iCanBusSharer);

        if (!iCanBusSharer->finalize())
        {
            CD_WARNING("Node device %s could not finalize CAN comms.\n", nodeDevices[i]->key.c_str());
            ok = false;
        }

        ok &= nodeDevices[i]->poly->close();
        delete nodeDevices[i]->poly;
    }

    for (const auto & bundle : canThreads)
    {
        if (bundle.reader && bundle.reader->isRunning())
        {
            ok &= bundle.reader->stop();
        }

        delete bundle.reader;

        if (bundle.writer && bundle.writer->isRunning())
        {
            ok &= bundle.writer->stop();
        }

        delete bundle.writer;
    }

    for (int i = 0; i < busDevices.size(); i++)
    {
        yarp::dev::ICanBus * iCanBus;
        busDevices[i]->poly->view(iCanBus);

        // Clear CAN acceptance filters ('0' = all IDs that were previously set by canIdAdd).
        if (!iCanBus->canIdDelete(0))
        {
            CD_WARNING("CAN filters on bus %s may be preserved on the next run.\n", busDevices[i]->key.c_str());
        }

        ok &= busDevices[i]->poly->close();
        delete busDevices[i]->poly;
    }

    return ok;
}

// -----------------------------------------------------------------------------
