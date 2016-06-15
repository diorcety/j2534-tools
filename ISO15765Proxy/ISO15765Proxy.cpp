#include "stdafx.h"
#include "../gateway/gateway.h"
#include "ISO15765Proxy.h"
#include "log.h"
#include "internal.h"
#include "simple.h"
#include <assert.h>

LibraryPtr library;

void create_library(j2534_fcts *proxy) {
    library = std::make_shared<LibrarySimple>(proxy);
}

void delete_library() {
    library = nullptr;
}

///////////////////////////////////// PassThruFunctions /////////////////////////////////////////////////

long J2534_API PassThruOpen(void *pName, unsigned long *pDeviceID) {
    LOG(INIT, "PassThruOpen");

    long ret = STATUS_NOERROR;
    try {
        DevicePtr device = library->open(pName);
        *pDeviceID = reinterpret_cast<unsigned long>(device.get());
    } catch (J2534Exception &exception) {
        ret = exception.code();
    }

    return ret;
}

long J2534_API PassThruClose(unsigned long DeviceID) {
    LOG(INIT, "PassThruClose");

    long ret = STATUS_NOERROR;
    try {
        Device *device = reinterpret_cast<Device *>(DeviceID);
        LibraryPtr library = device->getLibrary().lock();

        assert(library);

        library->close(device->shared_from_this());
    } catch (J2534Exception &exception) {
        ret = exception.code();
    }

    return ret;
}

long J2534_API PassThruConnect(unsigned long DeviceID, unsigned long ProtocolID, unsigned long Flags,
                               unsigned long Baudrate, unsigned long *pChannelID) {
    LOG(INIT, "PassThruConnect");

    long ret = STATUS_NOERROR;
    try {
        Device *device = reinterpret_cast<Device *>(DeviceID);
        ChannelPtr channel = device->connect(ProtocolID, Flags, Baudrate);
        *pChannelID = reinterpret_cast<unsigned long>(channel.get());
    } catch (J2534Exception &exception) {
        ret = exception.code();
    }

    return ret;
}

long J2534_API PassThruDisconnect(unsigned long ChannelID) {
    LOG(INIT, "PassThruDisconnect");

    long ret = STATUS_NOERROR;
    try {
        Channel *channel = reinterpret_cast<Channel *>(ChannelID);
        DevicePtr device = channel->getDevice().lock();

        assert(device);

        device->disconnect(channel->shared_from_this());
    } catch (J2534Exception &exception) {
        ret = exception.code();
    }

    return ret;
}


long J2534_API PassThruReadMsgs(unsigned long ChannelID, PASSTHRU_MSG *pMsg, unsigned long *pNumMsgs,
                                unsigned long Timeout) {
    LOG(INIT, "PassThruReadMsgs");

    long ret = STATUS_NOERROR;
    try {
        Channel *channel = reinterpret_cast<Channel *>(ChannelID);

        channel->readMsgs(pMsg, pNumMsgs, Timeout);
    } catch (J2534Exception &exception) {
        ret = exception.code();
    }

    return ret;
}


long J2534_API PassThruWriteMsgs(unsigned long ChannelID, PASSTHRU_MSG *pMsg, unsigned long *pNumMsgs,
                                 unsigned long Timeout) {
    LOG(INIT, "PassThruWriteMsgs");

    long ret = STATUS_NOERROR;
    try {
        Channel *channel = reinterpret_cast<Channel *>(ChannelID);

        channel->writeMsgs(pMsg, pNumMsgs, Timeout);
    } catch (J2534Exception &exception) {
        ret = exception.code();
    }

    return ret;
}


long J2534_API PassThruStartPeriodicMsg(unsigned long ChannelID, PASSTHRU_MSG *pMsg, unsigned long *pMsgID,
                                        unsigned long TimeInterval) {
    LOG(INIT, "PassThruStartPeriodicMsg");

    long ret = STATUS_NOERROR;
    try {
        Channel *channel = reinterpret_cast<Channel *>(ChannelID);

        *pMsgID = channel->startPeriodicMsg(pMsg, TimeInterval);
    } catch (J2534Exception &exception) {
        ret = exception.code();
    }

    return ret;
}


long J2534_API PassThruStopPeriodicMsg(unsigned long ChannelID, unsigned long MsgID) {
    LOG(INIT, "PassThruStopPeriodicMsg");

    long ret = STATUS_NOERROR;
    try {
        Channel *channel = reinterpret_cast<Channel *>(ChannelID);

        channel->stopPeriodicMsg(MsgID);
    } catch (J2534Exception &exception) {
        ret = exception.code();
    }

    return ret;
}


long J2534_API PassThruStartMsgFilter(unsigned long ChannelID, unsigned long FilterType, PASSTHRU_MSG *pMaskMsg,
                                      PASSTHRU_MSG *pPatternMsg, PASSTHRU_MSG *pFlowControlMsg,
                                      unsigned long *pFilterID) {
    LOG(INIT, "PassThruStartMsgFilter");

    long ret = STATUS_NOERROR;
    try {
        Channel *channel = reinterpret_cast<Channel *>(ChannelID);

        MessageFilterPtr messageFilter = channel->startMsgFilter(FilterType, pMaskMsg, pPatternMsg, pFlowControlMsg);
		*pFilterID = reinterpret_cast<unsigned long>(messageFilter.get());
    } catch (J2534Exception &exception) {
        ret = exception.code();
    }

    return ret;
}


long J2534_API PassThruStopMsgFilter(unsigned long ChannelID, unsigned long FilterID) {
    LOG(INIT, "PassThruStopMsgFilter");

    long ret = STATUS_NOERROR;
    try {
        Channel *channel = reinterpret_cast<Channel *>(ChannelID);
		MessageFilter *messageFilter = reinterpret_cast<MessageFilter *>(FilterID);

        channel->stopMsgFilter(messageFilter->shared_from_this());
    } catch (J2534Exception &exception) {
        ret = exception.code();
    }

    return ret;
}


long J2534_API PassThruSetProgrammingVoltage(unsigned long DeviceID, unsigned long PinNumber, unsigned long Voltage) {
    LOG(INIT, "PassThruSetProgrammingVoltage");

    long ret = STATUS_NOERROR;

    try {
        Device *device = reinterpret_cast<Device *>(DeviceID);
        device->setProgrammingVoltage(PinNumber, Voltage);
    } catch (J2534Exception &exception) {
        ret = exception.code();
    }

    return ret;
}


long J2534_API PassThruReadVersion(unsigned long DeviceID, char *pFirmwareVersion, char *pDllVersion,
                                   char *pApiVersion) {
    LOG(INIT, "PassThruReadVersion");

    long ret = STATUS_NOERROR;

    try {
        Device *device = reinterpret_cast<Device *>(DeviceID);
        device->readVersion(pFirmwareVersion, pDllVersion, pApiVersion);
    } catch (J2534Exception &exception) {
        ret = exception.code();
    }

    return ret;
}


long J2534_API PassThruGetLastError(char *pErrorDescription) {
    LOG(INIT, "PassThruGetLastError");

    long ret = STATUS_NOERROR;

    try {
        library->getLastError(pErrorDescription);
    } catch (J2534Exception &exception) {
        ret = exception.code();
    }

    return ret;
}


long J2534_API PassThruIoctl(unsigned long ChannelID, unsigned long IoctlID, void *pInput, void *pOutput) {
    LOG(INIT, "PassThruIoctl");

    long ret = STATUS_NOERROR;

    try {
        if (IoctlID == READ_PROG_VOLTAGE || IoctlID == READ_VBATT) {
            Device *device = reinterpret_cast<Device *> (ChannelID);
            device->ioctl(IoctlID, pInput, pOutput);
        } else {
            Channel *channel = reinterpret_cast<Channel *> (ChannelID);
            channel->ioctl(IoctlID, pInput, pOutput);
        }
    } catch (J2534Exception &exception) {
        ret = exception.code();
    }
    return ret;
}