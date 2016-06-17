#include "stdafx.h"
#include "../gateway/gateway.h"
#include "MVCIProxy.h"
#include "log.h"
#include <stdlib.h>
#include <string.h>

static j2534_fcts *last_ret_proxy = NULL;

#define LAST_RET(proxy, ret) last_ret_proxy = proxy;

class Device;

class Channel {
public:
    Channel(Device *device, unsigned long channelId) : device(device), channelId(channelId) {
    }

    Device *device;
    unsigned long channelId;
};

class Device {
public:
    Device(j2534_fcts *proxy, unsigned long deviceId) : proxy(proxy), deviceId(deviceId) {
    }

    j2534_fcts *proxy;
    unsigned long deviceId;
};

class Devices {
public:
    Devices(Device *device1, Device *device2 = NULL) : device1(device1), device2(device2) {
    }

    Device *device1;
    Device *device2;
};

static unsigned long filterProtocol(unsigned long protocol) {
    if(protocol == CAN_PS) {
        return CAN;
    } else if (protocol == ISO15765_PS) {
        return ISO15765;
    }
    return protocol;
}

static PASSTHRU_MSG* filterPS(const PASSTHRU_MSG *msg) {
    if(msg==NULL) {
        return NULL;
    }
    PASSTHRU_MSG *msg2 = (PASSTHRU_MSG*) malloc(sizeof(PASSTHRU_MSG));
    memcpy(msg2, msg, sizeof(PASSTHRU_MSG));
    msg2->ProtocolID = filterProtocol(msg2->ProtocolID);
    return msg2;
}

static PASSTHRU_MSG* filterPSs(const PASSTHRU_MSG *msg, unsigned long count) {
    if(msg==NULL) {
        return NULL;
    }
    PASSTHRU_MSG *msg2 = (PASSTHRU_MSG*) malloc(sizeof(PASSTHRU_MSG) * count);
    memcpy(msg2, msg, sizeof(PASSTHRU_MSG) * count);
    for(unsigned long i = 0; i < count; ++i) {
        msg2[i].ProtocolID = filterProtocol(msg2[i].ProtocolID);
    }
    return msg2;
}


///////////////////////////////////// PassThruFunctions /////////////////////////////////////////////////

long MVCI_PROXY_API PassThruOpen(void *pName, unsigned long *pDeviceID) {
    LOG(INIT, "PassThruOpen");

    long ret;
    SetDeviceToOpen(0);
    ret = proxy1->passThruOpen(pName, pDeviceID);
    LAST_RET(proxy1, ret);
    if (ret != STATUS_NOERROR) {
        return ret;
    }
    Device *device1 = new Device(proxy1, *pDeviceID);
    Device *device2 = NULL;
#ifdef ENABLE_ALTERNATE
    SetDeviceToOpen(1);
    ret = proxy2->passThruOpen(pName, pDeviceID);
    LAST_RET(proxy2, ret);
    if (ret != STATUS_NOERROR) {
        device1->proxy->passThruClose(device1->deviceId);
        delete device1;
        return ret;
    }
    device2 = new Device(proxy2, *pDeviceID);
#endif /* ENABLE_ALTERNATE */

    Devices *devices = new Devices(device1, device2);
    *pDeviceID = reinterpret_cast<unsigned long>(devices);
    return ret;
}

long MVCI_PROXY_API PassThruClose(unsigned long DeviceID) {
    LOG(INIT, "PassThruClose");
    Devices *devices = reinterpret_cast<Devices *>(DeviceID);

    long ret;
    DeviceID = devices->device1->deviceId;
    ret = devices->device1->proxy->passThruClose(DeviceID);
    LAST_RET(devices->device1->proxy, ret);
    delete devices->device1;
#ifdef ENABLE_ALTERNATE
    DeviceID = devices->device2->deviceId;
    ret = devices->device2->proxy->passThruClose(DeviceID);
    LAST_RET(devices->device2->proxy, ret);
    delete devices->device2;
#endif /* ENABLE_ALTERNATE */

    delete devices;
    return ret;
}

long MVCI_PROXY_API PassThruConnect(unsigned long DeviceID, unsigned long ProtocolID, unsigned long Flags,
                               unsigned long Baudrate, unsigned long *pChannelID) {
    LOG(INIT, "PassThruConnect");
    Devices *devices = reinterpret_cast<Devices *> (DeviceID);
    Device *device;
    if (ProtocolID & 0x8000) {
#ifdef ENABLE_ALTERNATE
        ProtocolID = filterProtocol(ProtocolID);
        device = devices->device2;
#else
        return ERR_NOT_SUPPORTED;
#endif /* ENABLE_ALTERNATE */
    } else {
        device = devices->device1;
    }

    // Create the channel
    j2534_fcts *proxy = device->proxy;
    DeviceID = device->deviceId;
    long ret = proxy->passThruConnect(DeviceID, ProtocolID, Flags, Baudrate, pChannelID);
    Channel *c = new Channel(device, *pChannelID);
    *pChannelID = reinterpret_cast<unsigned long>(c);
    LAST_RET(proxy, ret);
    return ret;
}

long MVCI_PROXY_API PassThruDisconnect(unsigned long ChannelID) {
    LOG(INIT, "PassThruDisconnect");
    Channel *channel = reinterpret_cast<Channel *> (ChannelID);
    Device *device = channel->device;
    j2534_fcts *proxy = device->proxy;
    ChannelID = channel->channelId;
    long ret;
    ret = channel->device->proxy->passThruDisconnect(ChannelID);
    delete channel;
    LAST_RET(proxy, ret);
    return ret;
}


long MVCI_PROXY_API PassThruReadMsgs(unsigned long ChannelID, PASSTHRU_MSG *pMsg, unsigned long *pNumMsgs,
                                unsigned long Timeout) {
    LOG(INIT, "PassThruReadMsgs");
    Channel *channel = reinterpret_cast<Channel *> (ChannelID);
    Device *device = channel->device;
    j2534_fcts *proxy = device->proxy;
    ChannelID = channel->channelId;
    long ret;
    ret = proxy->passThruReadMsgs(ChannelID, pMsg, pNumMsgs, Timeout);
    LAST_RET(proxy, ret);
    return ret;
}


long MVCI_PROXY_API PassThruWriteMsgs(unsigned long ChannelID, PASSTHRU_MSG *pMsg, unsigned long *pNumMsgs,
                                 unsigned long Timeout) {
    LOG(INIT, "PassThruWriteMsgs");
    Channel *channel = reinterpret_cast<Channel *> (ChannelID);
    Device *device = channel->device;
    j2534_fcts *proxy = device->proxy;
    ChannelID = channel->channelId;

    // Filter
    pMsg = filterPSs(pMsg, *pNumMsgs);

    long ret;
    ret = proxy->passThruWriteMsgs(ChannelID, pMsg, pNumMsgs, Timeout);
    LAST_RET(proxy, ret);

    // Free filter
    free(pMsg);

    return ret;
}


long MVCI_PROXY_API PassThruStartPeriodicMsg(unsigned long ChannelID, PASSTHRU_MSG *pMsg, unsigned long *pMsgID,
                                        unsigned long TimeInterval) {
    LOG(INIT, "PassThruStartPeriodicMsg");
    Channel *channel = reinterpret_cast<Channel *> (ChannelID);
    Device *device = channel->device;
    j2534_fcts *proxy = device->proxy;
    ChannelID = channel->channelId;

    // Filter
    pMsg = filterPS(pMsg);

    long ret;
    ret = proxy->passThruStartPeriodicMsg(ChannelID, pMsg, pMsgID, TimeInterval);
    LAST_RET(proxy, ret);

    // Free filter
    free(pMsg);

    return ret;
}


long MVCI_PROXY_API PassThruStopPeriodicMsg(unsigned long ChannelID, unsigned long MsgID) {
    LOG(INIT, "PassThruStopPeriodicMsg");
    Channel *channel = reinterpret_cast<Channel *> (ChannelID);
    Device *device = channel->device;
    j2534_fcts *proxy = device->proxy;
    ChannelID = channel->channelId;
    long ret;
    ret = proxy->passThruStopPeriodicMsg(ChannelID, MsgID);
    LAST_RET(proxy, ret);
    return ret;
}


long MVCI_PROXY_API PassThruStartMsgFilter(unsigned long ChannelID, unsigned long FilterType, PASSTHRU_MSG *pMaskMsg,
                                      PASSTHRU_MSG *pPatternMsg, PASSTHRU_MSG *pFlowControlMsg,
                                      unsigned long *pFilterID) {
    LOG(INIT, "PassThruStartMsgFilter");
    Channel *channel = reinterpret_cast<Channel *> (ChannelID);
    Device *device = channel->device;
    j2534_fcts *proxy = device->proxy;
    ChannelID = channel->channelId;

    // Filter
    pMaskMsg = filterPS(pMaskMsg);
    pPatternMsg = filterPS(pPatternMsg);
    if(FilterType == FLOW_CONTROL_FILTER) {
        pFlowControlMsg = filterPS(pFlowControlMsg);
    }

    long ret;
    ret = proxy->passThruStartMsgFilter(ChannelID, FilterType, pMaskMsg, pPatternMsg, pFlowControlMsg, pFilterID);
    LAST_RET(proxy, ret);

    // Free filter
    free(pMaskMsg);
    free(pPatternMsg);
    if(FilterType == FLOW_CONTROL_FILTER) {
        free(pFlowControlMsg);
    }

    return ret;
}


long MVCI_PROXY_API PassThruStopMsgFilter(unsigned long ChannelID, unsigned long FilterID) {
    LOG(INIT, "PassThruStopMsgFilter");
    Channel *channel = reinterpret_cast<Channel *> (ChannelID);
    Device *device = channel->device;
    j2534_fcts *proxy = device->proxy;
    ChannelID = channel->channelId;
    long ret;
    ret = proxy->passThruStopMsgFilter(ChannelID, FilterID);
    LAST_RET(proxy, ret);
    return ret;
}


long MVCI_PROXY_API PassThruSetProgrammingVoltage(unsigned long DeviceID, unsigned long PinNumber, unsigned long Voltage) {
    LOG(INIT, "PassThruSetProgrammingVoltage");
    Devices *devices = reinterpret_cast<Devices *>(DeviceID);
    DeviceID = devices->device1->deviceId;
    long ret;
    ret = proxy1->passThruSetProgrammingVoltage(DeviceID, PinNumber, Voltage);
    LAST_RET(proxy1, ret);
    return ret;
}


long MVCI_PROXY_API PassThruReadVersion(unsigned long DeviceID, char *pFirmwareVersion, char *pDllVersion,
                                   char *pApiVersion) {
    LOG(INIT, "PassThruReadVersion");
    Devices *devices = reinterpret_cast<Devices *>(DeviceID);
    DeviceID = devices->device1->deviceId;
    long ret;
    ret = proxy1->passThruReadVersion(DeviceID, pFirmwareVersion, pDllVersion, pApiVersion);
    LAST_RET(proxy1, ret);
    return ret;
}


long MVCI_PROXY_API PassThruGetLastError(char *pErrorDescription) {
    LOG(INIT, "PassThruGetLastError");
    if (last_ret_proxy != NULL) {
        return last_ret_proxy->passThruGetLastError(pErrorDescription);
    }
    return STATUS_NOERROR;
}


long MVCI_PROXY_API PassThruIoctl(unsigned long ChannelID, unsigned long IoctlID, void *pInput, void *pOutput) {
    LOG(INIT, "PassThruIoctl");
    if(IoctlID == READ_PROG_VOLTAGE || IoctlID == READ_VBATT) {
        Devices *devices = reinterpret_cast<Devices *> (ChannelID);
        Device *device = devices->device1;
        j2534_fcts *proxy = device->proxy;
        ChannelID = device->deviceId;
        long ret;
        ret = proxy->passThruIoctl(ChannelID, IoctlID, pInput, pOutput);
        LAST_RET(proxy, ret);
        return ret;
    } else {
        Channel *channel = reinterpret_cast<Channel *> (ChannelID);
        Device *device = channel->device;
        j2534_fcts *proxy = device->proxy;
        ChannelID = channel->channelId;
        long ret;
        ret = proxy->passThruIoctl(ChannelID, IoctlID, pInput, pOutput);
        LAST_RET(proxy, ret);
        return ret;
    }
}