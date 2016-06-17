#pragma once

#ifndef _INTERNAL_H_H
#define _INTERNAL_H_H

#include "j2534_v0404.h"
#include "utils.h"

DEFINE_SHARED(PeriodicMessage)
DEFINE_SHARED(MessageFilter)
DEFINE_SHARED(Channel)
DEFINE_SHARED(Device)
DEFINE_SHARED(Library)

class Configuration {
public:
    virtual ~Configuration();

    virtual bool getValue(unsigned long config, unsigned long *value) const = 0;

    virtual bool setValue(unsigned long config, unsigned long value) = 0;
};

class Library : public std::enable_shared_from_this<Library> {
public:
    virtual ~Library() = 0;

    virtual DevicePtr open(void *pName) = 0;

    virtual void close(const DevicePtr &devicePtr) = 0;

    virtual void getLastError(char *pErrorDescription) = 0;
};

class Device : public std::enable_shared_from_this<Device> {
public:
    virtual ~Device() = 0;

    virtual ChannelPtr connect(unsigned long ProtocolID, unsigned long Flags, unsigned long BaudRate) = 0;

    virtual void disconnect(const ChannelPtr &channelPtr) = 0;

    virtual void setProgrammingVoltage(unsigned long PinNumber, unsigned long Voltage) = 0;

    virtual void readVersion(char *pFirmwareVersion, char *pDllVersion, char *pApiVersion) = 0;

    virtual void ioctl(unsigned long IoctlID, void *pInput, void *pOutput) = 0;

    virtual LibraryWeakPtr getLibrary() const = 0;
};

class Channel : public std::enable_shared_from_this<Channel> {
public:
    virtual ~Channel() = 0;

    virtual void readMsgs(PASSTHRU_MSG *pMsg, unsigned long *pNumMsgs, unsigned long Timeout) = 0;

    virtual void writeMsgs(PASSTHRU_MSG *pMsg, unsigned long *pNumMsgs, unsigned long Timeout) = 0;

    virtual PeriodicMessagePtr startPeriodicMsg(PASSTHRU_MSG *pMsg, unsigned long TimeInterval) = 0;

    virtual void stopPeriodicMsg(const PeriodicMessagePtr &periodicMessage) = 0;

    virtual MessageFilterPtr startMsgFilter(unsigned long FilterType, PASSTHRU_MSG *pMaskMsg, PASSTHRU_MSG *pPatternMsg, PASSTHRU_MSG *pFlowControlMsg) = 0;

    virtual void stopMsgFilter(const MessageFilterPtr &messageFilter) = 0;

    virtual void ioctl(unsigned long IoctlID, void *pInput, void *pOutput) = 0;

    virtual DeviceWeakPtr getDevice() const = 0;
};

class MessageFilter : public std::enable_shared_from_this<MessageFilter> {
public:
    virtual ~MessageFilter() = 0;
    
    virtual ChannelWeakPtr getChannel() const = 0;
};

class PeriodicMessage : public std::enable_shared_from_this<PeriodicMessage> {
public:
    virtual ~PeriodicMessage() = 0;
    
    virtual ChannelWeakPtr getChannel() const = 0;
};

#endif //_INTERNAL_H_H
