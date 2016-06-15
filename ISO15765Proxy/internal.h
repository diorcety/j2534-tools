#pragma once

#ifndef _INTERNAL_H_H
#define _INTERNAL_H_H

#include "ISO15765Proxy.h"

#include <exception>
#include <list>
#include <memory>

class MessageFilter;

typedef std::shared_ptr <MessageFilter> MessageFilterPtr;
typedef std::weak_ptr <MessageFilter> MessageFilterWeakPtr;

class Channel;

typedef std::shared_ptr <Channel> ChannelPtr;
typedef std::weak_ptr <Channel> ChannelWeakPtr;

class Device;

typedef std::shared_ptr <Device> DevicePtr;
typedef std::weak_ptr <Device> DeviceWeakPtr;

class Library;

typedef std::shared_ptr <Library> LibraryPtr;
typedef std::weak_ptr <Library> LibraryWeakPtr;


class J2534Exception : public std::exception {
public:
    J2534Exception(long code);

    long code() const;

    virtual const char *what() const noexcept;

private:
    long mCode;
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

    virtual unsigned long startPeriodicMsg(PASSTHRU_MSG *pMsg, unsigned long TimeInterval) = 0;

    virtual void stopPeriodicMsg(unsigned long periodicMessage) = 0;

    virtual MessageFilterPtr startMsgFilter(unsigned long FilterType, PASSTHRU_MSG *pMaskMsg, PASSTHRU_MSG *pPatternMsg,
                                         PASSTHRU_MSG *pFlowControlMsg) = 0;

    virtual void stopMsgFilter(const MessageFilterPtr &messageFilter) = 0;

    virtual void ioctl(unsigned long IoctlID, void *pInput, void *pOutput) = 0;

    virtual DeviceWeakPtr getDevice() const = 0;
};

class MessageFilter : public std::enable_shared_from_this<MessageFilter> {
public:
    virtual ~MessageFilter() = 0;
	
    virtual ChannelWeakPtr getChannel() const = 0;
};

void create_library(j2534_fcts *proxy);

void delete_library();

#endif //_INTERNAL_H_H
