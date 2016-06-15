#pragma once

#ifndef _SIMPLE_H
#define _SIMPLE_H

#include "internal.h"
#include <memory>

class MessageFilterSimple;

typedef std::shared_ptr <MessageFilterSimple> MessageFilterSimplePtr;
typedef std::weak_ptr <MessageFilterSimple> MessageFilterSimpleWeakPtr;

class ChannelSimple;

typedef std::shared_ptr <ChannelSimple> ChannelSimplePtr;
typedef std::weak_ptr <ChannelSimple> ChannelSimpleWeakPtr;

class DeviceSimple;

typedef std::shared_ptr <DeviceSimple> DeviceSimplePtr;
typedef std::weak_ptr <DeviceSimple> DeviceSimpleWeakPtr;

class LibrarySimple;

typedef std::shared_ptr <LibrarySimple> LibrarySimplePtr;
typedef std::weak_ptr <LibrarySimple> LibrarySimpleWeakPtr;

class LibrarySimple : public Library {
public:
    LibrarySimple(j2534_fcts *proxy);

    virtual ~LibrarySimple();

    virtual DevicePtr open(void *pName) override;

    virtual void close(const DevicePtr &devicePtr) override;

    virtual void getLastError(char *pErrorDescription) override;
	
	// New

    virtual j2534_fcts* getProxy() const;
	
protected:
    virtual DevicePtr createDevice(void *pName, unsigned long deviceId);

    std::list<DevicePtr> mDevices;
    j2534_fcts *mProxy;
};

class DeviceSimple : public Device {
public:
    DeviceSimple(const LibrarySimplePtr &library, unsigned long deviceId);

    virtual ~DeviceSimple();

    virtual ChannelPtr connect(unsigned long ProtocolID, unsigned long Flags, unsigned long BaudRate) override;

    virtual void disconnect(const ChannelPtr &channelPtr) override;

    virtual void setProgrammingVoltage(unsigned long PinNumber, unsigned long Voltage) override;

    virtual void readVersion(char *pFirmwareVersion, char *pDllVersion, char *pApiVersion) override;

    virtual void ioctl(unsigned long IoctlID, void *pInput, void *pOutput) override;

    virtual LibraryWeakPtr getLibrary() const override;
	
	// New

    virtual LibrarySimpleWeakPtr getLibrarySimple() const;
	
protected:
    virtual ChannelPtr createChannel(unsigned long ProtocolID, unsigned long Flags, unsigned long BaudRate, unsigned long channelId);

    LibrarySimpleWeakPtr mLibrary;
    std::list<ChannelPtr> mChannels;
    unsigned long mDeviceId;
};

class ChannelSimple : public Channel {
	friend class MessageFilterSimple;
public:
    ChannelSimple(const DeviceSimplePtr &device, unsigned long channelId);

    virtual ~ChannelSimple();

    virtual void readMsgs(PASSTHRU_MSG *pMsg, unsigned long *pNumMsgs, unsigned long Timeout) override;

    virtual void writeMsgs(PASSTHRU_MSG *pMsg, unsigned long *pNumMsgs, unsigned long Timeout) override;

    virtual unsigned long startPeriodicMsg(PASSTHRU_MSG *pMsg, unsigned long TimeInterval) override;

    virtual void stopPeriodicMsg(unsigned long periodicMessage) override;

    virtual MessageFilterPtr startMsgFilter(unsigned long FilterType, PASSTHRU_MSG *pMaskMsg, PASSTHRU_MSG *pPatternMsg,
                                         PASSTHRU_MSG *pFlowControlMsg) override;

    virtual void stopMsgFilter(const MessageFilterPtr &messageFilter) override;

    virtual void ioctl(unsigned long IoctlID, void *pInput, void *pOutput) override;

    virtual DeviceWeakPtr getDevice() const override;
	
	// New

    virtual DeviceSimpleWeakPtr getDeviceSimple() const;
	
protected:
	virtual MessageFilterPtr createMessageFilter(unsigned long FilterType, PASSTHRU_MSG *pMaskMsg, PASSTHRU_MSG *pPatternMsg, PASSTHRU_MSG *pFlowControlMsg, unsigned long messageFilterId);

    DeviceSimpleWeakPtr mDevice;
	std::list<MessageFilterPtr> mMessageFilters;
    unsigned long mChannelId;
};

class MessageFilterSimple : public MessageFilter {
public:
	MessageFilterSimple(const ChannelSimplePtr& channel, unsigned long messageFilterId);
	~MessageFilterSimple();

	virtual ChannelWeakPtr getChannel() const override;
protected:
    ChannelSimpleWeakPtr mChannel;
    unsigned long mMessageFilterId;
};

#endif //_SIMPLE_H
