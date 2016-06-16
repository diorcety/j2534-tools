#include "stdafx.h"
#include "simple.h"
#include "ISO15765Proxy.h"
#include "utils.h"
#include <assert.h>

J2534Exception::J2534Exception(long code) : mCode(code) {
}

long J2534Exception::code() const {
    return mCode;
}

const char *J2534Exception::what() const noexcept {
    static char buffer[256];
    snprintf(buffer, 256, "Error code: %ld", mCode);
    return buffer;
}

LibrarySimple::LibrarySimple(j2534_fcts *proxy) : mProxy(proxy) {

}

LibrarySimple::~LibrarySimple() {

}

DevicePtr LibrarySimple::open(void *pName) {
    unsigned long deviceID;

    long ret;
    ret = mProxy->passThruOpen(pName, &deviceID);
    if (ret != STATUS_NOERROR) {
        throw J2534Exception(ret);
    }
    DevicePtr device = createDevice(pName, deviceID);
    mDevices.push_back(device);
    return device;
}

void LibrarySimple::close(const DevicePtr &devicePtr) {
    mDevices.remove(devicePtr);
}

void LibrarySimple::getLastError(char *pErrorDescription) {
    long ret = mProxy->passThruGetLastError(pErrorDescription);
    if (ret != STATUS_NOERROR) {
        throw J2534Exception(ret);
    }
}

j2534_fcts* LibrarySimple::getProxy() const {
    return mProxy;
}

DevicePtr LibrarySimple::createDevice(void *pName, unsigned long deviceId) {
	UNUSED(pName);
	return std::make_shared<DeviceSimple>(std::static_pointer_cast<LibrarySimple>(shared_from_this()), deviceId);
}

DeviceSimple::DeviceSimple(const LibrarySimplePtr &library, unsigned long deviceId): mLibrary(library), mDeviceId(deviceId) {
}

DeviceSimple::~DeviceSimple() {
    LibrarySimplePtr library = mLibrary.lock();
    assert(library);

    long ret;
    ret = library->getProxy()->passThruClose(mDeviceId);
    if (ret != STATUS_NOERROR) {
        throw J2534Exception(ret);
    }
}

ChannelPtr DeviceSimple::connect(unsigned long ProtocolID, unsigned long Flags, unsigned long BaudRate) {
    unsigned long channelID;

    LibrarySimplePtr library = mLibrary.lock();
    assert(library);

    long ret = library->getProxy()->passThruConnect(mDeviceId, ProtocolID, Flags, BaudRate, &channelID);
    if (ret != STATUS_NOERROR) {
        throw J2534Exception(ret);
    }

    ChannelPtr channel = createChannel(ProtocolID, Flags, BaudRate, channelID);
    mChannels.push_back(channel);
    return channel;
}

void DeviceSimple::disconnect(const ChannelPtr &channelPtr) {
    mChannels.remove(channelPtr);
}

void DeviceSimple::setProgrammingVoltage(unsigned long PinNumber, unsigned long Voltage) {
    LibrarySimplePtr library = mLibrary.lock();
    assert(library);

    long ret;
    ret = library->getProxy()->passThruSetProgrammingVoltage(mDeviceId, PinNumber, Voltage);
    if (ret != STATUS_NOERROR) {
        throw J2534Exception(ret);
    }
}

void DeviceSimple::readVersion(char *pFirmwareVersion, char *pDllVersion, char *pApiVersion) {
    LibrarySimplePtr library = mLibrary.lock();
    assert(library);

    long ret;
    ret = library->getProxy()->passThruReadVersion(mDeviceId, pFirmwareVersion, pDllVersion, pApiVersion);
    if (ret != STATUS_NOERROR) {
        throw J2534Exception(ret);
    }
}

void DeviceSimple::ioctl(unsigned long IoctlID, void *pInput, void *pOutput) {
    LibrarySimplePtr library = mLibrary.lock();
    assert(library);

    long ret;
    ret = library->getProxy()->passThruIoctl(mDeviceId, IoctlID, pInput, pOutput);
    if (ret != STATUS_NOERROR) {
        throw J2534Exception(ret);
    }
}

LibraryWeakPtr DeviceSimple::getLibrary() const {
    return mLibrary;
}

LibrarySimpleWeakPtr DeviceSimple::getLibrarySimple() const {
    return mLibrary;
}

ChannelPtr DeviceSimple::createChannel(unsigned long ProtocolID, unsigned long Flags, unsigned long BaudRate, unsigned long channelId) {
	UNUSED(ProtocolID);
	UNUSED(Flags);
	UNUSED(BaudRate);
	return std::make_shared<ChannelSimple>(std::static_pointer_cast<DeviceSimple>(shared_from_this()), channelId);
}

ChannelSimple::ChannelSimple(const DeviceSimplePtr &device, unsigned long channelId): mDevice(device), mChannelId(channelId) {

}

ChannelSimple::~ChannelSimple() {
    DeviceSimplePtr device = mDevice.lock();
    assert(device);

    LibrarySimplePtr library = device->getLibrarySimple().lock();
    assert(library);

    long ret;
    ret = library->getProxy()->passThruDisconnect(mChannelId);
    if (ret != STATUS_NOERROR) {
        throw J2534Exception(ret);
    }
}

void ChannelSimple::readMsgs(PASSTHRU_MSG *pMsg, unsigned long *pNumMsgs, unsigned long Timeout) {
    DeviceSimplePtr device = mDevice.lock();
    assert(device);

    LibrarySimplePtr library = device->getLibrarySimple().lock();
    assert(library);

    long ret;
    ret = library->getProxy()->passThruReadMsgs(mChannelId, pMsg, pNumMsgs, Timeout);
    if (ret != STATUS_NOERROR) {
        throw J2534Exception(ret);
    }
}

void ChannelSimple::writeMsgs(PASSTHRU_MSG *pMsg, unsigned long *pNumMsgs, unsigned long Timeout) {
    DeviceSimplePtr device = mDevice.lock();
    assert(device);

    LibrarySimplePtr library = device->getLibrarySimple().lock();
    assert(library);

    long ret;
    ret = library->getProxy()->passThruWriteMsgs(mChannelId, pMsg, pNumMsgs, Timeout);
    if (ret != STATUS_NOERROR) {
        throw J2534Exception(ret);
    }
}

PeriodicMessagePtr ChannelSimple::startPeriodicMsg(PASSTHRU_MSG *pMsg, unsigned long TimeInterval) {
    unsigned long msgID;
    DeviceSimplePtr device = mDevice.lock();
    assert(device);

    LibrarySimplePtr library = device->getLibrarySimple().lock();
    assert(library);

    long ret;
    ret = library->getProxy()->passThruStartPeriodicMsg(mChannelId, pMsg, &msgID, TimeInterval);
    if (ret != STATUS_NOERROR) {
        throw J2534Exception(ret);
    }

	PeriodicMessagePtr periodicMessage = createPeriodicMessage(pMsg, TimeInterval, msgID);
    mPeriodicMessages.push_back(periodicMessage);
    return periodicMessage;
}

void ChannelSimple::stopPeriodicMsg(const PeriodicMessagePtr &periodicMessage) {
    mPeriodicMessages.remove(periodicMessage);
}

MessageFilterPtr ChannelSimple::startMsgFilter(unsigned long FilterType, PASSTHRU_MSG *pMaskMsg, PASSTHRU_MSG *pPatternMsg,
                                     PASSTHRU_MSG *pFlowControlMsg) {
    unsigned long messageFilterId;
    DeviceSimplePtr device = mDevice.lock();
    assert(device);

    LibrarySimplePtr library = device->getLibrarySimple().lock();
    assert(library);

    long ret;
    ret = library->getProxy()->passThruStartMsgFilter(mChannelId, FilterType, pMaskMsg ,pPatternMsg, pFlowControlMsg, &messageFilterId);
    if (ret != STATUS_NOERROR) {
        throw J2534Exception(ret);
    }
	
	MessageFilterPtr messageFilter = createMessageFilter(FilterType, pMaskMsg, pPatternMsg, pFlowControlMsg, messageFilterId);
    mMessageFilters.push_back(messageFilter);
    return messageFilter;
}

void ChannelSimple::stopMsgFilter(const MessageFilterPtr &messageFilter) {
    mMessageFilters.remove(messageFilter);
}

void ChannelSimple::ioctl(unsigned long IoctlID, void *pInput, void *pOutput) {
    DeviceSimplePtr device = mDevice.lock();
    assert(device);

    LibrarySimplePtr library = device->getLibrarySimple().lock();
    assert(library);

    long ret;
    ret = library->getProxy()->passThruIoctl(mChannelId, IoctlID, pInput, pOutput);
    if (ret != STATUS_NOERROR) {
        throw J2534Exception(ret);
    }
}

DeviceWeakPtr ChannelSimple::getDevice() const {
    return mDevice;
}

DeviceSimpleWeakPtr ChannelSimple::getDeviceSimple() const {
    return mDevice;
}

MessageFilterPtr ChannelSimple::createMessageFilter(unsigned long FilterType, PASSTHRU_MSG *pMaskMsg, PASSTHRU_MSG *pPatternMsg, PASSTHRU_MSG *pFlowControlMsg, unsigned long messageFilterId) {
	UNUSED(FilterType);
	UNUSED(pMaskMsg);
	UNUSED(pPatternMsg);
	UNUSED(pFlowControlMsg);
    return std::make_shared<MessageFilterSimple>(std::static_pointer_cast<ChannelSimple>(shared_from_this()), messageFilterId);
}

PeriodicMessagePtr ChannelSimple::createPeriodicMessage(PASSTHRU_MSG *pMsg, unsigned long TimeInterval, unsigned long periodicMessageId) {
	UNUSED(pMsg);
	UNUSED(TimeInterval);
    return std::make_shared<PeriodicMessageSimple>(std::static_pointer_cast<ChannelSimple>(shared_from_this()), periodicMessageId);
}

/*
 * MessageFilterSimple
 */

MessageFilterSimple::MessageFilterSimple(const ChannelSimplePtr& channel, unsigned long messageFilterId) : mChannel(channel), mMessageFilterId(messageFilterId) {
	
}

MessageFilterSimple::~MessageFilterSimple() {
    ChannelSimplePtr channel = mChannel.lock();
    assert(channel);
	
    DeviceSimplePtr device = channel->getDeviceSimple().lock();
    assert(device);

    LibrarySimplePtr library = device->getLibrarySimple().lock();
    assert(library);

    long ret;
    ret = library->getProxy()->passThruStopMsgFilter(channel->mChannelId, mMessageFilterId);
    if (ret != STATUS_NOERROR) {
        throw J2534Exception(ret);
    }
}

ChannelWeakPtr MessageFilterSimple::getChannel() const {
	return mChannel;
}

/*
 * PeriodicMessageSimple
 */

PeriodicMessageSimple::PeriodicMessageSimple(const ChannelSimplePtr& channel, unsigned long periodicMessageId) : mChannel(channel), mPeriodicMessageId(periodicMessageId) {

}

PeriodicMessageSimple::~PeriodicMessageSimple() {
    ChannelSimplePtr channel = mChannel.lock();
    assert(channel);
	
    DeviceSimplePtr device = channel->getDeviceSimple().lock();
    assert(device);

    LibrarySimplePtr library = device->getLibrarySimple().lock();
    assert(library);

    long ret;
    ret = library->getProxy()->passThruStopPeriodicMsg(channel->mChannelId, mPeriodicMessageId);
    if (ret != STATUS_NOERROR) {
        throw J2534Exception(ret);
    }
}

ChannelWeakPtr PeriodicMessageSimple::getChannel() const {
    return mChannel;
}