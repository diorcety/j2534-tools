#pragma once

#ifndef __ISO15765_H
#define __ISO15765_H

#include "internal.h"
#include "configurable_channel.h"
#include <list>

DEFINE_SHARED(TransferISO15765)
DEFINE_SHARED(MessageFilterISO15765)
DEFINE_SHARED(ChannelISO15765)
DEFINE_SHARED(DeviceISO15765)
DEFINE_SHARED(LibraryISO15765)

class LibraryISO15765: public Library {
public:
    LibraryISO15765(const LibraryPtr &library);
    
    virtual ~LibraryISO15765();
    
    virtual DevicePtr open(void *pName) override;

    virtual void close(const DevicePtr &devicePtr) override;

    virtual void getLastError(char *pErrorDescription) override;

protected:
    std::list<DevicePtr> mDevices;
    LibraryPtr mLibrary;
};

class DeviceISO15765: public Device {
    friend class LibraryISO15765;
public:
    DeviceISO15765(const LibraryISO15765Ptr &library, const DevicePtr &device);
    
    virtual ~DeviceISO15765();
    
    virtual ChannelPtr connect(unsigned long ProtocolID, unsigned long Flags, unsigned long BaudRate) override;

    virtual void disconnect(const ChannelPtr &channelPtr) override;

    virtual void setProgrammingVoltage(unsigned long PinNumber, unsigned long Voltage) override;

    virtual void readVersion(char *pFirmwareVersion, char *pDllVersion, char *pApiVersion) override;

    virtual void ioctl(unsigned long IoctlID, void *pInput, void *pOutput) override;

    virtual LibraryWeakPtr getLibrary() const override;
    
protected:
    LibraryISO15765WeakPtr mLibrary;
    std::list<ChannelPtr> mChannels;
    DevicePtr mDevice;
};

class ChannelISO15765: public ConfigurableChannel {
    friend class TransferISO15765;
    friend class DeviceISO15765;
public:
    ChannelISO15765(unsigned long protocolId, const DeviceISO15765Ptr &device, const ChannelPtr &channel);

    virtual ~ChannelISO15765();
    
    virtual MessageFilterPtr startMsgFilter(unsigned long FilterType, PASSTHRU_MSG *pMaskMsg, PASSTHRU_MSG *pPatternMsg,
                                         PASSTHRU_MSG *pFlowControlMsg) override;

    virtual void stopMsgFilter(const MessageFilterPtr &messageFilter) override;
    
    virtual void readMsgs(PASSTHRU_MSG *pMsg, unsigned long *pNumMsgs, unsigned long Timeout) override;

    virtual void writeMsgs(PASSTHRU_MSG *pMsg, unsigned long *pNumMsgs, unsigned long Timeout) override;
    
    virtual PeriodicMessagePtr startPeriodicMsg(PASSTHRU_MSG *pMsg, unsigned long TimeInterval) override;

    virtual void stopPeriodicMsg(const PeriodicMessagePtr &periodicMessage) override;
    
    virtual DeviceWeakPtr getDevice() const override;

protected:
    virtual bool getConfig(SCONFIG *config) const override;

    virtual bool setConfig(SCONFIG *config) override;
    
    virtual bool clearTxBuffers() override;

    virtual bool clearRxBuffers() override;

    virtual bool clearPeriodicMessages() override;

    virtual bool clearMessageFilters() override;
    
    virtual bool handle_ioctl(unsigned long IoctlID, void *pInput, void *pOutput) override;
    
    virtual void ioctl(unsigned long IoctlID, void *pInput, void *pOutput) override;
    
    TransferISO15765Ptr getTransferByFlowControl(const PASSTHRU_MSG &msg);
    
    TransferISO15765Ptr getTransferByPattern(const PASSTHRU_MSG &msg);
    
protected:
    unsigned long mProtocolId;
    DeviceISO15765WeakPtr mDevice;
    std::list<MessageFilterPtr> mMessageFilters;
    ChannelPtr mChannel;
};
 
class TransferISO15765 {
public:
    TransferISO15765(Configuration &configuration, Channel &channel, const PASSTHRU_MSG &pMaskMsg, const PASSTHRU_MSG &pPatternMsg, const PASSTHRU_MSG &pFlowControlMsg);
    ~TransferISO15765();
    
    void clear();
    
    bool writeMsg(const PASSTHRU_MSG &msg, unsigned long Timeout);
    bool readMsg(const PASSTHRU_MSG &in_msg, PASSTHRU_MSG &out_msg, unsigned long Timeout);
    
    uint32_t getMaskPid();
    uint32_t getPatternPid();
    uint32_t getFlowControlPid();

private:
    enum TransferState {
        START_STATE = 0,
        FLOW_CONTROL_STATE,
        BLOCK_STATE
    };

    enum PCIFrameName {
        SingleFrame = 0,
        FirstFrame,
        ConsecutiveFrame,
        FlowControl,
        UnknownFrame
    };
    
    static PCIFrameName getFrameName(uint8_t pci);
    static uint8_t getPci(PCIFrameName frameName);
    static size_t getRemainingSize(const PASSTHRU_MSG &msg, off_t offset);
    static void prepareSentMessageHeaders(PASSTHRU_MSG &out_msg, const PASSTHRU_MSG &in_msg);
    static void prepareReceivedMessageHeaders(PASSTHRU_MSG &out_msg, const PASSTHRU_MSG &in_msg);
    static void paddingMessage(PASSTHRU_MSG &smsg);
    
    bool sendFlowControlMessage(unsigned long Timeout);

    Configuration &mChannelConfiguration;
    Channel &mChannel;
    
    uint32_t mMaskPid;
    uint32_t mPatternPid;
    uint32_t mFlowControlPid;
    
    unsigned long mBs;
    unsigned long mStmin;
    
    unsigned int mSequence;
    PASSTHRU_MSG mMessage;
    TransferState mState;
    off_t mOffset;
};

class MessageFilterISO15765: public MessageFilter {
    friend class ChannelISO15765;
public:
    MessageFilterISO15765(const ChannelISO15765Ptr &channel, const MessageFilterPtr &messageFilter, const TransferISO15765Ptr &transfer);
    virtual ~MessageFilterISO15765();
    
    virtual ChannelWeakPtr getChannel() const override;
    
    virtual TransferISO15765Ptr& getTransfer();
    
private:
    ChannelISO15765WeakPtr mChannel;
    TransferISO15765Ptr mTransfer;
    MessageFilterPtr mMessageFilter;
};

#endif //__ISO15765_H