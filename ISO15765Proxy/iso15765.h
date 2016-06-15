#pragma once

#ifndef __ISO15765_H
#define __ISO15765_H

#include "internal.h"

class ISO15765Transfer;

typedef std::shared_ptr <ISO15765Transfer> ISO15765TransferPtr;
typedef std::weak_ptr <ISO15765Transfer> ISO15765TransferWeakPtr;

class MessageFilterISO15765;

typedef std::shared_ptr <MessageFilterISO15765> MessageFilterISO15765Ptr;
typedef std::weak_ptr <MessageFilterISO15765> MessageFilterISO15765WeakPtr;

class ChannelISO15765;

typedef std::shared_ptr <ChannelISO15765> ChannelISO15765Ptr;
typedef std::weak_ptr <ChannelISO15765> ChannelISO15765WeakPtr;

class DeviceISO15765;

typedef std::shared_ptr <DeviceISO15765> DeviceISO15765Ptr;
typedef std::weak_ptr <DeviceISO15765> DeviceISO15765WeakPtr;

class LibraryISO15765;

typedef std::shared_ptr <LibraryISO15765> LibraryISO15765Ptr;
typedef std::weak_ptr <LibraryISO15765> LibraryISO15765WeakPtr;


class LibraryISO15765: public Library {
public:
	virtual ~LibraryISO15765();
};

class DeviceISO15765: public Device {
public:
	virtual ~DeviceISO15765();
};

class ChannelISO15765: public Channel {
	friend class ISO15765Transfer;
public:
    ChannelISO15765(const ChannelPtr &channel);

    virtual ~ChannelISO15765();
    
    virtual MessageFilterPtr startMsgFilter(unsigned long FilterType, PASSTHRU_MSG *pMaskMsg, PASSTHRU_MSG *pPatternMsg,
                                         PASSTHRU_MSG *pFlowControlMsg);

    virtual void stopMsgFilter(const MessageFilterPtr &messageFilter);
    
    virtual void readMsgs(PASSTHRU_MSG *pMsg, unsigned long *pNumMsgs, unsigned long Timeout);

    virtual void writeMsgs(PASSTHRU_MSG *pMsg, unsigned long *pNumMsgs, unsigned long Timeout);
    
    virtual unsigned long startPeriodicMsg(PASSTHRU_MSG *pMsg, unsigned long TimeInterval);

    virtual void stopPeriodicMsg(unsigned long periodicMessage);

    virtual DeviceWeakPtr getDevice() const;
    
    virtual void ioctl(unsigned long IoctlID, void *pInput, void *pOutput);
    
    virtual int getBs() const;
    
    virtual int getStmin() const;
    
protected:
    ChannelPtr mChannel;
	std::list<MessageFilterISO15765Ptr> mMessageFilters;
    
    int bs;
    int stmin;
    
    std::shared_ptr<ISO15765Transfer> getTransferByFlowControl(const PASSTHRU_MSG &msg);
    std::shared_ptr<ISO15765Transfer> getTransferByPattern(const PASSTHRU_MSG &msg);
};
 
class ISO15765Transfer {
public:
    ISO15765Transfer(ChannelISO15765 &channel, const PASSTHRU_MSG &pMaskMsg, const PASSTHRU_MSG &pPatternMsg, const PASSTHRU_MSG &pFlowControlMsg);
    ~ISO15765Transfer();
    
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

    ChannelISO15765 &mChannel;
    uint32_t mMaskPid;
    uint32_t mPatternPid;
    uint32_t mFlowControlPid;
    
    int mBs;
    int mStmin;
    
    unsigned int mSequence;
    PASSTHRU_MSG mMessage;
    int mMessageBs;
    TransferState mState;
    off_t mOffset;
};

class MessageFilterISO15765: public MessageFilter {
public:
    MessageFilterISO15765(const ChannelISO15765Ptr &channel, const MessageFilterPtr &messageFilter, const ISO15765TransferPtr &transfer);
	virtual ~MessageFilterISO15765();
	
	virtual ChannelWeakPtr getChannel() const override;
	
	virtual ISO15765TransferPtr& getTransfer();
	
private:
    ChannelISO15765WeakPtr mChannel;
    MessageFilterPtr mMessageFilter;
    ISO15765TransferPtr mTransfer;
};


ChannelPtr createISO15765Channel(const ChannelPtr &channel);

#endif //__ISO15765_H