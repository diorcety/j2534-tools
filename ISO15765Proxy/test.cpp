#include <vector>
#include <list>
#include <mutex>
#include <condition_variable>
#include <thread>

#include <string.h>

#include "internal.h"
#include "iso15765.h"
#include "utils.h"

#define DEBUG

#ifdef DEBUG
#define LOG_DEBUG(...) printf(__VA_ARGS__); printf("\n"); fflush(stdout)
#else //DEBUG
#define LOG_DEBUG(...)
#endif //DEBUG

class ChannelTest;

typedef std::shared_ptr <ChannelTest> ChannelTestPtr;
typedef std::weak_ptr <ChannelTest> ChannelTestWeakPtr;

class MessageFilterTest;

typedef std::shared_ptr <MessageFilterTest> MessageFilterTestPtr;
typedef std::weak_ptr <MessageFilterTest> MessageFilterTestWeakPtr;

class Bus;

typedef std::shared_ptr <Bus> BusPtr;
typedef std::weak_ptr <Bus> BusWeakPtr;

class Bus: public std::enable_shared_from_this<Bus> {
    friend class ChannelTest;
public:
    Bus();
    ~Bus();
    void addChannel(const ChannelTestPtr &channel);
    void removeChannel(const ChannelTestPtr &channel);

    static void _run(Bus *bus);
protected:
    void run();
private:
    std::list<ChannelTestPtr> mChannels;

    bool mContinue;
    std::thread mThread;
    std::mutex mMutex;
    std::list<ChannelTestPtr> mIncommingMessageChannels;
    std::condition_variable mInterrupted;
};

class ChannelTest: public Channel {
    friend class Bus;
public:
    ChannelTest();
	
	virtual ~ChannelTest();

    virtual void readMsgs(PASSTHRU_MSG *pMsg, unsigned long *pNumMsgs, unsigned long Timeout) override;

    virtual void writeMsgs(PASSTHRU_MSG *pMsg, unsigned long *pNumMsgs, unsigned long Timeout) override;

    virtual PeriodicMessagePtr startPeriodicMsg(PASSTHRU_MSG *pMsg, unsigned long TimeInterval) override;

    virtual void stopPeriodicMsg(const PeriodicMessagePtr &periodicMessage) override;

    virtual MessageFilterPtr startMsgFilter(unsigned long FilterType, PASSTHRU_MSG *pMaskMsg, PASSTHRU_MSG *pPatternMsg,
                                         PASSTHRU_MSG *pFlowControlMsg) override;

    virtual void stopMsgFilter(const MessageFilterPtr &messageFilter) override;

    virtual void ioctl(unsigned long IoctlID, void *pInput, void *pOutput) override;

    virtual DeviceWeakPtr getDevice() const override;

private:
    BusWeakPtr mBus;

    std::list<PASSTHRU_MSG> mInBuffers;
    std::list<PASSTHRU_MSG> mOutBuffers;

    std::mutex mMutex;
    std::condition_variable mInterrupted;
};

class MessageFilterTest : public MessageFilter {
public:
	MessageFilterTest(const ChannelTestPtr& channel);

	virtual ChannelWeakPtr getChannel() const override;
protected:
    ChannelTestWeakPtr mChannel;
};

MessageFilterTest::MessageFilterTest(const ChannelTestPtr& channel): mChannel(channel) {

}

ChannelWeakPtr MessageFilterTest::getChannel() const {
	return mChannel;
}

Bus::Bus(): mContinue(true), mThread(_run, this) {
}

Bus::~Bus() {
    mContinue = false;
    {
        std::unique_lock<std::mutex> lck(mMutex);
        mInterrupted.notify_all();
    }
    mThread.join();
}

void Bus::_run(Bus *bus) {
	bus->run();
}

void Bus::addChannel(const ChannelTestPtr &channel) {
    std::unique_lock<std::mutex> lck (mMutex);
    channel->mBus = shared_from_this();
    mChannels.push_back(channel);
}
void Bus::removeChannel(const ChannelTestPtr &channel) {
    std::unique_lock<std::mutex> lck (mMutex);
    channel->mBus.reset();
    mChannels.remove(channel);
}

static void printMsg(PASSTHRU_MSG &msg) {
#ifdef DEBUG
    for(unsigned int i = 0; i < msg.DataSize; ++i) {
        printf("%02x ", msg.Data[i]);
    }
    printf("\n");
#else //DEBUG
    UNUSED(msg);
#endif //DEBUG
}

void Bus::run() {
    std::unique_lock<std::mutex> lck (mMutex);
    while(mContinue) {
        mInterrupted.wait(lck);

        while(!mIncommingMessageChannels.empty()) {
            ChannelTestPtr channel = mIncommingMessageChannels.front();

            std::unique_lock<std::mutex> lckC (channel->mMutex);

            while(!channel->mOutBuffers.empty()) {
                // Dispatch the incoming message to the other channel on the bus
                PASSTHRU_MSG &msg = channel->mOutBuffers.front();
                printMsg(msg);
                for(ChannelTestPtr &c: mChannels) {
                    if(c != channel) {
                        std::unique_lock<std::mutex> lckC2 (c->mMutex);
                        c->mInBuffers.push_back(msg);
                        c->mInterrupted.notify_all();
                        LOG_DEBUG("%p -> %p", (void*)channel.get(), (void*)c.get());
                    }
                }
                channel->mOutBuffers.pop_front();
            }

            channel->mInterrupted.notify_all();
            mIncommingMessageChannels.pop_front();
        }
    }
}


ChannelTest::ChannelTest() {
}

ChannelTest::~ChannelTest() {
	
}

void ChannelTest::readMsgs(PASSTHRU_MSG *pMsg, unsigned long *pNumMsgs, unsigned long Timeout) {
    BusPtr bus = mBus.lock();
    if(!bus) {
        LOG_DEBUG("No connected to bus");
        *pNumMsgs = 0;
        return;
    }

    // Set Deadline
    std::chrono::time_point<std::chrono::steady_clock> deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(Timeout);
    unsigned long count = 0;
    std::unique_lock<std::mutex> lck (mMutex);
	
    for(unsigned long i = 0; i < *pNumMsgs; ++i) {
        while(mInBuffers.empty()) {
            if(mInterrupted.wait_until(lck, deadline) == std::cv_status::timeout) {
				goto end;
            }
        }
        *(pMsg++) = mInBuffers.front();
        mInBuffers.pop_front();
		
		count++;
    }
end:
	*pNumMsgs = count;
	LOG_DEBUG("Read %ld message(s)", *pNumMsgs);
}

void ChannelTest::writeMsgs(PASSTHRU_MSG *pMsg, unsigned long *pNumMsgs, unsigned long Timeout) {
    BusPtr bus = mBus.lock();
    if(!bus) {
        LOG_DEBUG("No connected to bus");
		*pNumMsgs = 0;
        return;
    }

    // Set Deadline
    std::chrono::time_point<std::chrono::steady_clock> deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(Timeout);

    unsigned long count = 0;
    std::unique_lock<std::mutex> lck (mMutex);

    LOG_DEBUG("Send %ld message(s)", *pNumMsgs);
    for(unsigned long i = 0; i < *pNumMsgs; ++i) {
        mOutBuffers.push_back(*(pMsg++));
        bus->mIncommingMessageChannels.push_back(std::static_pointer_cast<ChannelTest>(shared_from_this()));
        bus->mInterrupted.notify_all();

        while(!mOutBuffers.empty()) {
            if(mInterrupted.wait_until(lck, deadline) == std::cv_status::timeout) {
				goto end;
            }
        }
		
		count++;
    }

end:
	*pNumMsgs = count;
    return;
}

PeriodicMessagePtr ChannelTest::startPeriodicMsg(PASSTHRU_MSG *pMsg, unsigned long TimeInterval) {
    UNUSED(pMsg);
    UNUSED(TimeInterval);
    return nullptr;
}

void ChannelTest::stopPeriodicMsg(const PeriodicMessagePtr &periodicMessage) {
    UNUSED(periodicMessage);
}

MessageFilterPtr ChannelTest::startMsgFilter(unsigned long FilterType, PASSTHRU_MSG *pMaskMsg, PASSTHRU_MSG *pPatternMsg,
                                                             PASSTHRU_MSG *pFlowControlMsg) {
    UNUSED(FilterType);
    UNUSED(pMaskMsg);
    UNUSED(pPatternMsg);
    UNUSED(pFlowControlMsg);
    return std::make_shared<MessageFilterTest>(std::static_pointer_cast<ChannelTest>(shared_from_this()));
}

void ChannelTest::stopMsgFilter(const MessageFilterPtr &messageFilter) {
    UNUSED(messageFilter);
}

void ChannelTest::ioctl(unsigned long IoctlID, void *pInput, void *pOutput) {
    UNUSED(pInput);
    UNUSED(pOutput);
    if(IoctlID == CLEAR_RX_BUFFER) {
        mInBuffers.clear();
    } else if(IoctlID == CLEAR_TX_BUFFER) {
        mOutBuffers.clear();
    }
}

DeviceWeakPtr ChannelTest::getDevice() const {
    return DeviceWeakPtr();
}

static void pid2Data(uint32_t pid, uint8_t *data) {
    data[0] = 0x1F & (pid >> 24);
    data[1] = 0xFF & (pid >> 16);
    data[2] = 0xFF & (pid >> 8);
    data[3] = 0xFF & (pid >> 0);
}


#define J2534_DATA_OFFSET 4
int main(int argc, char *argv[]) {
    UNUSED(argc);
    UNUSED(argv);
    BusPtr bus = std::make_shared<Bus>();
    ChannelTestPtr channel1 = std::make_shared<ChannelTest>();
    ChannelTestPtr channel2 = std::make_shared<ChannelTest>();
    bus->addChannel(channel1);
    bus->addChannel(channel2);

    ChannelPtr c1 = std::make_shared<ChannelISO15765>(nullptr, channel1);
    ChannelPtr c2 = std::make_shared<ChannelISO15765>(nullptr, channel2);

    uint32_t pid1 = 0x1234;
    uint32_t pid2 = 0x4321;
    size_t size = 1023;

    PASSTHRU_MSG msg1;
	PASSTHRU_MSG msg2;
	
    msg1.DataSize = size + J2534_DATA_OFFSET;
	msg1.TxFlags = 0;
    pid2Data(pid2, msg1.Data);
    for(size_t i = 0; i < size; ++i) {
        msg1.Data[i + J2534_DATA_OFFSET] = (uint8_t)(i%256);
    }

    // Set BS and STMIN
    SCONFIG CfgItem[2];
    SCONFIG_LIST Input;

    CfgItem[0].Parameter = ISO15765_BS;
    CfgItem[0].Value = 0x20; /* BlockSize is 32 frames */
    CfgItem[1].Parameter = ISO15765_STMIN;
    CfgItem[1].Value = 0x01; /* SeparationTime is 1 millisecond */
    Input.NumOfParams = 2; /* Configuration list has 2 items */
    Input.ConfigPtr = CfgItem;

    c1->ioctl(SET_CONFIG, &Input, NULL);
    c2->ioctl(SET_CONFIG, &Input, NULL);


    PASSTHRU_MSG maskMsg1, maskMsg2;
    PASSTHRU_MSG patternMsg1, patternMsg2;
    PASSTHRU_MSG flowControlMsg1, flowControlMsg2;

    pid2Data(pid1, patternMsg1.Data);
    pid2Data(0xFFFFFFFF, maskMsg1.Data);
    pid2Data(pid2, flowControlMsg1.Data);
    c1->startMsgFilter(FLOW_CONTROL_FILTER, &maskMsg1, &patternMsg1, &flowControlMsg1);

    pid2Data(pid2, patternMsg2.Data);
    pid2Data(0xFFFFFFFF, maskMsg2.Data);
    pid2Data(pid1, flowControlMsg2.Data);
    c2->startMsgFilter(FLOW_CONTROL_FILTER, &maskMsg2, &patternMsg2, &flowControlMsg2);


    // Start the test
    unsigned long written = 0;
    unsigned long read = 0;
    printf("Start\n");
    fflush(stdout);
    std::thread t([&]() {
		written = 1;
        c1->writeMsgs(&msg1, &written, 5000);
    });

    std::thread t2([&]() {
		read = 1;
        c2->readMsgs(&msg2, &read, 5000);
    });

    t.join();
    t2.join();
	
    printf("Written %ld\n", written);
    printf("Read %ld\n", read);
	
	if(written != 1 || read != 1) {
        LOG_DEBUG("Wrong received/sent message");
        return -1;
	}

    // Check the test
    if(msg2.DataSize != (J2534_DATA_OFFSET + size)) {
        LOG_DEBUG("Wrong size");
        return -1;
    }

    //printMsg(msgs1[0]);
    //printMsg(msgs2[0]);

    if(memcmp(msg1.Data, msg2.Data, size + J2534_DATA_OFFSET) != 0) {
        LOG_DEBUG("Wrong content");
        return -2;
    }

    printf("Test OK!\n");

    return 0;
}