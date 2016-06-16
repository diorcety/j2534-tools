#include "configurable_channel.h"

#include "utils.h"

template<typename T>
class ConfigParams {
public:
    unsigned long id;
    std::function<unsigned long &(T &)> fct;
};

class DefaultConfig : public Configuration {
public:
    DefaultConfig();

    virtual ~DefaultConfig();

    virtual bool getValue(unsigned long config, unsigned long *value) const;

    virtual bool setValue(unsigned long config, unsigned long value);

protected:
    template<typename T>
    static std::function<unsigned long &()> getParam(T &obj, ConfigParams<T> *params, unsigned long id) {
        ConfigParams<T> *p = params;
        while (p != NULL && p->id != 0) {
            if (p->id == id) {
                return std::bind(p->fct, obj);
            }
            p++;
        }
        return NULL;
    }

    template<typename T>
    static std::function<unsigned long const &()> getParam(const T &obj, ConfigParams<T> *params, unsigned long id) {
        ConfigParams<T> *p = params;
        while (p != NULL && p->id != 0) {
            if (p->id == id) {
                return std::bind(p->fct, obj);
            }
            p++;
        }
        return NULL;
    }

private:
    unsigned long mDatarate;
    unsigned long mLoopback;

    static ConfigParams<DefaultConfig> parameters[];

};

class CANConfig : public DefaultConfig {
public:
    CANConfig();

    ~CANConfig();

    virtual bool getValue(unsigned long config, unsigned long *value) const;

    virtual bool setValue(unsigned long config, unsigned long value);

private:
    unsigned long mCan29BitId;
    unsigned long mbitSamplePoint;
    unsigned long mSyncJumpWidth;

    static ConfigParams<CANConfig> parameters[];
};

class ISO15765Config : public CANConfig {
public:
    ISO15765Config();

    virtual ~ISO15765Config();

    virtual bool getValue(unsigned long config, unsigned long *value) const;

    virtual bool setValue(unsigned long config, unsigned long value);

private:
    unsigned long mBlockSize;
    unsigned long mSeparationTime;
    unsigned long mISO15765AddrType;

    static ConfigParams<ISO15765Config> parameters[];
};

DefaultConfig::DefaultConfig() {
    mLoopback = 0;
}

DefaultConfig::~DefaultConfig() {

}

ConfigParams<DefaultConfig> DefaultConfig::parameters[] = {
        {DATA_RATE, [](DefaultConfig &c) -> unsigned long & { return c.mDatarate; }},
        {LOOPBACK,  [](DefaultConfig &c) -> unsigned long & { return c.mLoopback; }},
        {0, NULL}
};

bool DefaultConfig::getValue(unsigned long config, unsigned long *value) const {
    std::function<const unsigned long &()> fct = getParam(*this, parameters, config);
    if (fct) {
        *value = fct();
        return true;
    }
    return false;
}

bool DefaultConfig::setValue(unsigned long config, unsigned long value) {
    std::function<unsigned long &()> fct = getParam(*this, parameters, config);
    if (fct) {
        fct() = value;
        return true;
    }
    return false;
}


CANConfig::CANConfig() {
    mCan29BitId = 0;
    mbitSamplePoint = 80;
    mSyncJumpWidth = 15;
}

CANConfig::~CANConfig() {

}

ConfigParams<CANConfig> CANConfig::parameters[] = {
        {CAN_29BIT_ID,     [](CANConfig &c) -> unsigned long & { return c.mCan29BitId; }},
        {BIT_SAMPLE_POINT, [](CANConfig &c) -> unsigned long & { return c.mbitSamplePoint; }},
        {SYNC_JUMP_WIDTH,  [](CANConfig &c) -> unsigned long & { return c.mSyncJumpWidth; }},
        {0, NULL}
};

bool CANConfig::getValue(unsigned long config, unsigned long *value) const {
    if (DefaultConfig::getValue(config, value)) {
        return true;
    }
    std::function<const unsigned long &()> fct = getParam(*this, parameters, config);
    if (fct) {
        *value = fct();
        return true;
    }
    return false;
}

bool CANConfig::setValue(unsigned long config, unsigned long value) {
    if (DefaultConfig::setValue(config, value)) {
        return true;
    }
    std::function<unsigned long &()> fct = getParam(*this, parameters, config);
    if (fct) {
        fct() = value;
        return true;
    }
    return false;
}

ISO15765Config::ISO15765Config() {
    mBlockSize = 0;
    mSeparationTime = 0;
    mISO15765AddrType = 0;
}

ISO15765Config::~ISO15765Config() {

}

ConfigParams<ISO15765Config> ISO15765Config::parameters[] = {
        {ISO15765_BS,        [](ISO15765Config &c) -> unsigned long & { return c.mBlockSize; }},
        {ISO15765_STMIN,     [](ISO15765Config &c) -> unsigned long & { return c.mSeparationTime; }},
        {ISO15765_ADDR_TYPE, [](ISO15765Config &c) -> unsigned long & { return c.mISO15765AddrType; }},
        {0, NULL}
};

bool ISO15765Config::getValue(unsigned long config, unsigned long *value) const {
    if (CANConfig::getValue(config, value)) {
        return true;
    }
    std::function<const unsigned long &()> fct = getParam(*this, parameters, config);
    if (fct) {
        *value = fct();
        return true;
    }
    return false;
}

bool ISO15765Config::setValue(unsigned long config, unsigned long value) {
    if (CANConfig::setValue(config, value)) {
        return true;
    }
    std::function<unsigned long &()> fct = getParam(*this, parameters, config);
    if (fct) {
        fct() = value;
        return true;
    }
    return false;
}

ConfigurableChannel::ConfigurableChannel(unsigned long ProtocolID) {
    mConfiguration = createConfig(ProtocolID);
}

Configuration &ConfigurableChannel::getConfiguration() {
	return *mConfiguration;
}

std::unique_ptr<Configuration> ConfigurableChannel::createConfig(unsigned long ProtocolID) const {
    if ((ProtocolID & 0x7FFF) == ISO15765) {
        return std::make_unique<ISO15765Config>();
    }
    if ((ProtocolID & 0x7FFF) == CAN) {
        return std::make_unique<CANConfig>();
    }
    return std::make_unique<DefaultConfig>();
}

ConfigurableChannel::~ConfigurableChannel() {

}

bool ConfigurableChannel::getConfigs(SCONFIG_LIST *list) const {
	bool ret = false;
    if (list == NULL) {
        return ret;
    }
    SCONFIG *config = list->ConfigPtr;
    for (unsigned long i = 0; i < list->NumOfParams; ++i) {
        ret |= getConfig(config++);
    }
	return ret;
}

bool ConfigurableChannel::getConfig(SCONFIG *config) const {
    return mConfiguration->getValue(config->Parameter, &config->Value);
}

bool ConfigurableChannel::setConfigs(SCONFIG_LIST *list) {
	bool ret = false;
    if (list == NULL) {
        return ret;
    }
    SCONFIG *config = list->ConfigPtr;
    for (unsigned long i = 0; i < list->NumOfParams; ++i) {
        ret |= setConfig(config++);
    }
	return ret;
}

bool ConfigurableChannel::setConfig(SCONFIG *config) {
    return mConfiguration->setValue(config->Parameter, config->Value);
}

bool ConfigurableChannel::handle_ioctl(unsigned long IoctlID, void *pInput, void *pOutput) {
	UNUSED(pOutput);
    switch (IoctlID) {
        case GET_CONFIG:
            return getConfigs(reinterpret_cast<SCONFIG_LIST *>(pInput));
        case SET_CONFIG:
            return setConfigs(reinterpret_cast<SCONFIG_LIST *>(pInput));
        case CLEAR_TX_BUFFER:
            return clearTxBuffers();
        case CLEAR_RX_BUFFER:
            return clearRxBuffers();
        case CLEAR_PERIODIC_MSGS:
            return clearPeriodicMessages();
        case CLEAR_MSG_FILTERS:
            return clearMessageFilters();
    }
	return false;
}

void ConfigurableChannel::ioctl(unsigned long IoctlID, void *pInput, void *pOutput) {
    handle_ioctl(IoctlID, pInput, pOutput);
}