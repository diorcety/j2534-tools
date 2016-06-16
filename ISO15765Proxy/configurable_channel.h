#pragma once

#ifndef _CONFIGURABLE_CHANNEL_H
#define _CONFIGURABLE_CHANNEL_H

#include "internal.h"
#include <memory>

class ConfigurableChannel : public Channel {

public:
    ConfigurableChannel(unsigned long ProtocolID);
	virtual ~ConfigurableChannel();
	
    virtual void ioctl(unsigned long IoctlID, void *pInput, void *pOutput);
	
    virtual Configuration &getConfiguration();
   
protected:
    virtual std::unique_ptr<Configuration> createConfig(unsigned long ProtocolID) const;
	
    virtual bool getConfig(SCONFIG *config) const;

    virtual bool setConfig(SCONFIG *config);
	
    virtual bool clearTxBuffers() = 0;

    virtual bool clearRxBuffers() = 0;

    virtual bool clearPeriodicMessages() = 0;

    virtual bool clearMessageFilters() = 0;
	
	virtual bool handle_ioctl(unsigned long IoctlID, void *pInput, void *pOutput);
	
private:
    virtual bool getConfigs(SCONFIG_LIST *list) const;
    virtual bool setConfigs(SCONFIG_LIST *list);
	
	std::unique_ptr<Configuration> mConfiguration;
};

#endif //_CONFIGURABLE_CHANNEL_H