#pragma once

#ifndef __STUB_H
#define __STUB_H

#include "j2534_v0404.h"

#ifdef _WIN32
#include <windows.h>
#endif //_WIN32

#ifdef _WIN32
#define STUB_API J2534_API
#endif //_WIN32

#ifdef __linux__
#define STUB_API J2534_API __attribute__ ((visibility("default")))
#endif //__linux__

#ifdef __cplusplus
extern "C" {
#endif

long STUB_API PassThruOpen(void *pName, unsigned long *pDeviceID);
long STUB_API PassThruClose(unsigned long DeviceID);
long STUB_API PassThruConnect(unsigned long DeviceID, unsigned long ProtocolID, unsigned long Flags,
                               unsigned long Baudrate, unsigned long *pChannelID);
long STUB_API PassThruDisconnect(unsigned long ChannelID);
long STUB_API PassThruReadMsgs(unsigned long ChannelID, PASSTHRU_MSG *pMsg, unsigned long *pNumMsgs,
                                unsigned long Timeout);
long STUB_API PassThruWriteMsgs(unsigned long ChannelID, PASSTHRU_MSG *pMsg, unsigned long *pNumMsgs,
                                 unsigned long Timeout);
long STUB_API PassThruStartPeriodicMsg(unsigned long ChannelID, PASSTHRU_MSG *pMsg, unsigned long *pMsgID,
                                        unsigned long TimeInterval);
long STUB_API PassThruStopPeriodicMsg(unsigned long ChannelID, unsigned long MsgID);
long STUB_API PassThruStartMsgFilter(unsigned long ChannelID, unsigned long FilterType, PASSTHRU_MSG *pMaskMsg,
                                      PASSTHRU_MSG *pPatternMsg, PASSTHRU_MSG *pFlowControlMsg,
                                      unsigned long *pFilterID);
long STUB_API PassThruStopMsgFilter(unsigned long ChannelID, unsigned long FilterID);
long STUB_API PassThruSetProgrammingVoltage(unsigned long DeviceID, unsigned long PinNumber, unsigned long Voltage);
long STUB_API PassThruReadVersion(unsigned long DeviceID, char *pFirmwareVersion, char *pDllVersion,
                                   char *pApiVersion);
long STUB_API PassThruGetLastError(char *pErrorDescription);
long STUB_API PassThruIoctl(unsigned long ChannelID, unsigned long IoctlID, void *pInput, void *pOutput);

#ifdef __cplusplus
}
#endif

typedef struct {
    PTOPEN passThruOpen;
    PTCLOSE passThruClose;
    PTCONNECT passThruConnect;
    PTDISCONNECT passThruDisconnect;
    PTREADMSGS passThruReadMsgs;
    PTWRITEMSGS passThruWriteMsgs;
    PTSTARTPERIODICMSG passThruStartPeriodicMsg;
    PTSTOPPERIODICMSG passThruStopPeriodicMsg;
    PTSTARTMSGFILTER passThruStartMsgFilter;
    PTSTOPMSGFILTER passThruStopMsgFilter;
    PTSETPROGRAMMINGVOLTAGE passThruSetProgrammingVoltage;
    PTREADVERSION passThruReadVersion;
    PTGETLASTERROR passThruGetLastError;
    PTIOCTL passThruIoctl;
} j2534_fcts;

#endif // __STUB_H