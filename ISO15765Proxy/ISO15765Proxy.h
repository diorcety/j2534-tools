#pragma once

#ifndef __ISO15765PROXY_H
#define __ISO15765PROXY_H

#include "j2534_v0404.h"

#ifdef _WIN32
#include <windows.h>
#endif //_WIN32

#ifdef __cplusplus
extern "C" {
#endif

long J2534_API PassThruOpen(void *pName, unsigned long *pDeviceID);
long J2534_API PassThruClose(unsigned long DeviceID);
long J2534_API PassThruConnect(unsigned long DeviceID, unsigned long ProtocolID, unsigned long Flags,
                               unsigned long Baudrate, unsigned long *pChannelID);
long J2534_API PassThruDisconnect(unsigned long ChannelID);
long J2534_API PassThruReadMsgs(unsigned long ChannelID, PASSTHRU_MSG *pMsg, unsigned long *pNumMsgs,
                                unsigned long Timeout);
long J2534_API PassThruWriteMsgs(unsigned long ChannelID, PASSTHRU_MSG *pMsg, unsigned long *pNumMsgs,
                                 unsigned long Timeout);
long J2534_API PassThruStartPeriodicMsg(unsigned long ChannelID, PASSTHRU_MSG *pMsg, unsigned long *pMsgID,
                                        unsigned long TimeInterval);
long J2534_API PassThruStopPeriodicMsg(unsigned long ChannelID, unsigned long MsgID);
long J2534_API PassThruStartMsgFilter(unsigned long ChannelID, unsigned long FilterType, PASSTHRU_MSG *pMaskMsg,
                                      PASSTHRU_MSG *pPatternMsg, PASSTHRU_MSG *pFlowControlMsg,
                                      unsigned long *pFilterID);
long J2534_API PassThruStopMsgFilter(unsigned long ChannelID, unsigned long FilterID);
long J2534_API PassThruSetProgrammingVoltage(unsigned long DeviceID, unsigned long PinNumber, unsigned long Voltage);
long J2534_API PassThruReadVersion(unsigned long DeviceID, char *pFirmwareVersion, char *pDllVersion,
                                   char *pApiVersion);
long J2534_API PassThruGetLastError(char *pErrorDescription);
long J2534_API PassThruIoctl(unsigned long ChannelID, unsigned long IoctlID, void *pInput, void *pOutput);

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

#endif // __ISO15765PROXY_H