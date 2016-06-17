#include "stub.h"
#include "log.h"
#include "utils.h"

///////////////////////////////////// PassThruFunctions /////////////////////////////////////////////////

long STUB_API PassThruOpen(void *pName, unsigned long *pDeviceID) {
    UNUSED(pName);
    UNUSED(pDeviceID);
    LOG(INIT, "PassThruOpen");

    long ret = STATUS_NOERROR;

    return ret;
}

long STUB_API PassThruClose(unsigned long DeviceID) {
    UNUSED(DeviceID);
    LOG(INIT, "PassThruClose");

    long ret = STATUS_NOERROR;

    return ret;
}

long STUB_API PassThruConnect(unsigned long DeviceID, unsigned long ProtocolID, unsigned long Flags,
                               unsigned long Baudrate, unsigned long *pChannelID) {
    UNUSED(DeviceID);
    UNUSED(ProtocolID);
    UNUSED(Flags);
    UNUSED(Baudrate);
    UNUSED(pChannelID);
    LOG(INIT, "PassThruConnect");

    long ret = STATUS_NOERROR;

    return ret;
}

long STUB_API PassThruDisconnect(unsigned long ChannelID) {
    UNUSED(ChannelID);
    LOG(INIT, "PassThruDisconnect");

    long ret = STATUS_NOERROR;

    return ret;
}


long STUB_API PassThruReadMsgs(unsigned long ChannelID, PASSTHRU_MSG *pMsg, unsigned long *pNumMsgs,
                                unsigned long Timeout) {
    UNUSED(ChannelID);
    UNUSED(pMsg);
    UNUSED(pNumMsgs);
    UNUSED(Timeout);
    LOG(INIT, "PassThruReadMsgs");

    long ret = STATUS_NOERROR;

    return ret;
}


long STUB_API PassThruWriteMsgs(unsigned long ChannelID, PASSTHRU_MSG *pMsg, unsigned long *pNumMsgs,
                                 unsigned long Timeout) {
    UNUSED(ChannelID);
    UNUSED(pMsg);
    UNUSED(pNumMsgs);
    UNUSED(Timeout);
    LOG(INIT, "PassThruWriteMsgs");

    long ret = STATUS_NOERROR;

    return ret;
}


long STUB_API PassThruStartPeriodicMsg(unsigned long ChannelID, PASSTHRU_MSG *pMsg, unsigned long *pMsgID,
                                        unsigned long TimeInterval) {
    UNUSED(ChannelID);
    UNUSED(pMsg);
    UNUSED(pMsgID);
    UNUSED(TimeInterval);
    LOG(INIT, "PassThruStartPeriodicMsg");

    long ret = STATUS_NOERROR;

    return ret;
}


long STUB_API PassThruStopPeriodicMsg(unsigned long ChannelID, unsigned long MsgID) {
    UNUSED(ChannelID);
    UNUSED(MsgID);
    LOG(INIT, "PassThruStopPeriodicMsg");

    long ret = STATUS_NOERROR;

    return ret;
}


long STUB_API PassThruStartMsgFilter(unsigned long ChannelID, unsigned long FilterType, PASSTHRU_MSG *pMaskMsg,
                                      PASSTHRU_MSG *pPatternMsg, PASSTHRU_MSG *pFlowControlMsg,
                                      unsigned long *pFilterID) {
    UNUSED(ChannelID);
    UNUSED(FilterType);
    UNUSED(pMaskMsg);
    UNUSED(pPatternMsg);
    UNUSED(pFlowControlMsg);
    UNUSED(pFilterID);
    LOG(INIT, "PassThruStartMsgFilter");

    long ret = STATUS_NOERROR;

    return ret;
}


long STUB_API PassThruStopMsgFilter(unsigned long ChannelID, unsigned long FilterID) {
    UNUSED(ChannelID);
    UNUSED(FilterID);
    LOG(INIT, "PassThruStopMsgFilter");

    long ret = STATUS_NOERROR;

    return ret;
}


long STUB_API PassThruSetProgrammingVoltage(unsigned long DeviceID, unsigned long PinNumber, unsigned long Voltage) {
    UNUSED(DeviceID);
    UNUSED(PinNumber);
    UNUSED(Voltage);
    LOG(INIT, "PassThruSetProgrammingVoltage");

    long ret = STATUS_NOERROR;

    return ret;
}


long STUB_API PassThruReadVersion(unsigned long DeviceID, char *pFirmwareVersion, char *pDllVersion,
                                   char *pApiVersion) {
    UNUSED(DeviceID);
    UNUSED(pFirmwareVersion);
    UNUSED(pDllVersion);
    UNUSED(pApiVersion);
    LOG(INIT, "PassThruReadVersion");

    long ret = STATUS_NOERROR;

    return ret;
}


long STUB_API PassThruGetLastError(char *pErrorDescription) {
    UNUSED(pErrorDescription);
    LOG(INIT, "PassThruGetLastError");

    long ret = STATUS_NOERROR;

    return ret;
}


long STUB_API PassThruIoctl(unsigned long ChannelID, unsigned long IoctlID, void *pInput, void *pOutput) {
    UNUSED(ChannelID);
    UNUSED(IoctlID);
    UNUSED(pInput);
    UNUSED(pOutput);
    LOG(INIT, "PassThruIoctl");

    long ret = STATUS_NOERROR;

    return ret;
}