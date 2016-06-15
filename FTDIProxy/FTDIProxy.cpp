#include "stdafx.h"
#include "FTDIProxy.h"
#include "log.h"
#include "utils.h"

FT_STATUS WINAPI FT_Open(int deviceNumber, FT_HANDLE *pHandle) {
    LOG(INIT, "FT_Open");
    return proxy->open(deviceNumber, pHandle);
}

FT_STATUS WINAPI FT_OpenEx(PVOID pArg1, DWORD Flags, FT_HANDLE *pHandle) {
    LOG(INIT, "FT_OpenEx");
#ifdef ENABLE_ALTERNATE
    if (Flags & FT_OPEN_BY_DESCRIPTION) {
        LOG(INIT, "FT_OpenEx FT_OPEN_BY_DESCRIPTION");
        if (strcmp("M-VCI", (char *) pArg1) == 0) {
            LOG(INIT, "FT_OpenEx VCI");
#if 0
            long LocationID[16]; // more than enough room!
            long index = GetDeviceToOpen();
            FT_STATUS ftStatus = proxy->listDevices((PVOID) index, LocationID, FT_LIST_BY_INDEX | FT_OPEN_BY_LOCATION);
            LOG(INIT, "FT_OpenEx %d", LocationID[0]);
            if (ftStatus != FT_OK) {
                LOG(INIT, "FT_OpenEx err %d", ftStatus);
                return ftStatus;
            }
            LOG(INIT, "FT_OpenEx err OPEN", ftStatus);
            return proxy->openEx((PVOID) LocationID[0], FT_OPEN_BY_LOCATION, pHandle);
#else
            const char *sn = "A6UBWPN7";
            if(GetDeviceToOpen() != 0) {
                sn = "A6WXMWFF";
            }
            LOG(INIT, "FT_OpenEx %s", sn);
            return proxy->openEx((PVOID) sn, FT_OPEN_BY_SERIAL_NUMBER, pHandle);
#endif
        }
    }
#endif // ENABLE_ALTERNATE
    return proxy->openEx(pArg1, Flags, pHandle);
}

FT_STATUS WINAPI FT_ListDevices(PVOID pArg1, PVOID pArg2, DWORD Flags) {
    LOG(INIT, "FT_ListDevices");
    return proxy->listDevices(pArg1, pArg2, Flags);
}

FT_STATUS WINAPI FT_Close(FT_HANDLE ftHandle) {
    LOG(INIT, "FT_Close");
    return proxy->close(ftHandle);
}

FT_STATUS WINAPI FT_Read(FT_HANDLE ftHandle, LPVOID lpBuffer, DWORD nBufferSize, LPDWORD lpBytesReturned) {
    LOG(INIT, "FT_Read");
    return proxy->read(ftHandle, lpBuffer, nBufferSize, lpBytesReturned);
}

FT_STATUS WINAPI FT_Write(FT_HANDLE ftHandle, LPVOID lpBuffer, DWORD nBufferSize, LPDWORD lpBytesWritten) {
    LOG(INIT, "FT_Write");
    return proxy->write(ftHandle, lpBuffer, nBufferSize, lpBytesWritten);
}

FT_STATUS WINAPI FT_IoCtl(FT_HANDLE ftHandle, DWORD dwIoControlCode, LPVOID lpInBuf, DWORD nInBufSize, LPVOID lpOutBuf,
                          DWORD nOutBufSize, LPDWORD lpBytesReturned, LPOVERLAPPED lpOverlapped) {
    LOG(INIT, "FT_IoCtl");
    return proxy->ioCtl(ftHandle, dwIoControlCode, lpInBuf, nInBufSize, lpOutBuf, nOutBufSize, lpBytesReturned,
                        lpOverlapped
    );
}

FT_STATUS WINAPI FT_SetBaudRate(FT_HANDLE ftHandle, ULONG BaudRate) {
    LOG(INIT, "FT_SetBaudRate");
    return proxy->setBaudRate(ftHandle, BaudRate);
}

FT_STATUS WINAPI FT_SetDivisor(FT_HANDLE ftHandle, USHORT Divisor) {
    LOG(INIT, "FT_SetDivisor");
    return proxy->setDivisor(ftHandle, Divisor);
}

FT_STATUS WINAPI FT_SetDataCharacteristics(FT_HANDLE ftHandle, UCHAR WordLength, UCHAR StopBits, UCHAR Parity) {
    LOG(INIT, "FT_SetDataCharacteristics");
    return proxy->setDataCharacteristics(ftHandle, WordLength, StopBits, Parity);
}

FT_STATUS WINAPI FT_SetFlowControl(FT_HANDLE ftHandle, USHORT FlowControl, UCHAR XonChar, UCHAR XoffChar) {
    LOG(INIT, "FT_SetFlowControl");
    return proxy->setFlowControl(ftHandle, FlowControl, XonChar, XoffChar);
}

FT_STATUS WINAPI FT_ResetDevice(FT_HANDLE ftHandle) {
    LOG(INIT, "FT_ResetDevice");
    return proxy->resetDevice(ftHandle);
}

FT_STATUS WINAPI FT_SetDtr(FT_HANDLE ftHandle) {
    LOG(INIT, "FT_SetDtr");
    return proxy->setDtr(ftHandle);
}

FT_STATUS WINAPI FT_ClrDtr(FT_HANDLE ftHandle) {
    LOG(INIT, "FT_ClrDtr");
    return proxy->clrDtr(ftHandle);
}

FT_STATUS WINAPI FT_SetRts(FT_HANDLE ftHandle) {
    LOG(INIT, "FT_SetRts");
    return proxy->setRts(ftHandle);
}

FT_STATUS WINAPI FT_ClrRts(FT_HANDLE ftHandle) {
    LOG(INIT, "FT_ClrRts");
    return proxy->clrRts(ftHandle);
}

FT_STATUS WINAPI FT_GetModemStatus(FT_HANDLE ftHandle, ULONG *pModemStatus) {
    LOG(INIT, "FT_GetModemStatus");
    return proxy->getModemStatus(ftHandle, pModemStatus);
}

FT_STATUS WINAPI FT_SetChars(FT_HANDLE ftHandle, UCHAR EventChar, UCHAR EventCharEnabled, UCHAR ErrorChar,
                             UCHAR ErrorCharEnabled) {
    LOG(INIT, "FT_SetChars");
    return proxy->setChars(ftHandle, EventChar, EventCharEnabled, ErrorChar, ErrorCharEnabled);
}

FT_STATUS WINAPI FT_Purge(FT_HANDLE ftHandle, ULONG Mask) {
    LOG(INIT, "FT_Purge");
    return proxy->purge(ftHandle, Mask);
}

FT_STATUS WINAPI FT_SetTimeouts(FT_HANDLE ftHandle, ULONG ReadTimeout, ULONG WriteTimeout) {
    LOG(INIT, "FT_SetTimeouts");
    return proxy->setTimeouts(ftHandle, ReadTimeout, WriteTimeout);
}

FT_STATUS WINAPI FT_GetQueueStatus(FT_HANDLE ftHandle, DWORD *dwRxBytes) {
    LOG(INIT, "FT_GetQueueStatus");
    return proxy->getQueueStatus(ftHandle, dwRxBytes);
}

FT_STATUS WINAPI FT_SetEventNotification(FT_HANDLE ftHandle, DWORD Mask, PVOID Param) {
    LOG(INIT, "FT_SetEventNotification");
    return proxy->setEventNotification(ftHandle, Mask, Param);
}

FT_STATUS WINAPI FT_GetEventStatus(FT_HANDLE ftHandle, DWORD *dwEventDWord) {
    LOG(INIT, "FT_GetEventStatus");
    return proxy->getEventStatus(ftHandle, dwEventDWord);
}

FT_STATUS WINAPI FT_GetStatus(FT_HANDLE ftHandle, DWORD *dwRxBytes, DWORD *dwTxBytes, DWORD *dwEventDWord) {
    LOG(INIT, "FT_GetStatus");
    return proxy->getStatus(ftHandle, dwRxBytes, dwTxBytes, dwEventDWord);
}

FT_STATUS WINAPI FT_SetBreakOn(FT_HANDLE ftHandle) {
    LOG(INIT, "FT_SetBreakOn");
    return proxy->setBreakOn(ftHandle);
}

FT_STATUS WINAPI FT_SetBreakOff(FT_HANDLE ftHandle) {
    LOG(INIT, "FT_SetBreakOff");
    return proxy->setBreakOff(ftHandle);
}

FT_STATUS WINAPI FT_SetWaitMask(FT_HANDLE ftHandle, DWORD Mask) {
    LOG(INIT, "FT_SetWaitMask");
    return proxy->setWaitMask(ftHandle, Mask);
}

FT_STATUS WINAPI FT_WaitOnMask(FT_HANDLE ftHandle, DWORD *Mask) {
    LOG(INIT, "FT_WaitOnMask");
    return proxy->waitOnMask(ftHandle, Mask);
}

