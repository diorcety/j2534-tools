#pragma once

#ifndef __FTDIPROXY_H
#define __FTDIPROXY_H

#include "FTD2XX.H"
#include "../gateway/gateway.h"

#ifdef __cplusplus
extern "C" {
#endif
FT_STATUS WINAPI FT_Open(int deviceNumber, FT_HANDLE *pHandle);
FT_STATUS WINAPI FT_OpenEx(PVOID pArg1, DWORD Flags, FT_HANDLE *pHandle);
FT_STATUS WINAPI FT_ListDevices(PVOID pArg1, PVOID pArg2, DWORD Flags);
FT_STATUS WINAPI FT_Close(FT_HANDLE ftHandle);
FT_STATUS WINAPI FT_Read(FT_HANDLE ftHandle, LPVOID lpBuffer, DWORD nBufferSize, LPDWORD lpBytesReturned);
FT_STATUS WINAPI FT_Write(FT_HANDLE ftHandle, LPVOID lpBuffer, DWORD nBufferSize, LPDWORD lpBytesWritten);
FT_STATUS WINAPI FT_IoCtl(FT_HANDLE ftHandle, DWORD dwIoControlCode, LPVOID lpInBuf, DWORD nInBufSize, LPVOID lpOutBuf,
                          DWORD nOutBufSize, LPDWORD lpBytesReturned, LPOVERLAPPED lpOverlapped);
FT_STATUS WINAPI FT_SetBaudRate(FT_HANDLE ftHandle, ULONG BaudRate);
FT_STATUS WINAPI FT_SetDivisor(FT_HANDLE ftHandle, USHORT Divisor);
FT_STATUS WINAPI FT_SetDataCharacteristics(FT_HANDLE ftHandle, UCHAR WordLength, UCHAR StopBits, UCHAR Parity);
FT_STATUS WINAPI FT_SetFlowControl(FT_HANDLE ftHandle, USHORT FlowControl, UCHAR XonChar, UCHAR XoffChar);
FT_STATUS WINAPI FT_ResetDevice(FT_HANDLE ftHandle);
FT_STATUS WINAPI FT_SetDtr(FT_HANDLE ftHandle);
FT_STATUS WINAPI FT_ClrDtr(FT_HANDLE ftHandle);
FT_STATUS WINAPI FT_SetRts(FT_HANDLE ftHandle);
FT_STATUS WINAPI FT_ClrRts(FT_HANDLE ftHandle);
FT_STATUS WINAPI FT_GetModemStatus(FT_HANDLE ftHandle, ULONG *pModemStatus);
FT_STATUS WINAPI FT_SetChars(FT_HANDLE ftHandle, UCHAR EventChar, UCHAR EventCharEnabled, UCHAR ErrorChar,
                             UCHAR ErrorCharEnabled);
FT_STATUS WINAPI FT_Purge(FT_HANDLE ftHandle, ULONG Mask);
FT_STATUS WINAPI FT_SetTimeouts(FT_HANDLE ftHandle, ULONG ReadTimeout, ULONG WriteTimeout);
FT_STATUS WINAPI FT_GetQueueStatus(FT_HANDLE ftHandle, DWORD *dwRxBytes);
FT_STATUS WINAPI FT_SetEventNotification(FT_HANDLE ftHandle, DWORD Mask, PVOID Param);
FT_STATUS WINAPI FT_GetEventStatus(FT_HANDLE ftHandle, DWORD *dwEventDWord);
FT_STATUS WINAPI FT_GetStatus(FT_HANDLE ftHandle, DWORD *dwRxBytes, DWORD *dwTxBytes, DWORD *dwEventDWord);
FT_STATUS WINAPI FT_SetBreakOn(FT_HANDLE ftHandle);
FT_STATUS WINAPI FT_SetBreakOff(FT_HANDLE ftHandle);
FT_STATUS WINAPI FT_SetWaitMask(FT_HANDLE ftHandle, DWORD Mask);
FT_STATUS WINAPI FT_WaitOnMask(FT_HANDLE ftHandle, DWORD *Mask);

#ifdef __cplusplus
}
#endif

typedef struct {
    pFT_Open open;
    pFT_OpenEx openEx;
    pFT_ListDevices listDevices;
    pFT_Close close;
    pFT_Write write;
    pFT_Read read;
    pFT_IoCtl ioCtl;
    pFT_SetBaudRate setBaudRate;
    pFT_SetDivisor setDivisor;
    pFT_SetDataCharacteristics setDataCharacteristics;
    pFT_SetFlowControl setFlowControl;
    pFT_ResetDevice resetDevice;
    pFT_SetDtr setDtr;
    pFT_ClrDtr clrDtr;
    pFT_SetRts setRts;
    pFT_ClrRts clrRts;
    pFT_GetModemStatus getModemStatus;
    pFT_SetChars setChars;
    pFT_Purge purge;
    pFT_SetTimeouts setTimeouts;
    pFT_GetQueueStatus getQueueStatus;
    pFT_SetEventNotification setEventNotification;
    pFT_GetEventStatus getEventStatus;
    pFT_GetStatus getStatus;
    pFT_SetBreakOn setBreakOn;
    pFT_SetBreakOff setBreakOff;
    pFT_SetWaitMask setWaitMask;
    pFT_WaitOnMask waitOnMask;
} ftdi_fcts;

extern HINSTANCE proxy_handle;
extern ftdi_fcts *proxy;

extern HINSTANCE gateway_handle;
extern pGetDeviceToOpen GetDeviceToOpen;

#define ENABLE_ALTERNATE

#endif // __FTDIPROXY_H