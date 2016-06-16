#ifndef FTD2XX_H
#define FTD2XX_H

#include <windows.h>

typedef ULONG FT_HANDLE;

//
// Device status
//
enum FT_STATUS {
    FT_OK,
    FT_INVALID_HANDLE,
    FT_DEVICE_NOT_FOUND,
    FT_DEVICE_NOT_OPENED,
    FT_IO_ERROR,
    FT_INSUFFICIENT_RESOURCES,
    FT_INVALID_PARAMETER
};


#define FT_SUCCESS(status) ((status) == FT_OK)

//
// FT_OpenEx Flags
//

#define FT_OPEN_BY_SERIAL_NUMBER    1
#define FT_OPEN_BY_DESCRIPTION      2
#define FT_OPEN_BY_LOCATION            4

//
// FT_ListDevices Flags (used in conjunction with FT_OpenEx Flags
//

#define FT_LIST_NUMBER_ONLY            0x80000000
#define FT_LIST_BY_INDEX            0x40000000
#define FT_LIST_ALL                    0x20000000

#define FT_LIST_MASK (FT_LIST_NUMBER_ONLY|FT_LIST_BY_INDEX|FT_LIST_ALL)

//
// Baud Rates
//

#define FT_BAUD_300            300
#define FT_BAUD_600            600
#define FT_BAUD_1200        1200
#define FT_BAUD_2400        2400
#define FT_BAUD_4800        4800
#define FT_BAUD_9600        9600
#define FT_BAUD_14400        14400
#define FT_BAUD_19200        19200
#define FT_BAUD_38400        38400
#define FT_BAUD_57600        57600
#define FT_BAUD_115200        115200
#define FT_BAUD_230400        230400
#define FT_BAUD_460800        460800
#define FT_BAUD_921600        921600

//
// Word Lengths
//

#define FT_BITS_8            (UCHAR) 8
#define FT_BITS_7            (UCHAR) 7
#define FT_BITS_6            (UCHAR) 6
#define FT_BITS_5            (UCHAR) 5

//
// Stop Bits
//

#define FT_STOP_BITS_1        (UCHAR) 0
#define FT_STOP_BITS_1_5    (UCHAR) 1
#define FT_STOP_BITS_2        (UCHAR) 2

//
// Parity
//

#define FT_PARITY_NONE        (UCHAR) 0
#define FT_PARITY_ODD        (UCHAR) 1
#define FT_PARITY_EVEN        (UCHAR) 2
#define FT_PARITY_MARK        (UCHAR) 3
#define FT_PARITY_SPACE        (UCHAR) 4

//
// Flow Control
//

#define FT_FLOW_NONE        0x0000
#define FT_FLOW_RTS_CTS     0x0100
#define FT_FLOW_DTR_DSR     0x0200
#define FT_FLOW_XON_XOFF    0x0400

//
// Purge rx and tx buffers
//
#define FT_PURGE_RX         1
#define FT_PURGE_TX         2

//
// Events
//

typedef void (*PFT_EVENT_HANDLER)(DWORD, DWORD);

#define FT_EVENT_RXCHAR            1
#define FT_EVENT_MODEM_STATUS   2

//
// Timeouts
//

#define FT_DEFAULT_RX_TIMEOUT   300
#define FT_DEFAULT_TX_TIMEOUT   300

#ifdef __cplusplus
extern "C" {
#endif

typedef FT_STATUS (WINAPI *pFT_Open )(int deviceNumber, FT_HANDLE *pHandle);
typedef FT_STATUS (WINAPI *pFT_OpenEx )(PVOID pArg1, DWORD
Flags, FT_HANDLE *pHandle);
typedef FT_STATUS (WINAPI *pFT_ListDevices )(PVOID pArg1, PVOID
pArg2, DWORD Flags);
typedef FT_STATUS (WINAPI *pFT_Close )(FT_HANDLE ftHandle);
typedef FT_STATUS (WINAPI *pFT_Read )(FT_HANDLE ftHandle, LPVOID
lpBuffer, DWORD dwBytesToRead, LPDWORD
                                      lpBytesReturned);
typedef FT_STATUS (WINAPI *pFT_Write )(FT_HANDLE ftHandle, LPVOID
lpBuffer, DWORD dwBytesToWrite, LPDWORD
                                       lpBytesWritten);
typedef FT_STATUS (WINAPI *pFT_IoCtl )(FT_HANDLE ftHandle, DWORD
dwIoControlCode, LPVOID lpInBuf, DWORD
                                       nInBufSize, LPVOID lpOutBuf, DWORD
                                       nOutBufSize, LPDWORD lpBytesReturned, LPOVERLAPPED
                                       lpOverlapped);
typedef FT_STATUS (WINAPI *pFT_SetBaudRate )(FT_HANDLE ftHandle, ULONG
BaudRate);
typedef FT_STATUS (WINAPI *pFT_SetDivisor )(FT_HANDLE ftHandle, USHORT
Divisor);
typedef FT_STATUS (WINAPI *pFT_SetDataCharacteristics )(FT_HANDLE ftHandle, UCHAR
WordLength, UCHAR StopBits, UCHAR
                                                        Parity);
typedef FT_STATUS (WINAPI *pFT_SetFlowControl )(FT_HANDLE ftHandle, USHORT
FlowControl, UCHAR XonChar, UCHAR
                                                XoffChar);
typedef FT_STATUS (WINAPI *pFT_ResetDevice )(FT_HANDLE ftHandle);
typedef FT_STATUS (WINAPI *pFT_SetDtr )(FT_HANDLE ftHandle);
typedef FT_STATUS (WINAPI *pFT_ClrDtr )(FT_HANDLE ftHandle);
typedef FT_STATUS (WINAPI *pFT_SetRts )(FT_HANDLE ftHandle);
typedef FT_STATUS (WINAPI *pFT_ClrRts )(FT_HANDLE ftHandle);
typedef FT_STATUS (WINAPI *pFT_GetModemStatus )(FT_HANDLE ftHandle, ULONG *pModemStatus);
typedef FT_STATUS (WINAPI *pFT_SetChars )(FT_HANDLE ftHandle, UCHAR
EventChar, UCHAR EventCharEnabled, UCHAR
                                          ErrorChar, UCHAR ErrorCharEnabled);
typedef FT_STATUS (WINAPI *pFT_Purge )(FT_HANDLE ftHandle, ULONG
Mask);
typedef FT_STATUS (WINAPI *pFT_SetTimeouts )(FT_HANDLE ftHandle, ULONG
ReadTimeout, ULONG WriteTimeout);
typedef FT_STATUS (WINAPI *pFT_GetQueueStatus )(FT_HANDLE ftHandle, DWORD *dwRxBytes);
typedef FT_STATUS (WINAPI *pFT_SetEventNotification )(FT_HANDLE ftHandle, DWORD
Mask, PVOID Param);
typedef FT_STATUS (WINAPI *pFT_GetEventStatus )(FT_HANDLE ftHandle, DWORD *dwEventDWord);
typedef FT_STATUS (WINAPI *pFT_GetStatus )(FT_HANDLE ftHandle, DWORD *dwRxBytes, DWORD *dwTxBytes, DWORD *dwEventDWord);
typedef FT_STATUS (WINAPI *pFT_SetBreakOn )(FT_HANDLE ftHandle);
typedef FT_STATUS (WINAPI *pFT_SetBreakOff )(FT_HANDLE ftHandle);
typedef FT_STATUS (WINAPI *pFT_SetWaitMask )(FT_HANDLE ftHandle, DWORD
Mask);
typedef FT_STATUS (WINAPI *pFT_WaitOnMask )(FT_HANDLE ftHandle, DWORD *Mask);

#ifdef __cplusplus
}
#endif


#endif  /* FTD2XX_H */
