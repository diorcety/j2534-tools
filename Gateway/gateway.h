#pragma once
#ifndef _GATEWAY_H
#define _GATEWAY_H

#ifdef _WIN32
#define GATEWAY_API __stdcall
#endif //_WIN32
#ifdef __linux__
#define GATEWAY_API
#endif //__linux__

typedef void (GATEWAY_API *pSetDeviceToOpen)(long value);
typedef long (GATEWAY_API *pGetDeviceToOpen)();

#ifdef GATEWAY_EXPORTS
#ifdef __cplusplus
extern "C" {
#endif

void GATEWAY_API SetDeviceToOpen(long value);
long GATEWAY_API GetDeviceToOpen();

#ifdef __cplusplus
}
#endif
#endif //GATEWAY_EXPORTS

#endif //_GATEWAY_H
