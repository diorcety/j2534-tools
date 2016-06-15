#pragma once
#ifndef _GATEWAY_H
#define _GATEWAY_H

#ifdef GATEWAY_EXPORTS
#define GATEWAY_API __stdcall
#else
#define GATEWAY_API __stdcall
#endif

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
