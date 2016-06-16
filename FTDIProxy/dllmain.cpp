#include "stdafx.h"
#include <shlwapi.h>
#include "FTDIProxy.h"
#include "log.h"
#include "utils.h"

HINSTANCE proxy_handle;
ftdi_fcts *proxy;
HINSTANCE gateway_handle;
pGetDeviceToOpen GetDeviceToOpen;

#define LOAD_FCT(proxy_handle, name, type, dest) { \
    dest = (type)GetProcAddress(proxy_handle, #name); \
    if(dest == NULL) { \
        LOG(ERR,"Sardine: Can't load "#name" function"); \
        return FALSE; \
    } \
}

bool setup(HMODULE hModule) {
    LOG_START();
    LOG(INIT, "Setup");
    char fullpathname[1024];
    char libname[1024];
    if (!GetModuleFileNameA(hModule, fullpathname, 1024) != ERROR_SUCCESS) {
        LOG(ERR, "Can't get library name");
        return FALSE;
    }
    strcpy_s(libname, 1024, fullpathname);
    PathRemoveFileSpecA(libname);
    strcat_s(libname, 1024, "/");
    strcat_s(libname, 1024, "_ftd2xx.dll");
    proxy_handle = LoadLibraryA(libname);
    if (!proxy_handle) {
        LOG(ERR, "Can't load library %s", libname);
        return FALSE;
    }
    LOG(INIT, "Load functions");
    proxy = (ftdi_fcts *) malloc(sizeof(ftdi_fcts));
    LOAD_FCT(proxy_handle, FT_Open, pFT_Open, proxy->open);
    LOAD_FCT(proxy_handle, FT_OpenEx, pFT_OpenEx, proxy->openEx);
    LOAD_FCT(proxy_handle, FT_ListDevices, pFT_ListDevices, proxy->listDevices);
    LOAD_FCT(proxy_handle, FT_Close, pFT_Close, proxy->close);
    LOAD_FCT(proxy_handle, FT_Write, pFT_Write, proxy->write);
    LOAD_FCT(proxy_handle, FT_Read, pFT_Read, proxy->read);
    LOAD_FCT(proxy_handle, FT_IoCtl, pFT_IoCtl, proxy->ioCtl);
    LOAD_FCT(proxy_handle, FT_SetBaudRate, pFT_SetBaudRate, proxy->setBaudRate);
    LOAD_FCT(proxy_handle, FT_SetDivisor, pFT_SetDivisor, proxy->setDivisor);
    LOAD_FCT(proxy_handle, FT_SetDataCharacteristics, pFT_SetDataCharacteristics, proxy->setDataCharacteristics);
    LOAD_FCT(proxy_handle, FT_SetFlowControl, pFT_SetFlowControl, proxy->setFlowControl);
    LOAD_FCT(proxy_handle, FT_ResetDevice, pFT_ResetDevice, proxy->resetDevice);
    LOAD_FCT(proxy_handle, FT_SetDtr, pFT_SetDtr, proxy->setDtr);
    LOAD_FCT(proxy_handle, FT_ClrDtr, pFT_ClrDtr, proxy->clrDtr);
    LOAD_FCT(proxy_handle, FT_SetRts, pFT_SetRts, proxy->setRts);
    LOAD_FCT(proxy_handle, FT_ClrRts, pFT_ClrRts, proxy->clrRts);
    LOAD_FCT(proxy_handle, FT_GetModemStatus, pFT_GetModemStatus, proxy->getModemStatus);
    LOAD_FCT(proxy_handle, FT_SetChars, pFT_SetChars, proxy->setChars);
    LOAD_FCT(proxy_handle, FT_Purge, pFT_Purge, proxy->purge);
    LOAD_FCT(proxy_handle, FT_SetTimeouts, pFT_SetTimeouts, proxy->setTimeouts);
    LOAD_FCT(proxy_handle, FT_GetQueueStatus, pFT_GetQueueStatus, proxy->getQueueStatus);
    LOAD_FCT(proxy_handle, FT_SetEventNotification, pFT_SetEventNotification, proxy->setEventNotification);
    LOAD_FCT(proxy_handle, FT_GetEventStatus, pFT_GetEventStatus, proxy->getEventStatus);
    LOAD_FCT(proxy_handle, FT_GetStatus, pFT_GetStatus, proxy->getStatus);
    LOAD_FCT(proxy_handle, FT_SetBreakOn, pFT_SetBreakOn, proxy->setBreakOn);
    LOAD_FCT(proxy_handle, FT_SetBreakOff, pFT_SetBreakOff, proxy->setBreakOff);
    LOAD_FCT(proxy_handle, FT_SetWaitMask, pFT_SetWaitMask, proxy->setWaitMask);
    LOAD_FCT(proxy_handle, FT_WaitOnMask, pFT_WaitOnMask, proxy->waitOnMask);
    LOG(INIT, "Setup done");
    strcpy_s(libname, 1024, fullpathname);
    PathRemoveFileSpecA(libname);
    strcat_s(libname, 1024, "/");
    strcat_s(libname, 1024, "gateway.dll");
    gateway_handle = LoadLibraryA(libname);
    if (!gateway_handle) {
        LOG(ERR, "Can't load library %s", libname);
        return FALSE;
    }
    LOAD_FCT(gateway_handle, GetDeviceToOpen, pGetDeviceToOpen, GetDeviceToOpen);
    LOG(INIT, "Gateway Loaded");
    return TRUE;
}

void exitdll() {
    LOG(INIT, "Exitdll");
    free(proxy);
    FreeLibrary(proxy_handle);

    FreeLibrary(gateway_handle);
    LOG_STOP();
}

BOOL APIENTRY
DllMain(HMODULE hModule, DWORD ul_reason_for_call, LPVOID lpReserved) {
    UNUSED(lpReserved);
    LOG(INIT, "FTDI Proxy");
    LOG(INIT, "DllMain: %d", ul_reason_for_call);

    switch (ul_reason_for_call) {
        case
            DLL_PROCESS_ATTACH:
            if (!setup(hModule))
                return FALSE;
            break;
        case
            DLL_THREAD_ATTACH:
            break;
        case
            DLL_THREAD_DETACH:
            break;
        case
            DLL_PROCESS_DETACH:
            exitdll();
            break;
    }
    return TRUE;
}

