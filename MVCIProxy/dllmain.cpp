#include "stdafx.h"
#include <shlwapi.h>
#include "MVCIProxy.h"
#include "log.h"
#include "utils.h"

HINSTANCE proxy_handle1;
HINSTANCE proxy_handle2;
j2534_fcts *proxy1;
j2534_fcts *proxy2;
HINSTANCE gateway_handle;
pSetDeviceToOpen SetDeviceToOpen;

#define LOAD_FCT(proxy_handle, name, type, dest) { \
    dest = (type)GetProcAddress(proxy_handle, #name); \
    if(dest == NULL) { \
        LOG(ERR,"Sardine: Can't load "#name" function"); \
        return FALSE; \
    } \
}

bool load_proxy(char *libname, HINSTANCE *proxy_handle, j2534_fcts **proxy) {
    *proxy_handle = LoadLibraryA(libname);
    if (!(*proxy_handle)) {
        LOG(ERR, "Can't load library %s", libname);
        return FALSE;
    }
    LOG(INIT, "Load functions from %s", libname);
    *proxy = (j2534_fcts *) malloc(sizeof(j2534_fcts));
    LOAD_FCT(*proxy_handle, PassThruOpen, PTOPEN, (*proxy)->passThruOpen);
    LOAD_FCT(*proxy_handle, PassThruClose, PTCLOSE, (*proxy)->passThruClose);
    LOAD_FCT(*proxy_handle, PassThruConnect, PTCONNECT, (*proxy)->passThruConnect);
    LOAD_FCT(*proxy_handle, PassThruDisconnect, PTDISCONNECT, (*proxy)->passThruDisconnect);
    LOAD_FCT(*proxy_handle, PassThruReadMsgs, PTREADMSGS, (*proxy)->passThruReadMsgs);
    LOAD_FCT(*proxy_handle, PassThruWriteMsgs, PTWRITEMSGS, (*proxy)->passThruWriteMsgs);
    LOAD_FCT(*proxy_handle, PassThruStartPeriodicMsg, PTSTARTPERIODICMSG, (*proxy)->passThruStartPeriodicMsg);
    LOAD_FCT(*proxy_handle, PassThruStopPeriodicMsg, PTSTOPPERIODICMSG, (*proxy)->passThruStopPeriodicMsg);
    LOAD_FCT(*proxy_handle, PassThruStartMsgFilter, PTSTARTMSGFILTER, (*proxy)->passThruStartMsgFilter);
    LOAD_FCT(*proxy_handle, PassThruStopMsgFilter, PTSTOPMSGFILTER, (*proxy)->passThruStopMsgFilter);
    LOAD_FCT(*proxy_handle, PassThruSetProgrammingVoltage, PTSETPROGRAMMINGVOLTAGE,
             (*proxy)->passThruSetProgrammingVoltage);
    LOAD_FCT(*proxy_handle, PassThruReadVersion, PTREADVERSION, (*proxy)->passThruReadVersion);
    LOAD_FCT(*proxy_handle, PassThruGetLastError, PTGETLASTERROR, (*proxy)->passThruGetLastError);
    LOAD_FCT(*proxy_handle, PassThruIoctl, PTIOCTL, (*proxy)->passThruIoctl);
    LOG(INIT, "Setup done");
    return TRUE;
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
    strcat_s(libname, 1024, "MVCI32.dll");
    if (!load_proxy(libname, &proxy_handle1, &proxy1)) {
        return FALSE;
    }

#ifdef ENABLE_ALTERNATE
    strcpy_s(libname, 1024, fullpathname);
    PathRemoveFileSpecA(libname);
    strcat_s(libname, 1024, "/");
    strcat_s(libname, 1024, "MVCI32_2.dll");
    if (!load_proxy(libname, &proxy_handle2, &proxy2)) {
        return FALSE;
    }
#endif /* ENABLE_ALTERNATE */

    strcpy_s(libname, 1024, fullpathname);
    PathRemoveFileSpecA(libname);
    strcat_s(libname, 1024, "/");
    strcat_s(libname, 1024, "gateway.dll");
    gateway_handle = LoadLibraryA(libname);
    if (!gateway_handle) {
        LOG(ERR, "Can't load library %s", libname);
        return FALSE;
    }
    LOAD_FCT(gateway_handle, SetDeviceToOpen, pSetDeviceToOpen, SetDeviceToOpen);
    LOG(INIT, "Gateway Loaded");
    return TRUE;
}

void exitdll() {
    LOG(INIT, "Exitdll");
    free(proxy1);
    FreeLibrary(proxy_handle1);

#ifdef ENABLE_ALTERNATE
    free(proxy2);
    FreeLibrary(proxy_handle2);
#endif /* ENABLE_ALTERNATE */
    FreeLibrary(gateway_handle);
    LOG_STOP();
}

BOOL APIENTRY
DllMain(HMODULE hModule, DWORD ul_reason_for_call, LPVOID lpReserved) {
    UNUSED(lpReserved);
    LOG(INIT, "MVCI Proxy");
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
    return
            TRUE;
}

