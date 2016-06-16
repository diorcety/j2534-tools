#include "stdafx.h"
#include <shlwapi.h>
#include "ISO15765Proxy.h"
#include "log.h"
#include "utils.h"
#include "internal.h"

HINSTANCE proxy_handle1;
j2534_fcts *proxy1;

extern void create_library(j2534_fcts *proxy);
extern void delete_library();

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
    strcat_s(libname, 1024, "MVCIProxy.dll");
    if (!load_proxy(libname, &proxy_handle1, &proxy1)) {
        return FALSE;
    }
    create_library(proxy1);
    return TRUE;
}

void exitdll() {
    LOG(INIT, "Exitdll");
    delete_library();
    free(proxy1);
    FreeLibrary(proxy_handle1);

    LOG_STOP();
}

BOOL APIENTRY
DllMain(HMODULE hModule, DWORD ul_reason_for_call, LPVOID lpReserved) {
    UNUSED(lpReserved);
    LOG(INIT, "ISO15765 Proxy");
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

