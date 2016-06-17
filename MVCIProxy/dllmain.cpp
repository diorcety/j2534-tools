#include "stdafx.h"
#include "MVCIProxy.h"
#include "log.h"
#include "utils.h"

#include <string.h>
#include <stdlib.h>

#ifdef _WIN32
#include <shlwapi.h>
#endif //_WIN32

#ifdef __linux__
#include <dlfcn.h>
#include <libgen.h>
#endif //__linux__

#ifdef _WIN32
HINSTANCE proxy_handle1;
HINSTANCE proxy_handle2;
HINSTANCE gateway_handle;
#endif //_WIN32

#ifdef __linux__
void *proxy_handle1;
void *proxy_handle2;
void *gateway_handle;
#endif //__linux__

j2534_fcts *proxy1;
j2534_fcts *proxy2;
pSetDeviceToOpen SetDeviceToOpen;

#ifdef _WIN32
#define LOAD_FCT(proxy_handle, name, type, dest) { \
    dest = (type)GetProcAddress(proxy_handle, #name); \
    if(dest == NULL) { \
        LOG(ERR,"MVCIProxy: Can't load "#name" function"); \
        return FALSE; \
    } \
}
#endif //_WIN32

#ifdef __linux__
#define LOAD_FCT(proxy_handle, name, type, dest) { \
    dest = (type)dlsym(proxy_handle, #name); \
    if(dest == NULL) { \
        LOG(ERR,"MVCIProxy: Can't load "#name" function"); \
        return false; \
    } \
}
#endif //__linux__

#ifdef _WIN32
bool load_proxy(char *libname, HINSTANCE *proxy_handle, j2534_fcts **proxy) {
#endif //_WIN32
#ifdef __linux__
bool load_proxy(char *libname, void **proxy_handle, j2534_fcts **proxy) {
#endif //__linux__

#ifdef _WIN32
    *proxy_handle = LoadLibraryA(libname);
#endif //_WIN32
#ifdef __linux__
    *proxy_handle = dlopen(libname, RTLD_LAZY);
#endif //__linux__
    if (!(*proxy_handle)) {
        LOG(ERR, "Can't load library %s", libname);
        return false;
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
    return true;
}

#ifdef __WIN32
bool setup(HMODULE hModule) {
    LOG(INIT, "Setup");
    char fullpathname[1024];
    char libname[1024];
    if (!GetModuleFileNameA(hModule, fullpathname, 1024) != ERROR_SUCCESS) {
        LOG(ERR, "Can't get library name");
        return false;
    }

    strcpy_s(libname, 1024, fullpathname);
    PathRemoveFileSpecA(libname);
    strcat_s(libname, 1024, "/");
    strcat_s(libname, 1024, "MVCI32.dll");
    if (!load_proxy(libname, &proxy_handle1, &proxy1)) {
        return false;
    }

#ifdef ENABLE_ALTERNATE
    strcpy_s(libname, 1024, fullpathname);
    PathRemoveFileSpecA(libname);
    strcat_s(libname, 1024, "/");
    strcat_s(libname, 1024, "MVCI32_2.dll");
    if (!load_proxy(libname, &proxy_handle2, &proxy2)) {
        return false;
    }
#endif /* ENABLE_ALTERNATE */

    strcpy_s(libname, 1024, fullpathname);
    PathRemoveFileSpecA(libname);
    strcat_s(libname, 1024, "/");
    strcat_s(libname, 1024, "gateway.dll");
    gateway_handle = LoadLibraryA(libname);
    if (!gateway_handle) {
        LOG(ERR, "Can't load library %s", libname);
        return false;
    }
    LOAD_FCT(gateway_handle, SetDeviceToOpen, pSetDeviceToOpen, SetDeviceToOpen);
    LOG(INIT, "Gateway Loaded");
    return true;
}
#endif //_WIN32
#ifdef __linux__
bool setup(Dl_info *info) {
    LOG_START();
    LOG(INIT, "Setup");
    char fullpathname[1024];
    char libname[1024];

    strcpy(fullpathname, info->dli_fname);
    strcpy(libname, dirname(fullpathname));
    strcat(libname, "/");
    strcat(libname, "libMVCI32.so");
    if (!load_proxy(libname, &proxy_handle1, &proxy1)) {
        return false;
    }

#ifdef ENABLE_ALTERNATE
    strcpy(fullpathname, info->dli_fname);
    strcpy(libname, dirname(fullpathname));
    strcat(libname, "/");
    strcat(libname, "libMVCI32_2.so");
    if (!load_proxy(libname, &proxy_handle2, &proxy2)) {
        return false;
    }
#endif /* ENABLE_ALTERNATE */

    strcpy(fullpathname, info->dli_fname);
    strcpy(libname, dirname(fullpathname));
    strcat(libname, "/");
    strcat(libname, "libgateway.so");
    gateway_handle = dlopen(libname, RTLD_LAZY);
    if (!gateway_handle) {
        LOG(ERR, "Can't load library %s", libname);
        return false;
    }
    LOAD_FCT(gateway_handle, SetDeviceToOpen, pSetDeviceToOpen, SetDeviceToOpen);
    LOG(INIT, "Gateway Loaded");
    return true;
}
#endif //__linux__

void exitdll() {
    LOG(INIT, "Exitdll");
    free(proxy1);
#ifdef _WIN32
    FreeLibrary(proxy_handle1);
#endif //_WIN32
#ifdef __linux__
    dlclose(proxy_handle1);
#endif //__linux__

#ifdef ENABLE_ALTERNATE
    free(proxy2);
#ifdef _WIN32
    FreeLibrary(proxy_handle2);
#endif //_WIN32
#ifdef __linux__
    dlclose(proxy_handle2);
#endif //__linux__
#endif /* ENABLE_ALTERNATE */

#ifdef _WIN32
    FreeLibrary(gateway_handle);
#endif //_WIN32
#ifdef __linux__
    dlclose(gateway_handle);
#endif //__linux__
}

#ifdef _WIN32
BOOL APIENTRY
DllMain(HMODULE hModule, DWORD ul_reason_for_call, LPVOID lpReserved) {
    UNUSED(lpReserved);
    LOG_START();
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
            LOG_STOP();
            break;
    }
    return
            TRUE;
}
#endif //_WIN32

#ifdef __linux__
int __attribute__ ((constructor)) mvi_proxy_init(void) {
    LOG_START();
    LOG(INIT, "MVCI Proxy");

    Dl_info dl_info;
    dladdr((const void*)mvi_proxy_init, &dl_info);
    if(!setup(&dl_info)) {
        return 1;
    }
    return 0;
}

int __attribute__ ((destructor)) mvci_proxy_fini(void) {
    exitdll();
    LOG_STOP();
    return 0;
}
#endif //__linux__
