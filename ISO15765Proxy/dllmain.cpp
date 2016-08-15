#include "stdafx.h"
#include "ISO15765Proxy.h"
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
#endif //_WIN32

#ifdef __linux__
void *proxy_handle1;
#endif //__linux__


j2534_fcts *proxy1;

extern void create_library(j2534_fcts *proxy);
extern void delete_library();

#ifdef _WIN32
#define LOAD_FCT(proxy_handle, name, type, dest) { \
    dest = (type)GetProcAddress(proxy_handle, #name); \
    if(dest == NULL) { \
        LOG(ERR,"ISO17765: Can't load "#name" function"); \
        return FALSE; \
    } \
}
#endif //_WIN32

#ifdef __linux__
#define LOAD_FCT(proxy_handle, name, type, dest) { \
    dest = (type)dlsym(proxy_handle, #name); \
    if(dest == NULL) { \
        LOG(ERR,"ISO17765: Can't load "#name" function"); \
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

#ifdef _WIN32
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
    strcat_s(libname, 1024, "MVCIProxy.dll");
    if (!load_proxy(libname, &proxy_handle1, &proxy1)) {
        return false;
    }
    create_library(proxy1);
    return true;
}
#endif //_WIN32
#ifdef __linux__
bool setup(Dl_info *info) {
    LOG(INIT, "Setup");
    char fullpathname[1024];
    char libname[1024];
    strcpy(fullpathname, info->dli_fname);
    strcpy(libname, dirname(fullpathname));
    strcat(libname, "/");
    strcat(libname, "libMVCIProxy.so");
    if (!load_proxy(libname, &proxy_handle1, &proxy1)) {
        return false;
    }
    create_library(proxy1);
    return true;
}
#endif //__linux__

void exitdll() {
    LOG(INIT, "Exitdll");
    delete_library();
    free(proxy1);
#ifdef _WIN32
    FreeLibrary(proxy_handle1);
#endif //_WIN32
#ifdef __linux__
    dlclose(proxy_handle1);
#endif //__linux__
}

#ifdef _WIN32
BOOL APIENTRY
DllMain(HMODULE hModule, DWORD ul_reason_for_call, LPVOID lpReserved) {
    UNUSED(lpReserved);
    LOG_START();
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
            LOG_STOP();
            break;
    }
    return TRUE;
}
#endif //_WIN32

#ifdef __linux__
int __attribute__ ((constructor)) iso15765_proxy_init(void) {
    LOG_START();
    LOG(INIT, "ISO15765 Proxy");

    Dl_info dl_info;
    dladdr((const void*)iso15765_proxy_init, &dl_info);
    if(!setup(&dl_info)) {
        return 1;
    }
    return 0;
}

int __attribute__ ((destructor)) iso15765_proxy_fini(void) {
    exitdll();
    LOG_STOP();
    return 0;
}
#endif //__linux__