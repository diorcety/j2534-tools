#include "stdafx.h"
#include "Stub.h"
#include "log.h"
#include "utils.h"

#ifdef __linux__
#include <dlfcn.h>
#endif //__linux__

#ifdef _WIN32
BOOL APIENTRY
DllMain(HMODULE hModule, DWORD ul_reason_for_call, LPVOID lpReserved) {
    LOG_START();
    UNUSED(hModule);
    UNUSED(lpReserved);
    LOG(INIT, "Stub Proxy");
    LOG(INIT, "DllMain: %d", ul_reason_for_call);

    switch (ul_reason_for_call) {
        case
            DLL_PROCESS_ATTACH:
            break;
        case
            DLL_THREAD_ATTACH:
            break;
        case
            DLL_THREAD_DETACH:
            break;
        case
            DLL_PROCESS_DETACH:
            LOG_STOP();
            break;
    }
    return TRUE;
}
#endif //_WIN32

#ifdef __linux__
int __attribute__ ((constructor)) iso15765_proxy_init(void) {
    LOG_START();
    return 0;
}

int __attribute__ ((destructor)) iso15765_proxy_fini(void) {
    LOG_STOP();
    return 0;
}
#endif //__linux__