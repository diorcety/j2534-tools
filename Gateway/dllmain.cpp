#include "stdafx.h"
#include <windows.h>
#include "utils.h"

BOOL APIENTRY
DllMain(HMODULE hModule, DWORD ul_reason_for_call, LPVOID lpReserved) {
    UNUSED(hModule);
    UNUSED(lpReserved);
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
            break;
    }
    return TRUE;
}

