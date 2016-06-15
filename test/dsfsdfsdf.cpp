#include "stdafx.h"
#include <windows.h>
#include <tchar.h>
#include <shlwapi.h>
#include <stdio.h>
#include "FTD2XX.H"
#include "j2534_v0404.h"
#include "log.h"

typedef struct {
	pFT_Open open;
	pFT_OpenEx openEx;
	pFT_ListDevices listDevices;
	pFT_Close close;
	pFT_Write write;
	pFT_Read read;
	pFT_IoCtl ioCtl;
	pFT_SetBaudRate setBaudRate;
	pFT_SetDivisor setDivisor;
	pFT_SetDataCharacteristics setDataCharacteristics;
	pFT_SetFlowControl setFlowControl;
	pFT_ResetDevice resetDevice;
	pFT_SetDtr setDtr;
	pFT_ClrDtr clrDtr;
	pFT_SetRts setRts;
	pFT_ClrRts clrRts;
	pFT_GetModemStatus getModemStatus;
	pFT_SetChars setChars;
	pFT_Purge purge;
	pFT_SetTimeouts setTimeouts;
	pFT_GetQueueStatus getQueueStatus;
	pFT_SetEventNotification setEventNotification;
	pFT_GetEventStatus getEventStatus;
	pFT_GetStatus getStatus;
	pFT_SetBreakOn setBreakOn;
	pFT_SetBreakOff setBreakOff;
	pFT_SetWaitMask setWaitMask;
	pFT_WaitOnMask waitOnMask;	
} ftdi_fcts;

typedef struct {
	PTOPEN passThruOpen;
	PTCLOSE passThruClose;
	PTCONNECT passThruConnect;
	PTDISCONNECT passThruDisconnect;
	PTREADMSGS passThruReadMsgs;
	PTWRITEMSGS passThruWriteMsgs;
	PTSTARTPERIODICMSG passThruStartPeriodicMsg;
	PTSTOPPERIODICMSG passThruStopPeriodicMsg;
	PTSTARTMSGFILTER passThruStartMsgFilter;
	PTSTOPMSGFILTER passThruStopMsgFilter;
	PTSETPROGRAMMINGVOLTAGE passThruSetProgrammingVoltage;
	PTREADVERSION passThruReadVersion;
	PTGETLASTERROR passThruGetLastError;
	PTIOCTL passThruIoctl;
} j2534_fcts;

HINSTANCE j2534_proxy_handle;
j2534_fcts *j2534_proxy;
HINSTANCE jtdi_proxy_handle;
ftdi_fcts *ftdi_proxy;

#define LOAD_FCT(proxy_handle, name, type, dest) { \
	dest = (type)GetProcAddress(proxy_handle, #name); \
	if(dest == NULL) { \
		LOG(ERR,"Sardine: Can't load "#name" function"); \
		return FALSE; \
	} \
}

bool j2534_setup(HMODULE hModule)
{
	LOG_START();
	LOG(INIT,"Setup");
	char fullpathname[1024];
	char libname[1024];
	if (!GetModuleFileNameA(hModule, fullpathname, 1024) != ERROR_SUCCESS) {
		LOG(ERR,"Can't get library name");
		return FALSE;
	}
	strcpy_s(libname, 1024, fullpathname);
	PathRemoveFileSpecA(libname);
	strcat_s(libname, 1024, "/");
	strcat_s(libname, 1024, "MVCIProxy.dll");
	j2534_proxy_handle = LoadLibraryA("C:\\Program Files (x86)\\XHorse Electronics\\MVCI Driver for TOYOTA TIS\\MVCIProxy.dll");
	if (!j2534_proxy_handle) {
		LOG(ERR,"Can't load library %s", libname);
		return FALSE;
	}
	LOG(INIT, "Load functions");
	j2534_proxy = (j2534_fcts * )malloc(sizeof(j2534_fcts));
	LOAD_FCT(j2534_proxy_handle, PassThruOpen, PTOPEN, j2534_proxy->passThruOpen);
	LOAD_FCT(j2534_proxy_handle, PassThruClose, PTCLOSE, j2534_proxy->passThruClose);
	LOAD_FCT(j2534_proxy_handle, PassThruConnect, PTCONNECT, j2534_proxy->passThruConnect);
	LOAD_FCT(j2534_proxy_handle, PassThruDisconnect, PTDISCONNECT, j2534_proxy->passThruDisconnect);
	LOAD_FCT(j2534_proxy_handle, PassThruReadMsgs, PTREADMSGS, j2534_proxy->passThruReadMsgs);
	LOAD_FCT(j2534_proxy_handle, PassThruWriteMsgs, PTWRITEMSGS, j2534_proxy->passThruWriteMsgs);
	LOAD_FCT(j2534_proxy_handle, PassThruStartPeriodicMsg, PTSTARTPERIODICMSG, j2534_proxy->passThruStartPeriodicMsg);
	LOAD_FCT(j2534_proxy_handle, PassThruStopPeriodicMsg, PTSTOPPERIODICMSG, j2534_proxy->passThruStopPeriodicMsg);
	LOAD_FCT(j2534_proxy_handle, PassThruStartMsgFilter, PTSTARTMSGFILTER, j2534_proxy->passThruStartMsgFilter);
	LOAD_FCT(j2534_proxy_handle, PassThruStopMsgFilter, PTSTOPMSGFILTER, j2534_proxy->passThruStopMsgFilter);
	LOAD_FCT(j2534_proxy_handle, PassThruSetProgrammingVoltage, PTSETPROGRAMMINGVOLTAGE, j2534_proxy->passThruSetProgrammingVoltage);
	LOAD_FCT(j2534_proxy_handle, PassThruReadVersion, PTREADVERSION, j2534_proxy->passThruReadVersion);
	LOAD_FCT(j2534_proxy_handle, PassThruGetLastError, PTGETLASTERROR, j2534_proxy->passThruGetLastError);
	LOAD_FCT(j2534_proxy_handle, PassThruIoctl, PTIOCTL, j2534_proxy->passThruIoctl);
	LOG(INIT,"Setup done");
	return TRUE;
}

bool ftdi_setup(HMODULE hModule)
{
	LOG_START();
	LOG(INIT,"Setup");
	char fullpathname[1024];
	char libname[1024];
	if (!GetModuleFileNameA(hModule, fullpathname, 1024) != ERROR_SUCCESS) {
		LOG(ERR,"Can't get library name");
		return FALSE;
	}
	strcpy_s(libname, 1024, fullpathname);
	PathRemoveFileSpecA(libname);
	strcat_s(libname, 1024, "/");
	strcat_s(libname, 1024, "_ftd2xx.dll");
	jtdi_proxy_handle = LoadLibraryA(libname);
	if (!jtdi_proxy_handle) {
		LOG(ERR,"Can't load library %s", libname);
		return FALSE;
	}
	LOG(INIT, "Load functions");
	ftdi_proxy = (ftdi_fcts * )malloc(sizeof(ftdi_fcts));
	LOAD_FCT(jtdi_proxy_handle, FT_Open, pFT_Open, ftdi_proxy->open);
	LOAD_FCT(jtdi_proxy_handle, FT_OpenEx, pFT_OpenEx, ftdi_proxy->openEx);
	LOAD_FCT(jtdi_proxy_handle, FT_ListDevices, pFT_ListDevices, ftdi_proxy->listDevices);
	LOAD_FCT(jtdi_proxy_handle, FT_Close, pFT_Close, ftdi_proxy->close);
	LOAD_FCT(jtdi_proxy_handle, FT_Write, pFT_Write, ftdi_proxy->write);
	LOAD_FCT(jtdi_proxy_handle, FT_Read, pFT_Read, ftdi_proxy->read);
	LOAD_FCT(jtdi_proxy_handle, FT_IoCtl, pFT_IoCtl, ftdi_proxy->ioCtl);
	LOAD_FCT(jtdi_proxy_handle, FT_SetBaudRate, pFT_SetBaudRate, ftdi_proxy->setBaudRate);
	LOAD_FCT(jtdi_proxy_handle, FT_SetDivisor, pFT_SetDivisor, ftdi_proxy->setDivisor);
	LOAD_FCT(jtdi_proxy_handle, FT_SetDataCharacteristics, pFT_SetDataCharacteristics, ftdi_proxy->setDataCharacteristics);
	LOAD_FCT(jtdi_proxy_handle, FT_SetFlowControl, pFT_SetFlowControl, ftdi_proxy->setFlowControl);
	LOAD_FCT(jtdi_proxy_handle, FT_ResetDevice, pFT_ResetDevice, ftdi_proxy->resetDevice);
	LOAD_FCT(jtdi_proxy_handle, FT_SetDtr, pFT_SetDtr, ftdi_proxy->setDtr);
	LOAD_FCT(jtdi_proxy_handle, FT_ClrDtr, pFT_ClrDtr, ftdi_proxy->clrDtr);
	LOAD_FCT(jtdi_proxy_handle, FT_SetRts, pFT_SetRts, ftdi_proxy->setRts);
	LOAD_FCT(jtdi_proxy_handle, FT_ClrRts, pFT_ClrRts, ftdi_proxy->clrRts);
	LOAD_FCT(jtdi_proxy_handle, FT_GetModemStatus, pFT_GetModemStatus, ftdi_proxy->getModemStatus);
	LOAD_FCT(jtdi_proxy_handle, FT_SetChars, pFT_SetChars, ftdi_proxy->setChars);
	LOAD_FCT(jtdi_proxy_handle, FT_Purge, pFT_Purge, ftdi_proxy->purge);
	LOAD_FCT(jtdi_proxy_handle, FT_SetTimeouts, pFT_SetTimeouts, ftdi_proxy->setTimeouts);
	LOAD_FCT(jtdi_proxy_handle, FT_GetQueueStatus, pFT_GetQueueStatus, ftdi_proxy->getQueueStatus);
	LOAD_FCT(jtdi_proxy_handle, FT_SetEventNotification, pFT_SetEventNotification, ftdi_proxy->setEventNotification);
	LOAD_FCT(jtdi_proxy_handle, FT_GetEventStatus, pFT_GetEventStatus, ftdi_proxy->getEventStatus);
	LOAD_FCT(jtdi_proxy_handle, FT_GetStatus, pFT_GetStatus, ftdi_proxy->getStatus);
	LOAD_FCT(jtdi_proxy_handle, FT_SetBreakOn, pFT_SetBreakOn, ftdi_proxy->setBreakOn);
	LOAD_FCT(jtdi_proxy_handle, FT_SetBreakOff, pFT_SetBreakOff, ftdi_proxy->setBreakOff);
	LOAD_FCT(jtdi_proxy_handle, FT_SetWaitMask, pFT_SetWaitMask, ftdi_proxy->setWaitMask);
	LOAD_FCT(jtdi_proxy_handle, FT_WaitOnMask, pFT_WaitOnMask, ftdi_proxy->waitOnMask);
	LOG(INIT,"Setup done");
	return TRUE;
}

HMODULE GetCurrentModule()
{
    DWORD flags = GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS;
    HMODULE hm = 0;
    ::GetModuleHandleEx( flags, reinterpret_cast<LPCTSTR>( GetCurrentModule ), &hm );   
    return hm;
}

int _tmain(int argc, _TCHAR* argv[])
{
    (void)(argc);
    (void)(argv);
	//ftdi_setup(GetCurrentModule());
	j2534_setup(GetCurrentModule());
	/*
	DWORD devIndex = 0; // first device
	char Buffer[64]; // more than enough room!
	FT_STATUS ftStatus = ftdi_proxy->listDevices((PVOID)devIndex,Buffer,FT_LIST_BY_INDEX|FT_OPEN_BY_SERIAL_NUMBER);
	if (ftStatus == FT_OK) {
		printf("sdfsfdf: %s\n", Buffer);
	}
	else {
	    printf("sdfsfdf error: %d\n", ftStatus);
	}
	*/
	system("pause");
	unsigned long pid;
	char ErrorMsg[80];
	unsigned long status;
	status = j2534_proxy->passThruOpen(NULL, &pid);
	printf("sdfsfdf: %ld\n", status);
	if (status != STATUS_NOERROR)
	{
		// Failed! Get descriptive error string.
		j2534_proxy->passThruGetLastError(ErrorMsg);

		printf("%s\n", ErrorMsg);
	}
	system("pause");
	unsigned long pChannel1;
	status = j2534_proxy->passThruConnect(pid, ISO15765, 0, 1000000, &pChannel1);
	printf("sdfsfdf: %ld\n", status);
	if (status != STATUS_NOERROR)
	{
		// Failed! Get descriptive error string.
		j2534_proxy->passThruGetLastError(ErrorMsg);

		printf("%s\n", ErrorMsg);
	}
	system("pause");
	unsigned long pChannel2;
	status = j2534_proxy->passThruConnect(pid, ISO15765_PS, 0, 1000000, &pChannel2);
	printf("sdfsfdf: %ld\n", status);
	if (status != STATUS_NOERROR)
	{
		// Failed! Get descriptive error string.
		j2534_proxy->passThruGetLastError(ErrorMsg);

		printf("%s\n", ErrorMsg);
	}
	system("pause");
	return 0;
}

