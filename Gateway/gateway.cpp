#include "stdafx.h"
#include "gateway.h"

static long altOpening = 0;


void GATEWAY_API SetDeviceToOpen(long value) {
    altOpening = value;
}


long GATEWAY_API GetDeviceToOpen() {
    return altOpening;
}