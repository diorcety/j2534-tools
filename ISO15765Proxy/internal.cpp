#include "stdafx.h"

#include "internal.h"
#include <stdio.h>

J2534Exception::J2534Exception(long code) : mCode(code) {
}

long J2534Exception::code() const {
    return mCode;
}

const char *J2534Exception::what() const noexcept {
    static char buffer[256];
    snprintf(buffer, 256, "Error code: %ld", mCode);
    return buffer;
}

/*
 * False destructors
 */

Library::~Library() {
}

Device::~Device() {
}

Channel::~Channel() {
}

MessageFilter::~MessageFilter() {
}