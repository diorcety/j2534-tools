#include "stdafx.h"

#include "internal.h"
#include <stdio.h>

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

PeriodicMessage::~PeriodicMessage() {
}

Configuration::~Configuration() {
}