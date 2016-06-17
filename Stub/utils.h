#pragma once

#ifndef _UTILS_H
#define _UTILS_H

#include <memory>
#include <assert.h>
#include "log.h"

#define UNUSED(x) (void)(x)

#define DEFINE_SHARED(Cls) \
    class Cls; \
    typedef std::shared_ptr<Cls> Cls##Ptr; \
    typedef std::weak_ptr<Cls> Cls##WeakPtr;

#define ASSERT(x) {if(!(x)) {LOG(ERR, "Assert %s at %d", __FILE__, __LINE__); while(true) {}} assert(x);}

#endif /* _UTILS_H */