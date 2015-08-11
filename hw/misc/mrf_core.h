#ifndef MRF_CORE_H
#define MRF_CORE_H

#include "qemu/osdep.h"
#include "hw/qdev-core.h"
#include "exec/memory.h"

#define TYPE_MRF_CORE "mrf-core"

#define MRF_CORE(obj) OBJECT_CHECK(MRFCore, (obj), TYPE_MRF_CORE)
#define MRF_CORE_CLASS(klass) OBJECT_CLASS_CHECK(MRFCore, klass, TYPE_MRF_CORE)
#define MRF_CORE_GET_CLASS(obj) OBJECT_GET_CLASS(MRFCore, obj, TYPE_MRF_CORE)

typedef struct MRFCore {
    DeviceState parent_obj;

    MemoryRegion core;
} MRFCore;

#define TYPE_MRF_BUS "mrf-bus"

#endif // MRF_CORE_H
