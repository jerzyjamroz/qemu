/*
 * MRF emulation common code
 *
 * Michael Davisdaver
 * Copyright (c) 2017
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, see <http://www.gnu.org/licenses/>.
 */
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
