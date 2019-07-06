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

#include "qemu/module.h"

#include "mrf_core.h"

static
void mrf_core_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->bus_type = TYPE_MRF_BUS;
}

static const TypeInfo mrf_core = {
    .name = TYPE_MRF_CORE,
    .parent = TYPE_DEVICE,
    .class_init = mrf_core_class_init,
    .abstract = true,
};

static const TypeInfo mrf_bus = {
    .name = TYPE_MRF_BUS,
    .parent = TYPE_BUS,
};

static void mrf_core_types(void)
{
    type_register_static(&mrf_bus);
    type_register_static(&mrf_core);
}

type_init(mrf_core_types)
