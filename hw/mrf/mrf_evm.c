/*
 * QEMU MRF PCI EVM emulation
 *
 * Michael Davisdaver
 * Copyright (c) 2018
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

#include "qemu/osdep.h"
#include "qemu/module.h"
#include "qapi/error.h"
#include "hw/qdev-core.h"
#include "hw/qdev-properties.h"
#include "exec/memory.h"

#include "mrf_core.h"

#define TYPE_MRF_EVM "mrf-evm"

#define MRF_EVM(obj) \
    OBJECT_CHECK(EVMState, (obj), TYPE_MRF_EVM)

#define MRF_EVM_DEVICE_CLASS(klass) \
     OBJECT_CLASS_CHECK(MRFEVMBaseClass, (klass), TYPE_MRF_EVM)
#define MRF_EVM_DEVICE_GET_CLASS(obj) \
    OBJECT_GET_CLASS(MRFEVMBaseClass, (obj), TYPE_MRF_EVM)

typedef struct {
    /*< private >*/
    MRFCore parent_obj;

    BusState *mbus;
    MRFCore *evg,
            *fct,
            *evru,
            *evrd;


} EVMState;

static
void mrf_evm_prop_fanout_get(Object *obj,
                             Visitor *v,
                             const char *name,
                             void *opaque,
                             Error **errp)
{
    EVMState *evm = opaque;

    /* Always get from EVG */
    object_property_get(OBJECT(evm->evg), v, name, errp);
}

static
void mrf_evm_prop_fanout_set(Object *obj,
                             Visitor *v,
                             const char *name,
                             void *opaque,
                             Error **errp)
{
    EVMState *evm = opaque;

    /* Fanout setting to all sub-units.
     * Assumes Visitor is not modified.
     */
    object_property_set(OBJECT(evm->evg), v, name, errp);
    object_property_set(OBJECT(evm->evru), v, name, errp);
    object_property_set(OBJECT(evm->evrd), v, name, errp);
}

static
ObjectProperty* mrf_evm_object_property_fanout(EVMState *evm, const char *name, Error **errp)
{
    ObjectProperty *prop = object_property_find(OBJECT(evm->evg), name, errp);
    if(!prop) return prop;

    return object_property_add(OBJECT(evm), name, prop->type,
                               mrf_evm_prop_fanout_get,
                               mrf_evm_prop_fanout_set,
                               NULL,
                               evm, errp);
}

static
void mrf_evm_init(Object *obj)
{
    DeviceState *dev = DEVICE(obj), *sub;
    MRFCore *core = MRF_CORE(obj);
    EVMState *evm = MRF_EVM(obj);

    evm->mbus = qbus_create(TYPE_MRF_BUS, dev, "local");

    sub = qdev_create(evm->mbus, "mrf-evg");
    evm->evg = MRF_CORE(sub);

    sub = qdev_create(evm->mbus, "mrf-evr");
    evm->evru = MRF_CORE(sub);

    sub = qdev_create(evm->mbus, "mrf-evr");
    evm->evrd = MRF_CORE(sub);

    evm->fct = NULL; /* TODO */

    memory_region_init(&core->core, OBJECT(evm),
                       "mrf-evm", 0x40000);

    memory_region_add_subregion(&core->core, 0x00000, &evm->evg->core);
    /* TODO FCT @0x10000 */
    memory_region_add_subregion(&core->core, 0x20000, &evm->evrd->core);
    memory_region_add_subregion(&core->core, 0x30000, &evm->evru->core);

    /* only EVG IRQs are passed through, others are ignored */
    qdev_pass_gpios(DEVICE(evm->evg), dev, NULL);

    /* all sub-units have same endian-ness, bridge-type, and form factor */
    mrf_evm_object_property_fanout(evm, "endian", &error_abort);
    mrf_evm_object_property_fanout(evm, "bridge-type", &error_abort);
    mrf_evm_object_property_fanout(evm, "mrf-type", &error_abort);
    /* all (maybe) have same version */
    mrf_evm_object_property_fanout(evm, "version", &error_abort);
    /* TODO: internal event link routing */
    object_property_add_alias(obj, "chardev", OBJECT(evm->evg), "chardev", &error_abort);
}

static void mrf_evm_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    set_bit(DEVICE_CATEGORY_MISC, dc->categories);
    dc->desc = "Micro Research EVM core";
    dc->user_creatable = false;
}

static const TypeInfo mrf_evm_info = {
    .name          = TYPE_MRF_EVM,
    .parent        = TYPE_MRF_CORE,
    .instance_size = sizeof(EVMState),
    .instance_init = mrf_evm_init,
    .class_init    = mrf_evm_class_init,
};

static void mrf_evm_register_types(void)
{
    type_register_static(&mrf_evm_info);
}

type_init(mrf_evm_register_types)
