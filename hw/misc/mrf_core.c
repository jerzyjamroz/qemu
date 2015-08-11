
#include <qemu/module.h>

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
