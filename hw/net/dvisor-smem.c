/*
 *
 * dvisor-smem PCI device
 * add by Dd
 * this device is used to manage the shared memory among many dockervisor-containers
 *
 */

#include "qemu/osdep.h"
#include "hw/hw.h"
#include "hw/pci/pci.h"
#include "qemu/event_notifier.h"
#include "qemu/osdep.h"
#include "hw/net/dvisor.h"

#ifdef DOCKERVISOR_DEBUG
#define DBG(fmt, ...) do { \
        fprintf(stderr, "ptnet-smdev: " fmt "\n", ## __VA_ARGS__); \
    } while (0)
#else
#define DBG(fmt, ...) do {} while (0)
#endif

static uint64_t
upper_pow2(uint32_t v) {
    /* from bit-twiddling hacks */
    v--;
    v |= v >> 1;
    v |= v >> 2;
    v |= v >> 4;
    v |= v >> 8;
    v |= v >> 16;
    v++;
    return v;
}

typedef struct PTNetmapMemDevState {
    /*< private >*/
    PCIDevice parent_obj;

    /*< public >*/
    MemoryRegion io_bar;        /* ptnetmap register BAR */
    MemoryRegion mem_bar;       /* ptnetmap shared memory BAR */
    MemoryRegion mem_ram;       /* ptnetmap shared memory subregion */
    void *mem_ptr;              /* host virtual pointer to netmap memory */
    uint64_t mem_size;		/* size of shared memory*/
    int fd; 			/* file descriptor pointor to the dv_mm device*/

   char csb[4096];
 //   char * csb;
    QTAILQ_ENTRY(PTNetmapMemDevState) next;
} PTNetmapMemDevState;

static QTAILQ_HEAD(, PTNetmapMemDevState) ptn_memdevs = QTAILQ_HEAD_INITIALIZER(ptn_memdevs);

#define TYPE_PTNETMAP_MEMDEV	"Dockervisor_shared_memory"

#define PTNETMAP_MEMDEV(obj) \
    OBJECT_CHECK(PTNetmapMemDevState, (obj), TYPE_PTNETMAP_MEMDEV)

static void
ptnetmap_memdev_io_write(void *opaque, hwaddr addr, uint64_t val,
                         unsigned size)
{
    DBG("invalid I/O write [addr 0x%lx]", addr);
}

static uint64_t
ptnetmap_memdev_io_read(void *opaque, hwaddr addr, unsigned size)
{
    PTNetmapMemDevState *memd = opaque;
    uint64_t ret = 0;
#if 1
    switch (addr) {
        case PTNET_MDEV_IO_MEMSIZE_LO:
            ret = memd->mem_size & 0xffffffff;
            break;
        case PTNET_MDEV_IO_MEMSIZE_HI:
            ret = memd->mem_size >> 32;
            break;
        default:
            DBG("invalid I/O read [addr 0x%lx]", addr);
            return 0;
    }
#endif
    DBG("I/O read: addr 0x%lx, val 0x%lx", addr, ret);

    return ret;
}

static const MemoryRegionOps ptnetmap_memdev_io_ops = {
    .read = ptnetmap_memdev_io_read,
    .write = ptnetmap_memdev_io_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

static int
ptnetmap_memdev_init(PCIDevice *dev)
{
    PTNetmapMemDevState *memd = PTNETMAP_MEMDEV(dev);
    uint8_t *pci_conf;
    uint64_t size;

    pci_conf = dev->config;
    pci_conf[PCI_INTERRUPT_PIN] = 0; /* no interrupt pin */

    /* init register PCI_BAR */
    size = upper_pow2(DOCKERVISOR_MDEV_IO_END);
    memory_region_init_io(&memd->io_bar, OBJECT(memd),
            &ptnetmap_memdev_io_ops, memd, "dvisorsmem-io-bar", size);
    pci_register_bar(dev, DOCKERVISOR_IO_PCI_BAR, PCI_BASE_ADDRESS_SPACE_IO,
            &memd->io_bar);

    /* init PCI_BAR to map netmap memory into the guest */
    if (memd->mem_ptr) {
    //if (memd->csb) {
	*((int *)memd->mem_ptr) = 0x1;
	*((int *)(memd->csb)) = 0x1;
        size = upper_pow2(memd->mem_size);
        DBG("map BAR size %lx (%lu MiB)", size, size >> 20);

        memory_region_init(&memd->mem_bar, OBJECT(memd),
                           "dvisorsmem-mem-bar", size);
        memory_region_init_ram_ptr(&memd->mem_ram, OBJECT(memd),
                                   "dvisorsmem-mem-ram", memd->mem_size,
				   memd->mem_ptr);
        memory_region_set_fd(&memd->mem_ram, memd->fd);
        memory_region_add_subregion(&memd->mem_bar, 0, &memd->mem_ram);
        vmstate_register_ram(&memd->mem_ram, DEVICE(memd));
        pci_register_bar(dev, DOCKERVISOR_MEM_PCI_BAR,
                PCI_BASE_ADDRESS_SPACE_MEMORY  |
                PCI_BASE_ADDRESS_MEM_PREFETCH | PCI_BASE_ADDRESS_MEM_TYPE_64 , &memd->mem_bar);
    }else{
    	return -1;
    }

    QTAILQ_INSERT_TAIL(&ptn_memdevs, memd, next);
    DBG("new instance initialized");

    return 0;
}

static void
ptnetmap_memdev_uninit(PCIDevice *dev)
{
    PTNetmapMemDevState *memd = PTNETMAP_MEMDEV(dev);
	
    if (memd->mem_ptr)
	    munmap(memd->mem_ptr,memd->mem_size);
    if (memd->fd)
	    close(memd->fd);
    memd->mem_ptr=NULL;
    memd->fd=-1;
    QTAILQ_REMOVE(&ptn_memdevs, memd, next);
    DBG("new instance uninitialized");
}

 /*
  * find memd through memid
  */
#if 0
static struct PTNetmapMemDevState *
ptnetmap_memdev_find(uint16_t memid)
{
    //PTNetmapMemDevState *memd;


    return NULL;
}
#endif
/* Function exported to be used by the netmap backend. */
int
dockervisor_smem_create(uint64_t size)
{
    PTNetmapMemDevState *memd;
    PCIDevice *dev;
    PCIBus *bus;
    int fd,i;
    void *mem_ptr;              /* host virtual pointer to netmap memory */

    DBG("creating new instance");
#if 0
    if (ptnetmap_memdev_find(pi->memid)) {
        DBG("memdev instance for mem-id %d already exists", pi->memid);
        return 0;
    }
#endif

    bus = pci_find_primary_bus();

    if (bus == NULL) {
        DBG("unable to find PCI BUS");
        return -1;
    }
    /* Create a new PCI device belonging to the ptnetmap class. */
    dev = pci_create(bus, -1, TYPE_PTNETMAP_MEMDEV);

    /* Set shared memory parameters for the new ptnetmap memdev instance. */
#if 1
    memd = PTNETMAP_MEMDEV(dev);
    if ((fd = open("/dev/dockervisor_mm", O_RDWR| O_SYNC))<0){
	    perror("open");
	    return -1;
    }
    mem_ptr = mmap(0, size, PROT_READ| PROT_WRITE, MAP_SHARED , fd, 0);
    if (mem_ptr == MAP_FAILED){
    	close(fd);
	return -1;
    }
    fprintf(stdout,"[Dd] in dovkervisor_smem_create,mem_ptr:%p\n",mem_ptr);
    memd->fd = fd;
    for (i=0;i<size/4;i++){
	    ((int *)mem_ptr)[i]=0x0;
    }
    memd->mem_ptr = mem_ptr;
    memd->mem_size = size;
//    (memd->csb)[0] = 'd';
  //  memd->csb = g_malloc0(4096);
    *(memd->csb) = 'd';

#endif

    /* Initialize the new device. */
    qdev_init_nofail(&dev->qdev);

    DBG("created new instance");

    return 0;
}

static void
qdev_ptnetmap_memdev_reset(DeviceState *dev)
{
}

static void
ptnetmap_memdev_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    PCIDeviceClass *k = PCI_DEVICE_CLASS(klass);

    k->init = ptnetmap_memdev_init;
    k->exit = ptnetmap_memdev_uninit;
    k->vendor_id = DOCKERVISOR_PCI_VENDOR_ID;
    k->device_id = DOCKERVISOR_PCI_DEVICE_ID;
    k->revision = 0x00;
    k->class_id = PCI_CLASS_MEMORY_RAM;
    dc->desc = "ptnetmap memory device";
    set_bit(DEVICE_CATEGORY_MISC, dc->categories);
    dc->reset = qdev_ptnetmap_memdev_reset;
}

static const TypeInfo ptnetmap_memdev_info = {
    .name          = TYPE_PTNETMAP_MEMDEV,
    .parent        = TYPE_PCI_DEVICE,
    .instance_size = sizeof(PTNetmapMemDevState),
    .class_init    = ptnetmap_memdev_class_init,
};

static void ptnetmap_memdev_register_types(void)
{
    type_register_static(&ptnetmap_memdev_info);
}

type_init(ptnetmap_memdev_register_types)
