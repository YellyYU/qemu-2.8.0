#ifndef DOCKERVISOR_H
#define DOCKERVISOR_H

#include <net/if.h>
#include "net/netmap.h"
#include "net/net.h"
#include "exec/memory.h"
#include "net/netmap_virt.h" /* from netmap sources */

#define DOCKERVISOR_IO_PCI_BAR		0
#define DOCKERVISOR_MEM_PCI_BAR		1
#define DOCKERVISOR_MSIX_PCI_BAR	2
#define DOCKERVISOR_PCI_VENDOR_ID	0x1b36
#define DOCKERVISOR_PCI_DEVICE_ID	0x000e


#define DOCKERVISOR_MDEV_IO_MEMSIZE_LO	0
#define DOCKERVISOR_MDEV_IO_MEMSIZE_HI	4
#define DOCKERVISOR_MDEV_IO_END		100



int dockervisor_smem_create(uint64_t size);
int dockervisor_smem_create2(void);

#undef PTNET_DEBUG /* enable to add debug logs for ptnetmap netif and memdev */

#endif /* PTNETMAP_H */
