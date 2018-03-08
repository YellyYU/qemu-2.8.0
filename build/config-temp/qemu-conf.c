/*
 * If we have stable libs the we don't want the libxc compat
 * layers, regardless of what CFLAGS we may have been given.
 *
 * Also, check if xengnttab_grant_copy_segment_t is defined and
 * grant copy operation is implemented.
 */
#undef XC_WANT_COMPAT_EVTCHN_API
#undef XC_WANT_COMPAT_GNTTAB_API
#undef XC_WANT_COMPAT_MAP_FOREIGN_API
#include <xenctrl.h>
#include <xenstore.h>
#include <xenevtchn.h>
#include <xengnttab.h>
#include <xenforeignmemory.h>
#include <stdint.h>
#include <xen/hvm/hvm_info_table.h>
#if !defined(HVM_MAX_VCPUS)
# error HVM_MAX_VCPUS not defined
#endif
int main(void) {
  xc_interface *xc = NULL;
  xenforeignmemory_handle *xfmem;
  xenevtchn_handle *xe;
  xengnttab_handle *xg;
  xen_domain_handle_t handle;
  xengnttab_grant_copy_segment_t* seg = NULL;

  xs_daemon_open();

  xc = xc_interface_open(0, 0, 0);
  xc_hvm_set_mem_type(0, 0, HVMMEM_ram_ro, 0, 0);
  xc_domain_add_to_physmap(0, 0, XENMAPSPACE_gmfn, 0, 0);
  xc_hvm_inject_msi(xc, 0, 0xf0000000, 0x00000000);
  xc_hvm_create_ioreq_server(xc, 0, HVM_IOREQSRV_BUFIOREQ_ATOMIC, NULL);
  xc_domain_create(xc, 0, handle, 0, NULL, NULL);

  xfmem = xenforeignmemory_open(0, 0);
  xenforeignmemory_map(xfmem, 0, 0, 0, 0, 0);

  xe = xenevtchn_open(0, 0);
  xenevtchn_fd(xe);

  xg = xengnttab_open(0, 0);
  xengnttab_grant_copy(xg, 0, seg);

  return 0;
}
