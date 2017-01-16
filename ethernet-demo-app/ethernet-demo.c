/*
 * Copyright 2016, Data61
 * Commonwealth Scientific and Industrial Research Organisation (CSIRO)
 * ABN 41 687 119 230.
 *
 * This software may be distributed and modified according to the terms of
 * the BSD 2-Clause license. Note that NO WARRANTY is provided.
 * See "LICENSE_BSD2.txt" for details.
 *
 * @TAG(D61_BSD)
 */

#include <camkes.h>
#include <camkes/dma.h>

/* remove the camkes ERR_IF definition to not overlap with lwip */
#undef ERR_IF

#include <stdlib.h>
#include <stdio.h>

#include <platsupport/io.h>
#include <vka/vka.h>
#include <simple/simple.h>
#include <simple/simple_helpers.h>
#include <allocman/allocman.h>
#include <allocman/bootstrap.h>
#include <allocman/vka.h>
#include <sel4utils/vspace.h>
#include <ethdrivers/intel.h>
#include <ethdrivers/raw.h>
#include <ethdrivers/lwip.h>
#include <sel4utils/iommu_dma.h>
#include <sel4platsupport/arch/io.h>
#include <platsupport/timer.h>
#include <platsupport/plat/hpet.h>
#include <platsupport/arch/tsc.h>

#include <lwip/init.h>
#include <lwip/udp.h>
#include <lwip/dhcp.h>
#include <lwip/timers.h>
#include <netif/etharp.h>

#define RX_BUFS 256
#define BUF_SIZE 2048

static simple_t camkes_simple;
static vka_t vka;
static vspace_t vspace;
static allocman_t *allocman;
static char allocator_mempool[0x4000];
static lwip_iface_t _lwip;
static seL4_CPtr (*original_vspace_get_cap)(vspace_t*, void*);
static sel4utils_alloc_data_t vspace_data;
static pstimer_t *hpet_timer = NULL;
static pstimer_t *tsc_timer = NULL;

void camkes_make_simple(simple_t *simple);

static void init_env(void) {
    int UNUSED error;

    camkes_make_simple(&camkes_simple);

    /* Initialize allocator */
    allocman = bootstrap_use_current_1level(
            simple_get_cnode(&camkes_simple),
            simple_get_cnode_size_bits(&camkes_simple),
            simple_last_valid_cap(&camkes_simple) + 1,
            BIT(simple_get_cnode_size_bits(&camkes_simple)),
            sizeof(allocator_mempool), allocator_mempool
    );
    assert(allocman);
    error = allocman_add_simple_untypeds(allocman, &camkes_simple);
    assert(!error);
    allocman_make_vka(&vka, allocman);

    void *existing_frames[] = {
        NULL
    };

    error = sel4utils_bootstrap_vspace(&vspace, &vspace_data,
            simple_get_init_cap(&camkes_simple, seL4_CapInitThreadPD), &vka, NULL, NULL, existing_frames);
    assert(!error);
}

/* Returns the cap to the frame mapped to vaddr, assuming
 * vaddr points inside our dma pool. */
static seL4_CPtr get_dma_frame_cap(vspace_t *vspace, void *vaddr) {\
    seL4_CPtr cap = camkes_dma_get_cptr(vaddr);
    if (cap == seL4_CapNull) {
        return original_vspace_get_cap(vspace, vaddr);
    }
    return cap;
}

/* Allocate a dma buffer backed by the component's dma pool */
static void* camkes_iommu_dma_alloc(void *cookie, size_t size,
        int align, int cached, ps_mem_flags_t flags) {

    // allocate buffer from the dma pool
    void* vaddr = camkes_dma_alloc(size, align);
    if (vaddr == NULL) {
        return NULL;
    }
#ifdef CONFIG_IOMMU
    int error = sel4utils_iommu_dma_alloc_iospace(cookie, vaddr, size);
    if (error) {
        camkes_dma_free(vaddr, size);
        return NULL;
    }
#endif
    return vaddr;
}

#ifndef CONFIG_IOMMU
static uintptr_t camkes_dma_pin(void *cookie, void *addr, size_t size) {
    return camkes_dma_get_paddr(addr);
}
#endif

void eth_irq_handle(void) {
    int UNUSED error;
    error = lock_lock();
    ethif_lwip_handle_irq(&_lwip, 0);
    error = eth_irq_acknowledge();
    error = lock_unlock();
}

void hpet_irq_handle(void) {
    int UNUSED error;
    error = lock_lock();
    sys_check_timeouts();
    timer_handle_irq(hpet_timer, 0);
    error = hpet_irq_acknowledge();
    error = lock_unlock();
}

static lwip_iface_t *init_eth(int iospace_id, int bus, int dev, int fun) {
    int UNUSED error;
    int pci_bdf = bus * 256 + dev * 8 + fun;
    ps_io_ops_t ioops;

#ifdef CONFIG_IOMMU
    cspacepath_t iospace;
    error = vka_cspace_alloc_path(&vka, &iospace);
    error = simple_get_iospace(&camkes_simple, iospace_id, pci_bdf, &iospace);
    assert(!error);
#endif

    original_vspace_get_cap = vspace.get_cap;
    vspace.get_cap = get_dma_frame_cap;

#ifdef CONFIG_IOMMU
    error = sel4utils_make_iommu_dma_alloc(&vka, &vspace, &ioops.dma_manager, 1, &iospace.capPtr);
#else
    error = sel4utils_new_page_dma_alloc(&vka, &vspace, &ioops.dma_manager);
#endif
    assert(!error);
    ioops.dma_manager.dma_alloc_fn = camkes_iommu_dma_alloc;

#ifndef CONFIG_IOMMU
    ioops.dma_manager.dma_pin_fn = camkes_dma_pin;
#endif

    error = sel4platsupport_get_io_port_ops(&ioops.io_port_ops, &camkes_simple);
    assert(!error);

    ethif_intel_config_t eth_config = (ethif_intel_config_t) {
        .bar0 = (void*)eth_mmio,
    };

    printf("Initializing ethernet driver\n");
    lwip_iface_t *lwip = ethif_new_lwip_driver_no_malloc(ioops, NULL, ethif_e82574_init, &eth_config, &_lwip);
    assert(lwip);

    error = eth_irq_acknowledge();
    assert(!error);

    return lwip;
}

void udp_recv_callback(void *arg, struct udp_pcb *pcb, struct pbuf *p, ip_addr_t *ip, u16_t port) {
    printf("Received udp packet of %d bytes from %s:%d\n", p->len, ipaddr_ntoa(ip), port);
    char *data = malloc(p->len + 1);
    strncpy(data, p->payload, p->len);
    data[p->len] = '\0';

    printf("Data: %s\n", data);
    free(data);
    /* send the data back */
    udp_sendto(pcb, p, ip, port);
    pbuf_free(p);
}

void netif_link_callback(struct netif *netif) {
    if (netif_is_link_up(netif)) {
        int err;
        printf("Link is up\n");
        printf("IPADDR is %s\n", ipaddr_ntoa(&netif->ip_addr));
        struct udp_pcb *udp_conn = udp_new();
        ZF_LOGF_IF(!udp_conn, "Failed to create udp connection");

        err = udp_bind(udp_conn, IP_ADDR_ANY, 7);
        ZF_LOGF_IF(err != ERR_OK, "Failed to bind port 7");
        udp_recv(udp_conn, udp_recv_callback, NULL);
    }
}

static struct netif *init_interface(lwip_iface_t *lwip) {
    struct netif *netif = malloc(sizeof(*netif));
    err_t err;

    lwip->netif = netif_add(netif, NULL, NULL, NULL, lwip, ethif_get_ethif_init(lwip), ethernet_input);

    assert(lwip->netif != NULL);
    netif_set_default(lwip->netif);
    netif_set_status_callback(lwip->netif, netif_link_callback);
    err = dhcp_start(lwip->netif);
    ZF_LOGF_IF(err != ERR_OK, "Failed to start dhcp");

    return netif;
}

void init_timers(void) {
    hpet_config_t config = (hpet_config_t){hpet_mmio, 1 + IRQ_OFFSET, 0};
    hpet_timer = hpet_get_timer(&config);
    ZF_LOGF_IF(!hpet_timer, "Failed to get HPET timer");
    tsc_timer = tsc_get_timer(hpet_timer);
    ZF_LOGF_IF(!tsc_timer, "Failed to get TSC timer");
    /* configure the hpet timer for periodic timeout */
    timer_start(hpet_timer);
    timer_periodic(hpet_timer, NS_IN_MS * 100);
}

u32_t sys_now(void) {
    return timer_get_time(tsc_timer) / NS_IN_MS;
}

void pre_init() {
    int bus, dev, fun;
    init_env();

    printf("Initializing timers\n");
    init_timers();
    sscanf(pci_bdf, "%x:%x.%d", &bus, &dev, &fun);
    lwip_iface_t *lwip = init_eth(iospace_id, bus, dev, fun);
    printf("Initializing lwip\n");
    lwip_init();

    printf("Creating interface\n");
    struct UNUSED netif *netif = init_interface(lwip);
    assert(netif);
    printf("Waiting for DHCP\n");
}
