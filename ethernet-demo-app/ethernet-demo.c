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
#include <stdlib.h>

#include <camkes/dma.h>
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

#include <lwip/init.h>
#include <lwip/udp.h>
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

static int num_rx_bufs = 0;
static void *rx_bufs[RX_BUFS];

void camkes_make_simple(simple_t *simple);

static void init_env(void) {
    int error;

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
    ethif_lwip_handle_irq(&_lwip, 0);
    int error = eth_irq_acknowledge();
    assert(!error);
}

static lwip_iface_t *init_eth(int iospace_id, int bus, int dev, int fun) {
    int error;
    int pci_bdf = bus * 256 + dev * 8 + fun;
    ps_io_ops_t ioops;

    assert(!error);
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

static struct netif *init_interface(lwip_iface_t *lwip, const char *ip_str, const char *nm_str, const char *gw_str) {
    struct ip_addr ip, nm, gw;

    int err = 0;
    err |= !ipaddr_aton(ip_str, &ip);
    err |= !ipaddr_aton(nm_str, &nm);
    err |= !ipaddr_aton(gw_str, &gw);

    if (err) {
        return NULL;
    }

    struct netif *netif = malloc(sizeof(*netif));

    lwip->netif = netif_add(netif, &ip, &nm, &gw, lwip, ethif_get_ethif_init(lwip), ethernet_input);

    assert(lwip->netif != NULL);
    netif_set_up(lwip->netif);
    netif_set_default(lwip->netif);

    return netif;
}

static void test_udp(const char *dest_ip_str, struct udp_pcb *udp_conn, int port, const char *msg) {
    err_t err;
    struct ip_addr dst;
    ipaddr_aton(dest_ip_str, &dst);

    int len = strlen(msg);
    struct pbuf *pbuf = pbuf_alloc(PBUF_TRANSPORT, len, PBUF_RAM);
    assert(pbuf);

    printf("Attempting to send %d bytes to %s\n", len, dest_ip_str);

    memcpy(pbuf->payload, msg, len);
    err = udp_sendto(udp_conn, pbuf, &dst, port);
    assert(err == ERR_OK);

    pbuf_free(pbuf);
}

void udp_recv_callback(void *arg, struct udp_pcb *pcb, struct pbuf *p, ip_addr_t *ip, u16_t port) {
    printf("Received udp packet of %d bytes from %s:%d\n", p->len, ipaddr_ntoa(ip), port);
    char *data = malloc(p->len + 1);
    strncpy(data, p->payload, p->len);
    data[p->len] = '\0';

    printf("Data: %s\n", data);
    free(data);
    pbuf_free(p);
}

int run() {

    init_env();
    lwip_iface_t *lwip = init_eth(0x12, 6, 0, 0);

    printf("Initializing lwip\n");
    lwip_init();

    printf("Creating interface\n");
    struct netif *netif = init_interface(lwip, "192.168.0.2", "255.255.255.0", "192.168.0.1");
    assert(netif);

    struct udp_pcb *udp_conn = udp_new();
    assert(udp_conn);

    udp_recv(udp_conn, udp_recv_callback, NULL);

    char buf[128];

    unsigned int count = 0;
    for (int j = 0; j < 10; j++) {
        for (volatile int i = 0; i < 1000000000; i++);
        snprintf(buf, 128, "[%d] Hello, World!\n", count);
        test_udp("192.168.0.1", udp_conn, 1234, buf);
        count++;
    }

    udp_remove(udp_conn);

    return 0;
}
