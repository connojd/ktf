/*
 * Copyright © 2022 Amazon.com, Inc. or its affiliates.
 * All Rights Reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#include <list.h>
#include <mm/slab.h>
#include <pci.h>
#include <pci_cfg.h>
#include <string.h>

static list_head_t pci_list = LIST_INIT(pci_list);

static inline uint8_t pci_dev_hdr_type(pcidev_t *dev) { return dev->hdr & PCI_HDR_TYPE; }

static inline bool pci_dev_is_multifunc(pcidev_t *dev) {
    return ((dev->hdr & PCI_HDR_MULTIFUNC) != 0);
}

static inline void pci_disable_io(uint8_t bus, uint8_t dev, uint8_t func) {
    uint32_t cfg_val = pci_cfg_read(bus, dev, func, PCI_REG_COMMAND);

    cfg_val &= ~(PCI_COMMAND_PIO | PCI_COMMAND_MMIO);
    pci_cfg_write(bus, dev, func, PCI_REG_COMMAND, cfg_val);
}

static inline void pci_enable_io(uint8_t bus, uint8_t dev, uint8_t func) {
    uint32_t cfg_val = pci_cfg_read(bus, dev, func, PCI_REG_COMMAND);

    cfg_val |= (PCI_COMMAND_PIO | PCI_COMMAND_MMIO);
    pci_cfg_write(bus, dev, func, PCI_REG_COMMAND, cfg_val);
}

/* Debug helper */
__attribute__((unused)) static void dump_pci_cfg(uint8_t bus, uint8_t dev, uint8_t func) {
    printk("pci: config space for %02x:%02x.%1x\n", bus, dev, func);

    const unsigned nr_lines = 16;
    const unsigned nr_regs = 4;

    for (unsigned line = 0; line < nr_lines; line++) {
        for (unsigned i = 0; i < nr_regs; i++) {
            const uint32_t reg = line * 4 + i;
            const uint32_t cfg_val = pci_cfg_read(bus, dev, func, reg);

            printk(" %02x%02x%02x%02x", (cfg_val & 0xFF), (cfg_val & 0xFF00) >> 8,
                   (cfg_val & 0xFF0000) >> 16, (cfg_val & 0xFF000000) >> 24);
        }

        printk("\n");
    }
}

static uint64_t size_pio_bar(uint8_t bus, uint8_t dev, uint8_t func, uint8_t reg) {
    uint32_t cfg_val, orig_val;
    uint64_t size;

    /* Save original value and write 1's to BAR */
    orig_val = pci_cfg_read(bus, dev, func, reg);
    pci_cfg_write(bus, dev, func, reg, ~0U);

    /* Read encoded size, mask off rsvd/type bits */
    cfg_val = pci_cfg_read(bus, dev, func, reg);
    cfg_val &= PCI_BAR_PIO_BASE_MASK;

    /* Calculate size */
    size = ~cfg_val + 1;

    /* Restore original BAR value and enable PIO/MMIO */
    pci_cfg_write(bus, dev, func, reg, orig_val);

    return size;
}

static uint64_t size_32bit_mmio_bar(uint8_t bus, uint8_t dev, uint8_t func, uint8_t reg) {
    uint32_t cfg_val, orig_val;
    uint64_t size;

    /* Save original value and write 1's to BAR */
    orig_val = pci_cfg_read(bus, dev, func, reg);
    pci_cfg_write(bus, dev, func, reg, ~0U);

    /* Read encoded size, mask off rsvd/type bits */
    cfg_val = pci_cfg_read(bus, dev, func, reg);
    cfg_val &= PCI_BAR_MMIO_BASE_MASK;

    /* Calculate size */
    size = ~cfg_val + 1;

    /* Restore original BAR value and enable PIO/MMIO */
    pci_cfg_write(bus, dev, func, reg, orig_val);

    return size;
}

static uint64_t size_64bit_mmio_bar(uint8_t bus, uint8_t dev, uint8_t func, uint8_t reg) {
    uint32_t cfg_val_lo, cfg_val_hi, orig_val_lo, orig_val_hi;
    uint64_t size;

    /* Save original value and write 1's to BAR */
    orig_val_lo = pci_cfg_read(bus, dev, func, reg);
    orig_val_hi = pci_cfg_read(bus, dev, func, reg + 1);

    pci_cfg_write(bus, dev, func, reg, ~0U);
    pci_cfg_write(bus, dev, func, reg + 1, ~0U);

    /* Read encoded size, mask off rsvd/type bits */
    cfg_val_lo = pci_cfg_read(bus, dev, func, reg);
    cfg_val_hi = pci_cfg_read(bus, dev, func, reg + 1);
    cfg_val_lo &= PCI_BAR_MMIO_BASE_MASK;

    /* Calculate size */
    size = ~(((uint64_t) cfg_val_hi << 32) | cfg_val_lo) + 1;

    /* Restore original BAR value and enable PIO/MMIO */
    pci_cfg_write(bus, dev, func, reg, orig_val_lo);
    pci_cfg_write(bus, dev, func, reg + 1, orig_val_hi);

    return size;
}

static void pci_dev_parse_bars(pcidev_t *dev) {
    const uint32_t first_bar = 4;
    const uint32_t last_bar = ((dev->hdr & PCI_HDR_TYPE) == PCI_HDR_TYPE_NORMAL) ? 9 : 5;

    list_init(&dev->bar_list);

    /* Disable PIO/MMIO for subsequent BAR sizing */
    pci_disable_io(dev->bus, dev->dev, dev->func);

    for (uint32_t i = first_bar; i <= last_bar; i++) {
        pcibar_t *bar;
        uint32_t cfg_val = pci_cfg_read(dev->bus, dev->dev, dev->func, i);

        if (0 == cfg_val)
            continue;

        bar = kzalloc(sizeof(*bar));
        BUG_ON(!bar);

        if ((cfg_val & PCI_BAR_TYPE_MASK) == PCI_BAR_TYPE_PIO) {
            bar->type = PCI_BAR_TYPE_PIO;
            bar->base = cfg_val & PCI_BAR_PIO_BASE_MASK;
            bar->size = size_pio_bar(dev->bus, dev->dev, dev->func, i);
        }
        else {
            bar->type = PCI_BAR_TYPE_MMIO;
            bar->base = cfg_val & PCI_BAR_MMIO_BASE_MASK;

            if ((cfg_val & PCI_BAR_MMIO_BIT_MASK) == PCI_BAR_MMIO_64BIT) {
                cfg_val = pci_cfg_read(dev->bus, dev->dev, dev->func, i + 1);

                bar->base |= ((uint64_t) cfg_val << 32);
                bar->size = size_64bit_mmio_bar(dev->bus, dev->dev, dev->func, i);
                i++;
            }
            else {
                bar->size = size_32bit_mmio_bar(dev->bus, dev->dev, dev->func, i);
            }

            bar->prefetch = !!(cfg_val & PCI_BAR_MMIO_PREFETCH);
        }

        list_add_tail(&bar->list, &dev->bar_list);
        dev->nr_bars++;
    }

    pci_enable_io(dev->bus, dev->dev, dev->func);
}

static pcidev_t *probe_pci_dev(uint8_t bus, uint8_t dev, uint8_t func,
                               uint32_t device_vendor) {
    uint32_t cfg_val;
    pcidev_t *new_dev = kzalloc(sizeof(*new_dev));

    if (NULL == new_dev)
        return NULL;

    new_dev->segment = 0;
    new_dev->bus = bus;
    new_dev->dev = dev;
    new_dev->func = func;

    new_dev->vendor_id = PCI_VENDOR(device_vendor);
    new_dev->device_id = PCI_DEVICE(device_vendor);

    cfg_val = pci_cfg_read(bus, dev, func, PCI_REG_COMMAND);
    new_dev->command = PCI_COMMAND(cfg_val);
    new_dev->status = PCI_STATUS(cfg_val);

    cfg_val = pci_cfg_read(bus, dev, func, PCI_REG_CLASS);
    new_dev->class = PCI_CLASS(cfg_val);
    new_dev->subclass = PCI_SUBCLASS(cfg_val);

    cfg_val = pci_cfg_read(bus, dev, func, PCI_REG_HDR);
    new_dev->hdr = PCI_HDR(cfg_val);

    if (new_dev->status & PCI_STATUS_CAP_LIST)
        new_dev->cap_ptr = pci_cfg_read8(bus, dev, func, PCI_REG_CAP_PTR);

    pci_dev_parse_bars(new_dev);

    return new_dev;
}

static void probe_pci_bus(uint8_t bus, uint8_t start_dev, pcidev_t *bridge) {
    for (uint8_t dev = start_dev; dev < PCI_NR_DEV; dev++) {
        for (uint8_t func = 0; func < PCI_NR_FUNC; func++) {
            pcidev_t *new_dev;
            uint32_t cfg_val;

            cfg_val = pci_cfg_read(bus, dev, func, PCI_REG_VENDOR);
            if (!PCI_DEV_EXISTS(cfg_val)) {
                continue;
            }

            new_dev = probe_pci_dev(bus, dev, func, cfg_val);
            BUG_ON(!new_dev);

            new_dev->bridge = bridge;
            snprintf(&new_dev->bdf_str[0], sizeof(new_dev->bdf_str), "%02x:%02x.%1x", bus,
                     dev, func);

            list_add_tail(&new_dev->list, &pci_list);

            if (pci_dev_hdr_type(new_dev) == PCI_HDR_TYPE_PCI_BRIDGE) {
                uint8_t secondary, subordinate;

                cfg_val = pci_cfg_read(bus, dev, func, PCI_REG_BUS);
                secondary = PCI_SEC_BUS(cfg_val);
                subordinate = PCI_SUB_BUS(cfg_val);

                for (uint8_t b = secondary; b <= subordinate; b++) {
                    probe_pci_bus(b, 0, new_dev);
                }
            }

            if (!pci_dev_is_multifunc(new_dev))
                break;
        }
    }
}

static void probe_pci(void) {
    pcidev_t *host_bridge;
    uint32_t cfg_val;
    uint32_t class_reg;
    const uint8_t bus = 0;
    const uint8_t dev = 0;
    const uint8_t func = 0;
    const uint32_t vendor_reg = pci_cfg_read(bus, dev, func, PCI_REG_VENDOR);

    if (!PCI_DEV_EXISTS(vendor_reg)) {
        printk("pci: non-existent host bridge @ 00.00.0\n");
        return;
    }

    class_reg = pci_cfg_read(bus, dev, func, PCI_REG_CLASS);
    if (PCI_CLASS(class_reg) != PCI_CLASS_BRIDGE &&
        PCI_SUBCLASS(class_reg) != PCI_SUBCLASS_HOST_BRIDGE) {
        printk("pci: expected host bridge class code @ 00.00.0\n");
        return;
    }

    /* Found the host bridge, initialize it */
    host_bridge = kzalloc(sizeof(*host_bridge));
    BUG_ON(!host_bridge);

    host_bridge->segment = 0;
    host_bridge->vendor_id = PCI_VENDOR(vendor_reg);
    host_bridge->device_id = PCI_DEVICE(vendor_reg);

    cfg_val = pci_cfg_read(bus, dev, func, PCI_REG_COMMAND);
    host_bridge->command = PCI_COMMAND(cfg_val);
    host_bridge->status = PCI_STATUS(cfg_val);

    host_bridge->class = PCI_CLASS_BRIDGE;
    host_bridge->subclass = PCI_SUBCLASS_HOST_BRIDGE;
    host_bridge->bridge = NULL;

    strncpy(&host_bridge->bdf_str[0], "00:00.0", sizeof(host_bridge->bdf_str));
    list_add_tail(&host_bridge->list, &pci_list);

    /* Probe the rest of bus 0 */
    probe_pci_bus(0, 1, host_bridge);
}

static void pci_dev_print_bars(pcidev_t *dev) {
    pcibar_t *bar;

    list_for_each_entry (bar, &dev->bar_list, list) {
        printk("pci: %s: %s BAR @ [0x%016x - 0x%016x] %s\n", &dev->bdf_str[0],
               bar->type == PCI_BAR_TYPE_PIO ? "PIO " : "MMIO", bar->base,
               bar->base + bar->size - 1, bar->prefetch ? "prefetch" : "");
    }
}

void init_pci(void) {
    pcidev_t *dev;

    printk("Initializing PCI\n");
    probe_pci();

    list_for_each_entry (dev, &pci_list, list) {
        printk("pci: found device at %s\n", &dev->bdf_str[0]);

        if (dev->nr_bars > 0)
            pci_dev_print_bars(dev);
    }
}
