# camkes-apps-ethernet-demo-x86--devel

A simple ethernet-demo camkes application. The ethernet-demo application is currently written to
require an IOMMU for DMA translation, hence it doesn't run in QEMU and must be run on real hardware with Vt-x support.
