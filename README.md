
# IntelMausi

[![Build Status](https://github.com/acidanthera/IntelMausi/actions/workflows/main.yml/badge.svg?branch=master)](https://github.com/acidanthera/IntelMausi/actions) [![Scan Status](https://scan.coverity.com/projects/18406/badge.svg?flat=1)](https://scan.coverity.com/projects/18406)

IntelMausi is an advanced Intel onboard LAN driver for macOS, created by [Laura Müller](https://github.com/Mieze). For more details, please refer to the [original repository](https://github.com/Mieze/IntelMausiEthernet). This version includes modifications tailored for the [Acidanthera](https://github.com/acidanthera) project and kernel debugging support initially contributed by [aerror2](https://github.com/aerror2) in the [IntelMausiEthernetWithKernelDebugger](https://github.com/aerror2/IntelMausiEthernetWithKernelDebugger) repository. Please use the original version when uncertain, as no support or troubleshooting is provided here.

### Wake on LAN
Wake on LAN functionality should work out-of-the-box. For misconfigured hardware, you can force-enable it by injecting the `mausi-force-wol` device property (with any value, recommended) or by using the `-mausiwol` boot argument (for testing purposes).

## Overview
A few days before Christmas, I embarked on a new project to develop a driver for recent Intel onboard LAN controllers. My goal was not to completely replace hnak's AppleIntelE1000e.kext but to deliver superior performance and stability on modern hardware, which is why I have dropped support for some older NICs. Currently, the driver supports:

### Supported Controllers

| **Series**   | **Controllers**                                             |
|--------------|-------------------------------------------------------------|
| **5 Series** | 82578LM, 82578LC, 82578DM, 82578DC                          |
| **6 Series** | 82579LM, 82579V                                             |
| **7 Series** | 82579LM, 82579V                                             |
| **8 Series** | I217LM, I217V, I218LM, I218V, I218LM2, I218V2, I218LM3      |
| **9 Series** | I217LM, I217V, I218LM, I218V, I218LM2, I218V2, I218LM3      |
| **100 Series** | I219V, I219LM, I219V2, I219LM2, I219LM3                   |
| **200 Series** | I219LM, I219V                                             |
| **300 Series** | I219LM, I219V                                             |

### Key Features
- **Multisegment Packets**: Reduces unnecessary copy operations for packet assembly during transmission.
- **No-copy Receive and Transmit**: Only small packets are copied upon reception for efficiency.
- **Checksum Offload**: Supports TCP, UDP, and IPv4 checksum offload (receive and transmit).
- **IPv6 Checksum Offload**: Supports TCP/IPv6 and UDP/IPv6 checksum offload.
- **TCP Segmentation Offload (TSO)**: Utilizes TSO with IPv4 and IPv6 to reduce CPU load during data transmission (disabled due to hardware bugs).
- **Optimization for Mavericks and Newer**: Fully optimized for 64-bit architecture.
- **Energy Efficient Ethernet (EEE)**: Supports EEE.
- **VLAN Support**: Implemented but untested.

### Licensing
The driver is published under GPLv2.

## Support
For further questions, support, or to submit problem reports, please visit the driver's thread on [insanelymac.com](https://www.insanelymac.com/forum/topic/304235-intelmausiethernetkext-for-intel-onboard-lan/). As of now, support requests here on GitHub will be ignored.
