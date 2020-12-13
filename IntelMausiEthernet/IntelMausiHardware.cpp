/* IntelMausiHardware.cpp -- IntelMausi hardware specific routines.
 *
 * Copyright (c) 2014 Laura Müller <laura-mueller@uni-duesseldorf.de>
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 2 of the License, or (at your option)
 * any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * Driver for Intel PCIe gigabit ethernet controllers.
 *
 * This driver is based on Intel's E1000e driver for Linux.
 */


#include "IntelMausiEthernet.h"

#pragma mark --- hardware initialization methods ---

bool IntelMausi::initPCIConfigSpace(IOPCIDevice *provider)
{
    UInt32 lat1, lat2;
    bool result = false;
    
    /* Get vendor and device info. */
    pciDeviceData.vendor = provider->extendedConfigRead16(kIOPCIConfigVendorID);
    pciDeviceData.device = provider->extendedConfigRead16(kIOPCIConfigDeviceID);
    pciDeviceData.subsystem_vendor = provider->extendedConfigRead16(kIOPCIConfigSubSystemVendorID);
    pciDeviceData.subsystem_device = provider->extendedConfigRead16(kIOPCIConfigSubSystemID);
    pciDeviceData.revision = provider->extendedConfigRead8(kIOPCIConfigRevisionID);
    
    /* Identify the chipset. */
    if (!intelIdentifyChip())
        goto done;
    
    if (chipType >= board_pch_lpt) {
        pciDeviceData.maxSnoop = provider->extendedConfigRead16(E1000_PCI_LTR_CAP_LPT);
        pciDeviceData.maxNoSnoop = provider->extendedConfigRead16(E1000_PCI_LTR_CAP_LPT + 2);
        
        lat1 = (pciDeviceData.maxSnoop & 0x3ff) << (((pciDeviceData.maxSnoop >> 10) & 0x7) * 5);
        lat2 = (pciDeviceData.maxNoSnoop & 0x3ff) << (((pciDeviceData.maxNoSnoop >> 10) & 0x7) * 5);
        maxLatency = (lat1 > lat2) ? lat1 : lat2;
        
        if (maxLatency > kMaxDmaLatency)
            maxLatency = kMaxDmaLatency;
        
        DebugLog("[IntelMausi]: maxSnoop: 0x%04x (%uns), maxNoSnoop: 0x%04x (%uns).\n", pciDeviceData.maxSnoop, lat1, pciDeviceData.maxNoSnoop, lat2);
    }
    /* Get the bus information. */
    adapterData.hw.bus.func = pciDevice->getFunctionNumber();
    adapterData.hw.bus.width = e1000_bus_width_pcie_x1;

    /* Enable the device. */
    intelEnablePCIDevice(provider);
    
    baseMap = provider->mapDeviceMemoryWithRegister(kIOPCIConfigBaseAddress0, kIOMapInhibitCache);
    
    if (!baseMap) {
        IOLog("[IntelMausi]: region #0 not an MMIO resource, aborting.\n");
        goto done;
    }
    baseAddr = reinterpret_cast<volatile void *>(baseMap->getVirtualAddress());
    adapterData.hw.hw_addr = (u8 __iomem *)baseAddr;
    adapterData.hw.flash_address = NULL;
    
    if ((adapterData.flags & FLAG_HAS_FLASH) && (adapterData.hw.mac.type < e1000_pch_spt)) {
        flashMap = provider->mapDeviceMemoryWithRegister(kIOPCIConfigBaseAddress1, kIOMapInhibitCache);
        
        if (!flashMap) {
            IOLog("[IntelMausi]: region #1 not an MMIO resource, aborting.\n");
            goto error;
        }
        flashAddr = reinterpret_cast<volatile void *>(flashMap->getVirtualAddress());
        adapterData.hw.flash_address = (u8 __iomem *)flashAddr;
    }
    result = true;
    
done:
    return result;
    
error:
    RELEASE(baseMap);
    baseMap = NULL;
    adapterData.hw.hw_addr = NULL;
    goto done;
}

void IntelMausi::initPCIPowerManagment(IOPCIDevice *provider, const struct e1000_info *ei)
{
#ifdef DEBUG
    UInt32 pcieLinkCap;
#endif
    UInt16 pcieLinkCtl;
    UInt16 aspmDisable;
    UInt16 pmCap;
    UInt8 pmCapOffset;
    
    /* Setup power management. */
    if (provider->findPCICapability(kIOPCIPowerManagementCapability, &pmCapOffset)) {
        pmCap = provider->extendedConfigRead16(pmCapOffset + kIOPCIPMCapability);
        DebugLog("[IntelMausi]: PCI power management capabilities: 0x%x.\n", pmCap);
        
        if (pmCap & (kPCIPMCPMESupportFromD3Cold | kPCIPMCPMESupportFromD3Hot)) {
            wolCapable = true;
            DebugLog("[IntelMausi]: PME# from D3 (cold/hot) supported.\n");
        }
        pciPMCtrlOffset = pmCapOffset + kIOPCIPMControl;
    } else {
        IOLog("[IntelMausi]: PCI power management unsupported.\n");
    }
    provider->enablePCIPowerManagement();
    
    /* Get PCIe link information. */
    if (provider->findPCICapability(kIOPCIPCIExpressCapability, &pcieCapOffset)) {
#ifdef DEBUG
        pcieLinkCap = provider->extendedConfigRead32(pcieCapOffset + kIOPCIELinkCapability);
#endif
        pcieLinkCtl = provider->extendedConfigRead16(pcieCapOffset + kIOPCIELinkControl);
        DebugLog("[IntelMausi]: PCIe link capabilities: 0x%08x, link control: 0x%04x.\n", pcieLinkCap, pcieLinkCtl);
        aspmDisable = 0;
        
        if (ei->flags2 & FLAG2_DISABLE_ASPM_L0S)
            aspmDisable |= kIOPCIELinkCtlL0s;
        
        if (ei->flags2 & FLAG2_DISABLE_ASPM_L1)
            aspmDisable |= kIOPCIELinkCtlL1;

        if (aspmDisable)
            provider->extendedConfigWrite16(pcieCapOffset + kIOPCIELinkControl, (pcieLinkCtl & ~aspmDisable));

#ifdef DEBUG
        pcieLinkCtl = provider->extendedConfigRead16(pcieCapOffset + kIOPCIELinkControl);
        
        if (pcieLinkCtl & (kIOPCIELinkCtlASPM | kIOPCIELinkCtlClkReqEn))
            IOLog("[IntelMausi]: PCIe ASPM enabled. link control: 0x%04x.\n", pcieLinkCtl);
        else
            IOLog("[IntelMausi]: PCIe ASPM disabled. link control: 0x%04x.\n", pcieLinkCtl);
#endif  /* DEBUG */
    }
}

IOReturn IntelMausi::setPowerStateWakeAction(OSObject *owner, void *arg1, void *arg2, void *arg3, void *arg4)
{
    IntelMausi *ethCtlr = OSDynamicCast(IntelMausi, owner);
    IOPCIDevice *dev;
    UInt16 val16;
    UInt8 offset;
    
    if (ethCtlr) {
        dev = ethCtlr->pciDevice;
        offset = ethCtlr->pciPMCtrlOffset;
        
        val16 = dev->extendedConfigRead16(offset);
        
        val16 &= ~(kPCIPMCSPowerStateMask | kPCIPMCSPMEStatus | kPCIPMCSPMEEnable);
        val16 |= kPCIPMCSPowerStateD0;
        
        dev->extendedConfigWrite16(offset, val16);
        
        /* Restore the PCI Command register. */
        ethCtlr->intelEnablePCIDevice(dev);
    }
    return kIOReturnSuccess;
}

IOReturn IntelMausi::setPowerStateSleepAction(OSObject *owner, void *arg1, void *arg2, void *arg3, void *arg4)
{
    IntelMausi *ethCtlr = OSDynamicCast(IntelMausi, owner);
    IOPCIDevice *dev;
    UInt16 val16;
    UInt8 offset;
    
    if (ethCtlr) {
        dev = ethCtlr->pciDevice;
        offset = ethCtlr->pciPMCtrlOffset;
        
        val16 = dev->extendedConfigRead16(offset);
        
        val16 &= ~(kPCIPMCSPowerStateMask | kPCIPMCSPMEStatus | kPCIPMCSPMEEnable);
        
        if (ethCtlr->wolActive)
            val16 |= (kPCIPMCSPMEStatus | kPCIPMCSPMEEnable | kPCIPMCSPowerStateD3);
        else
            val16 |= kPCIPMCSPowerStateD3;
        
        dev->extendedConfigWrite16(offset, val16);
    }
    return kIOReturnSuccess;
}

void IntelMausi::intelEEPROMChecks(struct e1000_adapter *adapter)
{
	struct e1000_hw *hw = &adapter->hw;
	int ret_val;
	u16 buf = 0;
    
	if (hw->mac.type != e1000_82573)
		return;
    
	ret_val = e1000_read_nvm(hw, NVM_INIT_CONTROL2_REG, 1, &buf);
	le16_to_cpus(&buf);
    
	if (!ret_val && (!(buf & (1 << 0)))) {
		/* Deep Smart Power Down (DSPD) */
		IOLog("[IntelMausi]: Warning: detected DSPD enabled in EEPROM.\n");
	}
}

void IntelMausi::intelEnableIRQ(UInt32 newMask)
{
    intelWriteMem32(E1000_IMC, ~newMask);
    intelWriteMem32(E1000_IMS, newMask);
    intelFlush();
}

void IntelMausi::intelDisableIRQ()
{
    intelWriteMem32(E1000_IMC, 0xFFFFFFFF);
    intelFlush();
}

void IntelMausi::intelEnable()
{
    struct e1000_hw *hw = &adapterData.hw;
    const IONetworkMedium *selectedMedium;

    selectedMedium = getSelectedMedium();
    
    if (!selectedMedium) {
        DebugLog("[IntelMausi]: No medium selected. Falling back to autonegotiation.\n");
        selectedMedium = mediumTable[MEDIUM_INDEX_AUTO];
        setCurrentMedium(selectedMedium);
    }

#ifdef __PRIVATE_SPI__
    /* Check if we re waking up from sleep with WoL enabled and still have a valid link. */
    if (!intelCheckLink(&adapterData)) {
        setLinkStatus(kIONetworkLinkValid);
    }
    polling = false;
#else
    setLinkStatus(kIONetworkLinkValid);
#endif /* __PRIVATE_SPI__ */

#ifdef DEBUG
    if (adapterData.flags2 & FLAG2_HAS_PHY_WAKEUP) {
        struct e1000_hw *hw = &adapterData.hw;
        UInt16 phyData;
        
        hw->phy.ops.read_reg(hw, BM_WUS, &phyData);
        hw->phy.ops.write_reg(hw, BM_WUS, ~0);
        
        IOLog("[IntelMausi]: WUS=0x%04x\n", phyData);
    } else {
        IOLog("[IntelMausi]: WUS=0x%08x\n", intelReadMem32(E1000_WUS));
        
        intelWriteMem32(E1000_WUS, ~0);
    }
#endif

    intelSetupAdvForMedium(selectedMedium);

    e1000_phy_hw_reset(hw);
    
    if (hw->mac.type >= e1000_pch2lan)
		e1000_resume_workarounds_pchlan(hw);
    
    e1000e_power_up_phy(&adapterData);
    
    intelReset(&adapterData);
    
#if DISABLED_CODE
    
	/* DMA latency requirement to workaround jumbo issue */
	pm_qos_add_request(&adapter->netdev->pm_qos_req, PM_QOS_CPU_DMA_LATENCY,
                       PM_QOS_DEFAULT_VALUE);
    
#endif /* DISABLED_CODE */
    
	intelConfigure(&adapterData);
    
	/* From here on the code is the same as e1000e_up() */
	clear_bit(__E1000_DOWN, &adapterData.state);
    
	intelEnableIRQ(intrMask);
    
	hw->mac.get_link_status = true;
}

void IntelMausi::intelDisable()
{
    struct IntelAddrData addrData;
    struct e1000_hw *hw = &adapterData.hw;
    UInt32 linkStatus = kIONetworkLinkValid;
    UInt32 wufc = adapterData.wol;
    UInt32 ctrl, ctrlExt, rctl, status;
    int retval;
    
#ifdef __PRIVATE_SPI__
    polling = false;
#endif /* __PRIVATE_SPI__ */

    /* Flush LPIC. */
    intelFlushLPIC();

    status = intelReadMem32(E1000_STATUS);
    
    if (status & E1000_STATUS_LU)
        wufc &= ~E1000_WUFC_LNKC;

    if (wolActive && wufc) {
        /* Get interface's IP addresses. */
        getAddressList(&addrData);

        intelDown(&adapterData, false);
        intelSetupRxControl(&adapterData);
        
        rctl = intelReadMem32(E1000_RCTL);
        rctl &= ~(E1000_RCTL_UPE | E1000_RCTL_MPE);
        
        /* turn on all-multi mode if wake on multicast is enabled */
        if (wufc & E1000_WUFC_MC)
            rctl |= E1000_RCTL_MPE;

        intelWriteMem32(E1000_RCTL, rctl);

        ctrl = intelReadMem32(E1000_CTRL);
        ctrl |= E1000_CTRL_ADVD3WUC;
        
        if (!(adapterData.flags2 & FLAG2_HAS_PHY_WAKEUP))
            ctrl |= E1000_CTRL_EN_PHY_PWR_MGMT;
        
        intelWriteMem32(E1000_CTRL, ctrl);
        
        if (adapterData.hw.phy.media_type == e1000_media_type_fiber ||
            adapterData.hw.phy.media_type == e1000_media_type_internal_serdes) {
            /* keep the laser running in D3 */
            ctrlExt = intelReadMem32(E1000_CTRL_EXT);
            ctrlExt |= E1000_CTRL_EXT_SDP3_DATA;
            intelWriteMem32(E1000_CTRL_EXT, ctrlExt);
        }
        if (adapterData.flags & FLAG_IS_ICH)
            e1000_suspend_workarounds_ich8lan(hw);
        
        if (adapterData.flags2 & FLAG2_HAS_PHY_WAKEUP) {
            /* enable wakeup by the PHY */
            intelInitPhyWakeup(wufc, &addrData);
            
            DebugLog("[IntelMausi]: Configure wakeup by PHY.\n");
        } else {
            /* enable wakeup by the MAC */
            intelInitMacWakeup(wufc, &addrData);
            
            DebugLog("[IntelMausi]: Configure wakeup by MAC.\n");
        }
        DebugLog("[IntelMausi]: WUFC=0x%08x.\n", wufc);
    } else {
        intelDown(&adapterData, true);
        intelWriteMem32(E1000_WUC, 0);
        intelWriteMem32(E1000_WUFC, 0);
        intelPowerDownPhy(&adapterData);

        /* ULP seems to cause trouble on I218LM3 and I218V3. Enable it only for I219V/LM. */
        if (hw->mac.type >= e1000_pch_spt) {
            if (!(wufc & (E1000_WUFC_EX | E1000_WUFC_MC | E1000_WUFC_BC)))
            /* ULP does not support wake from unicast, multicast
             * or broadcast.
             */
                e1000_enable_ulp_lpt_lp(hw, false);
        }
    }
    
    /* Ensure that the appropriate bits are set in LPI_CTRL
     * for EEE in Sx
     */
    if ((hw->phy.type >= e1000_phy_i217) && adapterData.eee_advert && hw->dev_spec.ich8lan.eee_lp_ability) {
        UInt16 lpi_ctrl = 0;
        
        retval = hw->phy.ops.acquire(hw);
        if (!retval) {
            retval = e1e_rphy_locked(hw, I82579_LPI_CTRL, &lpi_ctrl);
            if (!retval) {
                if (adapterData.eee_advert &
                    hw->dev_spec.ich8lan.eee_lp_ability &
                    I82579_EEE_100_SUPPORTED)
                    lpi_ctrl |= I82579_LPI_CTRL_100_ENABLE;
                if (adapterData.eee_advert &
                    hw->dev_spec.ich8lan.eee_lp_ability &
                    I82579_EEE_1000_SUPPORTED)
                    lpi_ctrl |= I82579_LPI_CTRL_1000_ENABLE;
                
                retval = e1e_wphy_locked(hw, I82579_LPI_CTRL,
                                         lpi_ctrl);
                (void)retval;
            }
        }
        hw->phy.ops.release(hw);
    }
    
    if (linkUp) {
        linkUp = false;
        setLinkStatus(linkStatus);
        DebugLog("[IntelMausi]: Link down on en%u\n", netif->getUnitNumber());
    }
}

/**
 * intelConfigure - configure the hardware for Rx and Tx
 * @adapter: private board structure
 **/
void IntelMausi::intelConfigure(struct e1000_adapter *adapter)
{
    setMulticastMode(true);
    intelInitManageabilityPt(adapter);
    
    intelSetupRssHash(adapter);
    intelVlanStripEnable(adapter);
    intelConfigureTx(adapter);
    intelSetupRxControl(adapter);
    intelConfigureRx(adapter);
}

/**
 * intelConfigureTx - Configure Transmit Unit after Reset
 * @adapter: board private structure
 *
 * Configure the Tx unit of the MAC after a reset.
 **/
void IntelMausi::intelConfigureTx(struct e1000_adapter *adapter)
{
	struct e1000_hw *hw = &adapter->hw;
	UInt32 tctl, tarc;
    UInt32 txdctl;
    
    /* Setup the HW Tx Head and Tail descriptor pointers */
    intelInitTxRing();
    
	/* Set the Tx Interrupt Delay register */
	intelWriteMem32(E1000_TIDV, adapter->tx_int_delay);
	/* Tx irq moderation */
	intelWriteMem32(E1000_TADV, adapter->tx_abs_int_delay);
    
    txdctl = intelReadMem32(E1000_TXDCTL(0));

    /* Fix TXDCTL to disable descriptor prefetch for 82579, I217, I218 and I219. */
    if (chipType >= board_pch2lan) {
        txdctl = 0x01410000;
        intelWriteMem32(E1000_TXDCTL(0), txdctl);
    }
    /* erratum work around: set txdctl the same for both queues */
    intelWriteMem32(E1000_TXDCTL(1), txdctl);

	/* Program the Transmit Control Register */
	tctl = intelReadMem32(E1000_TCTL);
    tctl &= ~E1000_TCTL_CT;
    tctl |= E1000_TCTL_PSP | E1000_TCTL_RTLC | (E1000_COLLISION_THRESHOLD << E1000_CT_SHIFT);
    
	/* errata: program both queues to unweighted RR */
	if (adapter->flags & FLAG_TARC_SET_BIT_ZERO) {
		tarc = intelReadMem32(E1000_TARC(0));
		tarc |= 1;
		intelWriteMem32(E1000_TARC(0), tarc);
		tarc = intelReadMem32(E1000_TARC(1));
		tarc |= 1;
		intelWriteMem32(E1000_TARC(1), tarc);
	}
	intelWriteMem32(E1000_TCTL, tctl);
    
	hw->mac.ops.config_collision_dist(hw);
    
    /* SPT Si errata workaround to avoid data corruption */
    if (hw->mac.type == e1000_pch_spt) {
        UInt32 val;
        
        val = intelReadMem32(E1000_IOSFPC);
        val |= E1000_RCTL_RDMTS_HEX;
        intelWriteMem32(E1000_IOSFPC, val);
        
        /*
         * SPT and KBL Si errata workaround to avoid Tx hang.
         * Dropping the number of outstanding requests from
         * 3 to 2 in order to avoid a buffer overrun.
         */
        val = intelReadMem32(E1000_TARC(0));
        val &= ~E1000_TARC0_CB_MULTIQ_3_REQ;
        val |= E1000_TARC0_CB_MULTIQ_2_REQ;
        intelWriteMem32(E1000_TARC(0), val);
    }
}

/**
 * intelSetupRxControl - configure the receive control registers
 * @adapter: Board private structure
 **/

void IntelMausi::intelSetupRxControl(struct e1000_adapter *adapter)
{
	struct e1000_hw *hw = &adapter->hw;
	u32 rctl, rfctl;
    
	/* Workaround Si errata on PCHx - configure jumbo frame flow.
	 * If jumbo frames not set, program related MAC/PHY registers
	 * to h/w defaults
	 */
	if (hw->mac.type >= e1000_pch2lan) {
		s32 ret_val;
        
		if (mtu > ETH_DATA_LEN)
			ret_val = e1000_lv_jumbo_workaround_ich8lan(hw, true);
		else
			ret_val = e1000_lv_jumbo_workaround_ich8lan(hw, false);
        
		if (ret_val)
			DebugLog("[IntelMausi]: failed to enable/disable jumbo frame workaround mode.\n");
	}
	/* Program MC offset vector base */
	rctl = intelReadMem32(E1000_RCTL);
	rctl &= ~(3 << E1000_RCTL_MO_SHIFT);
	rctl |= E1000_RCTL_BAM | E1000_RCTL_LBM_NO | E1000_RCTL_RDMTS_HALF | (adapter->hw.mac.mc_filter_type << E1000_RCTL_MO_SHIFT);
    
	/* Do not Store bad packets */
	rctl &= ~E1000_RCTL_SBP;
    
	/* Enable Long Packet receive */
	if (mtu <= ETH_DATA_LEN)
		rctl &= ~E1000_RCTL_LPE;
	else
		rctl |= E1000_RCTL_LPE;
    
	/* Some systems expect that the CRC is included in SMBUS traffic. The
	 * hardware strips the CRC before sending to both SMBUS (BMC) and to
	 * host memory when this is enabled
     *
     * We setup hardware to always strip CRC as we want jumbo frame support
     * anyway even if it breaks management SMBUS traffic on some machines.
	 */
	rctl |= E1000_RCTL_SECRC;
    
	/* Workaround Si errata on 82577 PHY - configure IPG for jumbos */
	if ((hw->phy.type == e1000_phy_82577) && (rctl & E1000_RCTL_LPE)) {
		u16 phy_data;
        
		e1e_rphy(hw, PHY_REG(770, 26), &phy_data);
		phy_data &= 0xfff8;
		phy_data |= (1 << 2);
		e1e_wphy(hw, PHY_REG(770, 26), phy_data);
        
		e1e_rphy(hw, 22, &phy_data);
		phy_data &= 0x0fff;
		phy_data |= (1 << 14);
		e1e_wphy(hw, 0x10, 0x2823);
		e1e_wphy(hw, 0x11, 0x0003);
		e1e_wphy(hw, 22, phy_data);
	}
    
	/* Set buffer sizes to 2048 */
    //rctl |= (0x2 << E1000_RCTL_FLXB_SHIFT);
    rctl &= ~(E1000_RCTL_SZ_256 | E1000_RCTL_BSEX);
    
	/* Enable Extended Status in all Receive Descriptors */
	rfctl = intelReadMem32(E1000_RFCTL);
    rfctl |= (E1000_RFCTL_NEW_IPV6_EXT_DIS | E1000_RFCTL_IPV6_EX_DIS | E1000_RFCTL_EXTEN | E1000_RFCTL_NFSW_DIS | E1000_RFCTL_NFSR_DIS);
	intelWriteMem32(E1000_RFCTL, rfctl);
    
	intelWriteMem32(E1000_RCTL, rctl);
}

/**
 * intelConfigureRx - Configure Receive Unit after Reset
 * @adapter: board private structure
 *
 * Configure the Rx unit of the MAC after a reset.
 **/
void IntelMausi::intelConfigureRx(struct e1000_adapter *adapter)
{
	struct e1000_hw *hw = &adapter->hw;
	u32 rctl, rxcsum, ctrl_ext;
    
	/* disable receives while setting up the descriptors */
	rctl = intelReadMem32(E1000_RCTL);
    
	intelFlush();
	usleep_range(10000, 20000);
    
	/* set the Receive Delay Timer Register */
	intelWriteMem32(E1000_RDTR, adapter->rx_int_delay);
    
	/* irq moderation */
	intelWriteMem32(E1000_RADV, adapter->rx_abs_int_delay);
    
    /* Set interrupt throttle value. */
    intelWriteMem32(E1000_ITR, intrThrValue1000);
    
    /* Auto-Mask interrupts upon ICR access. */
	ctrl_ext = intelReadMem32(E1000_CTRL_EXT);
	ctrl_ext |= E1000_CTRL_EXT_IAME;
	intelWriteMem32(E1000_IAM, 0xffffffff);
	intelWriteMem32(E1000_CTRL_EXT, ctrl_ext);
	e1e_flush();
    
	/* Setup the HW Rx Head and Tail Descriptor Pointers and
	 * the Base and Length of the Rx Descriptor Ring
	 */
    intelInitRxRing();
    
	/* Enable Receive Checksum Offload for TCP and UDP */
	rxcsum = intelReadMem32(E1000_RXCSUM);
    rxcsum |= E1000_RXCSUM_TUOFL;
	intelWriteMem32(E1000_RXCSUM, rxcsum);
    
	/* With jumbo frames, excessive C-state transition latencies result
	 * in dropped transactions. Latency requirements will be adjusted
     * when the link has been established and speed is known.
	 */
	if ((mtu > ETH_DATA_LEN) && (adapter->flags & FLAG_IS_ICH)) {
        u32 rxdctl = intelReadMem32(E1000_RXDCTL(0));
            
        intelWriteMem32(E1000_RXDCTL(0), rxdctl | 0x3);
    }
	intelWriteMem32(E1000_RCTL, rctl);
}

/**
 * intelDown - quiesce the device and optionally reset the hardware
 * @adapter: board private structure
 * @reset: boolean flag to reset the hardware or not
 */
void IntelMausi::intelDown(struct e1000_adapter *adapter, bool reset)
{
	struct e1000_hw *hw = &adapter->hw;
	UInt32 tctl, rctl;
    
	/* signal that we're down so the interrupt handler does not
	 * reschedule our watchdog timer
	 */
	set_bit(__E1000_DOWN, &adapter->state);
    
	/* disable receives in the hardware */
	rctl = intelReadMem32(E1000_RCTL);
    rctl &= ~E1000_RCTL_EN;
	intelWriteMem32(E1000_RCTL, rctl);
    
	/* disable transmits in the hardware */
	tctl = intelReadMem32(E1000_TCTL);
	tctl &= ~E1000_TCTL_EN;
	intelWriteMem32(E1000_TCTL, tctl);
    
	/* flush both disables and wait for them to finish */
	intelFlush();
	usleep_range(10000, 20000);
    
	intelDisableIRQ();
    updateStatistics(adapter);
    
	adapter->link_speed = 0;
	adapter->link_duplex = 0;
    
	/* Disable Si errata workaround on PCHx for jumbo frame flow */
	if ((hw->mac.type >= e1000_pch2lan) && (mtu > ETH_DATA_LEN) && e1000_lv_jumbo_workaround_ich8lan(hw, false))
		DebugLog("[IntelMausi]: failed to disable jumbo frame workaround mode\n");
    
	if (reset)
		intelReset(adapter);
    else if (hw->mac.type >= e1000_pch_spt)
        intelFlushDescRings(adapter);
    
    clearDescriptors();
    
    if (chipType >= board_pch_lpt)
        requireMaxBusStall(0);
}

void IntelMausi::intelInitManageabilityPt(struct e1000_adapter *adapter)
{
	struct e1000_hw *hw = &adapter->hw;
	u32 manc, manc2h, mdef, i, j;
    
	if (!(adapter->flags & FLAG_MNG_PT_ENABLED))
		return;
    
	manc = intelReadMem32(E1000_MANC);
    
	/* enable receiving management packets to the host. this will probably
	 * generate destination unreachable messages from the host OS, but
	 * the packets will be handled on SMBUS
	 */
	manc |= E1000_MANC_EN_MNG2HOST;
	manc2h = intelReadMem32(E1000_MANC2H);
    
	switch (hw->mac.type) {
        default:
            manc2h |= (E1000_MANC2H_PORT_623 | E1000_MANC2H_PORT_664);
            break;
            
        case e1000_82574:
        case e1000_82583:
            /* Check if IPMI pass-through decision filter already exists;
             * if so, enable it.
             */
            for (i = 0, j = 0; i < 8; i++) {
                mdef = intelReadMem32(E1000_MDEF(i));
                
                /* Ignore filters with anything other than IPMI ports */
                if (mdef & ~(E1000_MDEF_PORT_623 | E1000_MDEF_PORT_664))
                    continue;
                
                /* Enable this decision filter in MANC2H */
                if (mdef)
                    manc2h |= (1 << i);
                
                j |= mdef;
            }
            
            if (j == (E1000_MDEF_PORT_623 | E1000_MDEF_PORT_664))
                break;
            
            /* Create new decision filter in an empty filter */
            for (i = 0, j = 0; i < 8; i++)
                if (intelReadMem32(E1000_MDEF(i)) == 0) {
                    intelWriteMem32(E1000_MDEF(i), (E1000_MDEF_PORT_623 | E1000_MDEF_PORT_664));
                    manc2h |= (1 << 1);
                    j++;
                    break;
                }
            
            if (!j)
                IOLog("[IntelMausi]: Unable to create IPMI pass-through filter.\n");
            break;
	}
	intelWriteMem32(E1000_MANC2H, manc2h);
	intelWriteMem32(E1000_MANC, manc);
}

/**
 * intelReset - bring the hardware into a known good state
 *
 * This function boots the hardware and enables some settings that
 * require a configuration cycle of the hardware - those cannot be
 * set/changed during runtime. After reset the device needs to be
 * properly configured for Rx, Tx etc.
 */
void IntelMausi::intelReset(struct e1000_adapter *adapter)
{
	struct e1000_mac_info *mac = &adapter->hw.mac;
	struct e1000_fc_info *fc = &adapter->hw.fc;
	struct e1000_hw *hw = &adapter->hw;
	u32 tx_space, min_tx_space, min_rx_space;
	u32 pba = adapter->pba;
	u16 hwm;
    
	/* reset Packet Buffer Allocation to default */
	intelWriteMem32(E1000_PBA, pba);
    
	if (adapter->max_frame_size > ETH_FRAME_LEN + ETH_FCS_LEN) {
		/* To maintain wire speed transmits, the Tx FIFO should be
		 * large enough to accommodate two full transmit packets,
		 * rounded up to the next 1KB and expressed in KB.  Likewise,
		 * the Rx FIFO should be large enough to accommodate at least
		 * one full receive packet and is similarly rounded up and
		 * expressed in KB.
		 */
		pba = intelReadMem32(E1000_PBA);
		/* upper 16 bits has Tx packet buffer allocation size in KB */
		tx_space = pba >> 16;
		/* lower 16 bits has Rx packet buffer allocation size in KB */
		pba &= 0xffff;
		/* the Tx fifo also stores 16 bytes of information about the Tx
		 * but don't include ethernet FCS because hardware appends it
		 */
		min_tx_space = (adapter->max_frame_size +
                        sizeof(struct e1000_tx_desc) - ETH_FCS_LEN) * 2;
		min_tx_space = ALIGN(min_tx_space, 1024);
		min_tx_space >>= 10;
		/* software strips receive CRC, so leave room for it */
		min_rx_space = adapter->max_frame_size;
		min_rx_space = ALIGN(min_rx_space, 1024);
		min_rx_space >>= 10;
        
		/* If current Tx allocation is less than the min Tx FIFO size,
		 * and the min Tx FIFO size is less than the current Rx FIFO
		 * allocation, take space away from current Rx allocation
		 */
		if ((tx_space < min_tx_space) &&
		    ((min_tx_space - tx_space) < pba)) {
			pba -= min_tx_space - tx_space;
            
			/* if short on Rx space, Rx wins and must trump Tx
			 * adjustment
			 */
			if (pba < min_rx_space)
				pba = min_rx_space;
		}
		intelWriteMem32(E1000_PBA, pba);
	}
    
	/* flow control settings
	 *
	 * The high water mark must be low enough to fit one full frame
	 * (or the size used for early receive) above it in the Rx FIFO.
	 * Set it to the lower of:
	 * - 90% of the Rx FIFO size, and
	 * - the full Rx FIFO size minus one full frame
	 */
	if (adapter->flags & FLAG_DISABLE_FC_PAUSE_TIME)
		fc->pause_time = 0xFFFF;
	else
		fc->pause_time = E1000_FC_PAUSE_TIME;
	fc->send_xon = true;
	fc->current_mode = fc->requested_mode;
    
	switch (hw->mac.type) {
        case e1000_ich9lan:
        case e1000_ich10lan:
            if (mtu > ETH_DATA_LEN) {
                pba = 14;
                intelWriteMem32(E1000_PBA, pba);
                fc->high_water = 0x2800;
                fc->low_water = fc->high_water - 8;
                break;
            }
            /* fall-through */
        default:
            hwm = min(((pba << 10) * 9 / 10),
                      ((pba << 10) - adapter->max_frame_size));
            
            fc->high_water = hwm & E1000_FCRTH_RTH;	/* 8-byte granularity */
            fc->low_water = fc->high_water - 8;
            break;
        case e1000_pchlan:
            /* Workaround PCH LOM adapter hangs with certain network
             * loads.  If hangs persist, try disabling Tx flow control.
             */
            if (mtu > ETH_DATA_LEN) {
                fc->high_water = 0x3500;
                fc->low_water = 0x1500;
            } else {
                fc->high_water = 0x5000;
                fc->low_water = 0x3000;
            }
            fc->refresh_time = 0x1000;
            break;
        case e1000_pch2lan:
        case e1000_pch_lpt:
        case e1000_pch_spt:
        case e1000_pch_cnp:
            fc->refresh_time = 0x0400;
            
            if (mtu <= ETH_DATA_LEN) {
                fc->high_water = 0x05C20;
                fc->low_water = 0x05048;
                fc->pause_time = 0x0650;
                break;
            }
            
            pba = 14;
            intelWriteMem32(E1000_PBA, pba);
            fc->high_water = ((pba << 10) * 9 / 10) & E1000_FCRTH_RTH;
            fc->low_water = ((pba << 10) * 8 / 10) & E1000_FCRTL_RTL;
            break;
	}
    
	/* Alignment of Tx data is on an arbitrary byte boundary with the
	 * maximum size per Tx descriptor limited only to the transmit
	 * allocation of the packet buffer minus 96 bytes with an upper
	 * limit of 24KB due to receive synchronization limitations.
	 */
	adapter->tx_fifo_limit = min_t(u32, ((intelReadMem32(E1000_PBA) >> 16) << 10) - 96,
                                   24 << 10);
    
    /* Set interrupt throttle value. */
    intelWriteMem32(E1000_ITR, intrThrValue100);
    
    if (hw->mac.type >= e1000_pch_spt)
        intelFlushDescRings(adapter);

	/* Allow time for pending master requests to run */
	mac->ops.reset_hw(hw);
    
	intelWriteMem32(E1000_WUC, 0);
    
	if (mac->ops.init_hw(hw))
		IOLog("[IntelMausi]: Hardware Error.\n");
    
	//e1000_update_mng_vlan(adapter);
    
	/* Enable h/w to recognize an 802.1Q VLAN Ethernet packet */
	intelWriteMem32(E1000_VET, ETH_P_8021Q);
    
	intelResetAdaptive(hw);
    
	/* initialize systim and reset the ns time counter */
	//e1000e_config_hwtstamp(adapter, &adapter->hwtstamp_config);
    
	/* Set EEE advertisement as appropriate */
	if (adapter->flags2 & FLAG2_HAS_EEE) {
		s32 ret_val;
		u16 adv_addr;
        
		switch (hw->phy.type) {
            case e1000_phy_82579:
                adv_addr = I82579_EEE_ADVERTISEMENT;
                break;
            case e1000_phy_i217:
                adv_addr = I217_EEE_ADVERTISEMENT;
                break;
            default:
                IOLog("[IntelMausi]: Invalid PHY type setting EEE advertisement.\n");
                return;
		}
        
		ret_val = hw->phy.ops.acquire(hw);
        
		if (ret_val) {
			IOLog("[IntelMausi]: EEE advertisement - unable to acquire PHY.\n");
			return;
		}
		e1000_write_emi_reg_locked(hw, adv_addr, hw->dev_spec.ich8lan.eee_disable ? 0 : adapter->eee_advert);
		hw->phy.ops.release(hw);
	}
	e1000_get_phy_info(hw);
    
	if ((adapter->flags & FLAG_HAS_SMART_POWER_DOWN) && !(adapter->flags & FLAG_SMART_POWER_DOWN)) {
		u16 phy_data = 0;
		/* speed up time to link by disabling smart power down, ignore
		 * the return value of this function because there is nothing
		 * different we would do if it failed
		 */
		e1e_rphy(hw, IGP02E1000_PHY_POWER_MGMT, &phy_data);
		phy_data &= ~IGP02E1000_PM_SPD;
		e1e_wphy(hw, IGP02E1000_PHY_POWER_MGMT, phy_data);
	}
}

/**
 * intelPowerDownPhy - Power down the PHY
 *
 * Power down the PHY so no link is implied when interface is down.
 * The PHY cannot be powered down if management or WoL is active.
 */
void IntelMausi::intelPowerDownPhy(struct e1000_adapter *adapter)
{
	if (adapter->hw.phy.ops.power_down)
		adapter->hw.phy.ops.power_down(&adapter->hw);
}

/**
 *  intelEnableMngPassThru - Check if management passthrough is needed
 *  @hw: pointer to the HW structure
 *
 *  Verifies the hardware needs to leave interface enabled so that frames can
 *  be directed to and from the management interface.
 **/
bool IntelMausi::intelEnableMngPassThru(struct e1000_hw *hw)
{
	u32 manc;
	u32 fwsm, factps;
    
	manc = intelReadMem32(E1000_MANC);
    
	if (!(manc & E1000_MANC_RCV_TCO_EN))
		return false;
    
	if (hw->mac.has_fwsm) {
		fwsm = intelReadMem32(E1000_FWSM);
		factps = intelReadMem32(E1000_FACTPS);
        
		if (!(factps & E1000_FACTPS_MNGCG) &&
		    ((fwsm & E1000_FWSM_MODE_MASK) ==
		     (e1000_mng_mode_pt << E1000_FWSM_MODE_SHIFT)))
			return true;
	} else if ((manc & E1000_MANC_SMBUS_EN) &&
               !(manc & E1000_MANC_ASF_EN)) {
		return true;
	}
    
	return false;
}

/**
 *  intelResetAdaptive - Reset Adaptive Interframe Spacing
 *  @hw: pointer to the HW structure
 *
 *  Reset the Adaptive Interframe Spacing throttle to default values.
 **/
void IntelMausi::intelResetAdaptive(struct e1000_hw *hw)
{
	struct e1000_mac_info *mac = &hw->mac;
    
	if (!mac->adaptive_ifs) {
		DebugLog("[IntelMausi]: Not in Adaptive IFS mode!\n");
		return;
	}
	mac->current_ifs_val = 0;
	mac->ifs_min_val = IFS_MIN;
	mac->ifs_max_val = IFS_MAX;
	mac->ifs_step_size = IFS_STEP;
	mac->ifs_ratio = IFS_RATIO;
    
	mac->in_ifs_mode = false;
	intelWriteMem32(E1000_AIT, 0);
}

/**
 *  intelUpdateAdaptive - Update Adaptive Interframe Spacing
 *  @hw: pointer to the HW structure
 *
 *  Update the Adaptive Interframe Spacing Throttle value based on the
 *  time between transmitted packets and time between collisions.
 **/
void IntelMausi::intelUpdateAdaptive(struct e1000_hw *hw)
{
	struct e1000_mac_info *mac = &hw->mac;
    
	if (!mac->adaptive_ifs) {
		DebugLog("[IntelMausi]: Not in Adaptive IFS mode!\n");
		return;
	}
    
	if ((mac->collision_delta * mac->ifs_ratio) > mac->tx_packet_delta) {
		if (mac->tx_packet_delta > MIN_NUM_XMITS) {
			mac->in_ifs_mode = true;
			if (mac->current_ifs_val < mac->ifs_max_val) {
				if (!mac->current_ifs_val)
					mac->current_ifs_val = mac->ifs_min_val;
				else
					mac->current_ifs_val += mac->ifs_step_size;
                
				intelWriteMem32(E1000_AIT, mac->current_ifs_val);
			}
		}
	} else {
		if (mac->in_ifs_mode && (mac->tx_packet_delta <= MIN_NUM_XMITS)) {
			mac->current_ifs_val = 0;
			mac->in_ifs_mode = false;
			intelWriteMem32(E1000_AIT, 0);
		}
	}
}

/**
 * intelVlanStripDisable - helper to disable HW VLAN stripping
 * @adapter: board private structure to initialize
 **/
void IntelMausi::intelVlanStripDisable(struct e1000_adapter *adapter)
{
	UInt32 ctrl;
    
	/* disable VLAN tag insert/strip */
	ctrl = intelReadMem32(E1000_CTRL);
	ctrl &= ~E1000_CTRL_VME;
	intelWriteMem32(E1000_CTRL, ctrl);
}

/**
 * intelVlanStripEnable - helper to enable HW VLAN stripping
 * @adapter: board private structure to initialize
 **/
void IntelMausi::intelVlanStripEnable(struct e1000_adapter *adapter)
{
	UInt32 ctrl;
    
	/* enable VLAN tag insert/strip */
	ctrl = intelReadMem32(E1000_CTRL);
	ctrl |= E1000_CTRL_VME;
	intelWriteMem32(E1000_CTRL, ctrl);
}

static const u32 rsskey[10] = {
    0xda565a6d, 0xc20e5b25, 0x3d256741, 0xb08fa343, 0xcb2bcad0,
    0xb4307bae, 0xa32dcb77, 0x0cf23080, 0x3bb7426a, 0xfa01acbe
};

void IntelMausi::intelSetupRssHash(struct e1000_adapter *adapter)
{
	UInt32 mrqc, rxcsum;
	int i;
    
	for (i = 0; i < 10; i++)
		intelWriteMem32(E1000_RSSRK(i), rsskey[i]);
    
	/* Direct all traffic to queue 0 */
	for (i = 0; i < 32; i++)
		intelWriteMem32(E1000_RETA(i), 0);
    
	/* Disable raw packet checksumming so that RSS hash is placed in
	 * descriptor on writeback.
	 */
	rxcsum = intelReadMem32(E1000_RXCSUM);
	rxcsum |= E1000_RXCSUM_PCSD;
    
	intelWriteMem32(E1000_RXCSUM, rxcsum);
    
	mrqc = (E1000_MRQC_RSS_FIELD_IPV4 |
            E1000_MRQC_RSS_FIELD_IPV4_TCP |
            E1000_MRQC_RSS_FIELD_IPV6 |
            E1000_MRQC_RSS_FIELD_IPV6_TCP |
            E1000_MRQC_RSS_FIELD_IPV6_TCP_EX);
    
    mrqc |= 0x01;
	intelWriteMem32(E1000_MRQC, mrqc);
}


/* Reset the NIC in case a tx deadlock or a pci error occurred. timerSource and txQueue
 * are stopped immediately but will be restarted by checkLinkStatus() when the link has
 * been reestablished.
 */

void IntelMausi::intelRestart()
{
    
#ifdef __PRIVATE_SPI__
    /* Stop output thread and flush txQueue */
    netif->stopOutputThread();
    netif->flushOutputQueue();
#else
    /* Stop and cleanup txQueue. */
    txQueue->stop();
    txQueue->flush();
#endif /* __PRIVATE_SPI__ */

    /*  Also set the link status to down. */
    if (linkUp)
        DebugLog("[IntelMausi]: Link down on en%u\n", netif->getUnitNumber());

    setLinkStatus(kIONetworkLinkValid);
    linkUp = false;

    /* Reset NIC and cleanup both descriptor rings. */
    intelDisableIRQ();
	intelReset(&adapterData);
    
    clearDescriptors();
    rxCleanedCount = rxNextDescIndex = 0;
    deadlockWarn = 0;
    forceReset = false;
    eeeMode = 0;

    /* Reinitialize NIC. */
    intelConfigure(&adapterData);
    
    /* From here on the code is the same as e1000e_up() */
	clear_bit(__E1000_DOWN, &adapterData.state);
    
	intelEnableIRQ(intrMask);
        
	adapterData.hw.mac.get_link_status = true;
}

void IntelMausi::intelInitTxRing()
{
	intelWriteMem32(E1000_TDBAL(0), (txPhyAddr & 0xffffffff));
	intelWriteMem32(E1000_TDBAH(0), (txPhyAddr >> 32));
	intelWriteMem32(E1000_TDLEN(0), kTxDescSize);
	intelWriteMem32(E1000_TDH(0), 0);
	intelWriteMem32(E1000_TDT(0), 0);
    txNextDescIndex = txDirtyIndex = txCleanBarrierIndex = 0;
    txNumFreeDesc = kNumTxDesc;
}

void IntelMausi::intelInitRxRing()
{
	intelWriteMem32(E1000_RDBAL(0), (rxPhyAddr & 0xffffffff));
	intelWriteMem32(E1000_RDBAH(0), (rxPhyAddr >> 32));
	intelWriteMem32(E1000_RDLEN(0), kRxDescSize);
	intelWriteMem32(E1000_RDH(0), 0);
    
    if (adapterData.flags2 & FLAG2_PCIM2PCI_ARBITER_WA)
        intelUpdateRxDescTail(kRxLastDesc);
    else
        intelWriteMem32(E1000_RDT(0), kRxLastDesc);
    
    rxCleanedCount = rxNextDescIndex = 0;
}

void IntelMausi::intelUpdateTxDescTail(UInt32 index)
{
    if (adapterData.flags2 & FLAG2_PCIM2PCI_ARBITER_WA) {
        struct e1000_hw *hw = &adapterData.hw;
        s32 ret = __ew32_prepare(hw);
        
        intelWriteMem32(E1000_TDT(0),index);
        
        if (!ret && (index != intelReadMem32(E1000_TDT(0)))) {
            UInt32 tctl = intelReadMem32(E1000_TCTL);
            
            intelWriteMem32(E1000_TCTL, tctl & ~E1000_TCTL_EN);
            forceReset = true;
            
            IOLog("[IntelMausi]: ME firmware caused invalid TDT - resetting.\n");
        }
    } else {
        intelWriteMem32(E1000_TDT(0), index);
    }
    txCleanBarrierIndex = txNextDescIndex;
}

void IntelMausi::intelUpdateRxDescTail(UInt32 index)
{
    struct e1000_hw *hw = &adapterData.hw;
    SInt32 ret = __ew32_prepare(hw);
    
    intelWriteMem32(E1000_RDT(0),index);
    
    if (!ret && (index != intelReadMem32(E1000_RDT(0)))) {
        UInt32 rctl = intelReadMem32(E1000_RCTL);
        
        intelWriteMem32(E1000_RCTL, rctl & ~E1000_RCTL_EN);
        forceReset = true;
        
        IOLog("[IntelMausi]: ME firmware caused invalid RDT - resetting.\n");
    }
}

inline void IntelMausi::intelEnablePCIDevice(IOPCIDevice *provider)
{
    UInt16 cmdReg;
    
    cmdReg = provider->extendedConfigRead16(kIOPCIConfigCommand);
    cmdReg |= (kIOPCICommandBusMaster | kIOPCICommandMemorySpace);
    cmdReg &= ~kIOPCICommandIOSpace;
	provider->extendedConfigWrite16(kIOPCIConfigCommand, cmdReg);
}

void IntelMausi::intelFlushDescriptors()
{
    /* flush pending descriptor writebacks to memory */
    intelWriteMem32(E1000_TIDV, adapterData.tx_int_delay | E1000_TIDV_FPD);
    //intelWriteMem32(E1000_RDTR, adapterData.rx_int_delay | E1000_RDTR_FPD);
    
    /* execute the writes immediately */
    intelFlush();
    
    /* due to rare timing issues, write to TIDV/RDTR again to ensure the
     * write is successful
     */
    intelWriteMem32(E1000_TIDV, adapterData.tx_int_delay | E1000_TIDV_FPD);
    //intelWriteMem32(E1000_RDTR, adapterData.rx_int_delay | E1000_RDTR_FPD);
    
    /* execute the writes immediately */
    intelFlush();
}

/**
 * intelFlushTxRing - remove all descriptors from the tx_ring
 *
 * We want to clear all pending descriptors from the TX ring.
 * zeroing happens when the HW reads the regs. We  assign the ring itself as
 * the data of the next descriptor. We don't care about the data we are about
 * to reset the HW.
 */
void IntelMausi::intelFlushTxRing(struct e1000_adapter *adapter)
{
    struct e1000_data_desc *desc = NULL;
    UInt32 tdt, tctl, txdLower = E1000_TXD_CMD_IFCS;
    UInt16 size = 512;
    
    tctl = intelReadMem32(E1000_TCTL);
    intelWriteMem32(E1000_TCTL, tctl | E1000_TCTL_EN);
    tdt = intelReadMem32(E1000_TDT(0));
    
    if (tdt != txNextDescIndex) {
        IOLog("[IntelMausi]: Failed to flush tx descriptor ring.\n");
        goto done;
    }
    DebugLog("[IntelMausi]: Flushing tx descriptor ring.\n");

    OSAddAtomic(-1, &txNumFreeDesc);
    desc = &txDescArray[txNextDescIndex++];
    txNextDescIndex &= kTxDescMask;

    desc->buffer_addr = OSSwapHostToLittleInt64(txPhyAddr);
    desc->lower.data = OSSwapHostToLittleInt32(txdLower | size);
    desc->upper.data = 0;
    
    intelWriteMem32(E1000_TDT(0), txNextDescIndex);
    intelFlush();
    usleep_range(200, 250);
    
done:
    return;
}

/**
 * intelFlushRxRing - remove all descriptors from the rx_ring
 *
 * Mark all descriptors in the RX ring as consumed and disable the rx ring
 */
void IntelMausi::intelFlushRxRing(struct e1000_adapter *adapter)
{
    UInt32 rctl, rxdctl;
    
    DebugLog("[IntelMausi]: Flushing rx descriptor ring.\n");

    rctl = intelReadMem32(E1000_RCTL);
    intelWriteMem32(E1000_RCTL, rctl & ~E1000_RCTL_EN);
    intelFlush();
    usleep_range(100, 150);
    
    rxdctl = intelReadMem32(E1000_RXDCTL(0));
    /* zero the lower 14 bits (prefetch and host thresholds) */
    rxdctl &= 0xffffc000;
    
    /* update thresholds: prefetch threshold to 31, host threshold to 1
     * and make sure the granularity is "descriptors" and not "cache lines"
     */
    rxdctl |= (0x1F | (1 << 8) | E1000_RXDCTL_THRESH_UNIT_DESC);
    
    intelWriteMem32(E1000_RXDCTL(0), rxdctl);
    /* momentarily enable the RX ring for the changes to take effect */
    intelWriteMem32(E1000_RCTL, rctl | E1000_RCTL_EN);
    intelFlush();
    usleep_range(100, 150);
    intelWriteMem32(E1000_RCTL, rctl & ~E1000_RCTL_EN);
}

/**
 * intelFlushDescRings - remove all descriptors from the descriptor rings
 *
 * In i219, the descriptor rings must be emptied before resetting the HW
 * or before changing the device state to D3 during runtime (runtime PM).
 *
 * Failure to do this will cause the HW to enter a unit hang state which can
 * only be released by PCI reset on the device
 *
 */

void IntelMausi::intelFlushDescRings(struct e1000_adapter *adapter)
{
    UInt16 hangState;
    u32 fext_nvm11, tdlen;
    struct e1000_hw *hw = &adapter->hw;
    
    /* First, disable MULR fix in FEXTNVM11 */
    fext_nvm11 = intelReadMem32(E1000_FEXTNVM11);
    fext_nvm11 |= E1000_FEXTNVM11_DISABLE_MULR_FIX;
    ew32(FEXTNVM11, fext_nvm11);
    /* do nothing if we're not in faulty state, or if the queue is empty */
    tdlen = er32(TDLEN(0));
    hangState = pciDevice->extendedConfigRead16(PCICFG_DESC_RING_STATUS);

    if (!(hangState & FLUSH_DESC_REQUIRED) || !tdlen)
        goto done;
    
    intelFlushTxRing(adapter);
    /* recheck, maybe the fault is caused by the rx ring */
    hangState = pciDevice->extendedConfigRead16(PCICFG_DESC_RING_STATUS);

    if (hangState & FLUSH_DESC_REQUIRED)
        intelFlushRxRing(adapter);

done:
    return;
}

bool IntelMausi::intelCheckLink(struct e1000_adapter *adapter)
{
	struct e1000_hw *hw = &adapter->hw;
	bool link_active = false;
	s32 ret_val = 0;
    
	/* get_link_status is set on LSC (link status) interrupt or
	 * Rx sequence error interrupt.  get_link_status will stay
	 * false until the check_for_link establishes link
	 * for copper adapters ONLY
	 */
	switch (hw->phy.media_type) {
        case e1000_media_type_copper:
            if (hw->mac.get_link_status) {
                ret_val = hw->mac.ops.check_for_link(hw);
                link_active = !hw->mac.get_link_status;
            } else {
                link_active = true;
            }
            break;
        case e1000_media_type_fiber:
            ret_val = hw->mac.ops.check_for_link(hw);
            link_active = !!(intelReadMem32(E1000_STATUS) & E1000_STATUS_LU);
            break;
        case e1000_media_type_internal_serdes:
            ret_val = hw->mac.ops.check_for_link(hw);
            link_active = adapter->hw.mac.serdes_has_link;
            break;
        default:
        case e1000_media_type_unknown:
            break;
	}
	if ((ret_val == E1000_ERR_PHY) && (hw->phy.type == e1000_phy_igp_3) &&
	    (intelReadMem32(E1000_CTRL) & E1000_PHY_CTRL_GBE_DISABLE)) {
		/* See e1000_kmrn_lock_loss_workaround_ich8lan() */
		IOLog("[IntelMausi]: Gigabit has been disabled, downgrading speed.\n");
	}
	return link_active;
}

/**
 * intelPhyReadStatus - Update the PHY register status snapshot
 * @adapter: board private structure
 **/
void IntelMausi::intelPhyReadStatus(struct e1000_adapter *adapter)
{
	struct e1000_hw *hw = &adapter->hw;
	struct e1000_phy_regs *phy = &adapter->phy_regs;
    
	if ((intelReadMem32(E1000_STATUS) & E1000_STATUS_LU) &&
	    (adapter->hw.phy.media_type == e1000_media_type_copper)) {
		int ret_val;
        
		ret_val = e1e_rphy(hw, MII_BMCR, &phy->bmcr);
		ret_val |= e1e_rphy(hw, MII_BMSR, &phy->bmsr);
		ret_val |= e1e_rphy(hw, MII_ADVERTISE, &phy->advertise);
		ret_val |= e1e_rphy(hw, MII_LPA, &phy->lpa);
		ret_val |= e1e_rphy(hw, MII_EXPANSION, &phy->expansion);
		ret_val |= e1e_rphy(hw, MII_CTRL1000, &phy->ctrl1000);
		ret_val |= e1e_rphy(hw, MII_STAT1000, &phy->stat1000);
		ret_val |= e1e_rphy(hw, MII_ESTATUS, &phy->estatus);
        
		if (ret_val)
			IOLog("[IntelMausi]: Error reading PHY register.\n");
	} else {
		/* Do not read PHY registers if link is not up
		 * Set values to typical power-on defaults
		 */
		phy->bmcr = (BMCR_SPEED1000 | BMCR_ANENABLE | BMCR_FULLDPLX);
		phy->bmsr = (BMSR_100FULL | BMSR_100HALF | BMSR_10FULL |
                     BMSR_10HALF | BMSR_ESTATEN | BMSR_ANEGCAPABLE |
                     BMSR_ERCAP);
		phy->advertise = (ADVERTISE_PAUSE_ASYM | ADVERTISE_PAUSE_CAP |
                          ADVERTISE_ALL | ADVERTISE_CSMA);
		phy->lpa = 0;
		phy->expansion = EXPANSION_ENABLENPAGE;
		phy->ctrl1000 = ADVERTISE_1000FULL;
		phy->stat1000 = 0;
		phy->estatus = (ESTATUS_1000_TFULL | ESTATUS_1000_THALF);
	}
}

void IntelMausi::intelInitPhyWakeup(UInt32 wufc, struct IntelAddrData *addrData)
{
    struct e1000_hw *hw = &adapterData.hw;
    UInt32 i, mac_reg, wuc, ad, num;
    int error;
    UInt16 phy_reg, wuc_enable, av;
    
    /* copy MAC RARs to PHY RARs */
    e1000_copy_rx_addrs_to_phy_ich8lan(hw);
    
    error = hw->phy.ops.acquire(hw);
    
    if (error) {
        DebugLog("[IntelMausi]: Failed to acquire PHY.\n");
        return;
    }
    /* Enable access to wakeup registers on and set page to BM_WUC_PAGE */
    error = e1000_enable_phy_wakeup_reg_access_bm(hw, &wuc_enable);
    
    if (error) {
        DebugLog("[IntelMausi]: Failed to access PHY wakeup registers.\n");
        goto release;
    }
    /* copy MAC MTA to PHY MTA - only needed for pchlan */
    for (i = 0; i < hw->mac.mta_reg_count; i++) {
        mac_reg = E1000_READ_REG_ARRAY(hw, E1000_MTA, i);
        hw->phy.ops.write_reg_page(hw, BM_MTA(i),
                                   (u16)(mac_reg & 0xFFFF));
        hw->phy.ops.write_reg_page(hw, BM_MTA(i) + 1,
                                   (u16)((mac_reg >> 16) & 0xFFFF));
    }
    /* configure PHY Rx Control register */
    hw->phy.ops.read_reg_page(hw, BM_RCTL, &phy_reg);
    mac_reg = intelReadMem32(E1000_RCTL);
    
    if (mac_reg & E1000_RCTL_UPE)
        phy_reg |= BM_RCTL_UPE;
    
    if (mac_reg & E1000_RCTL_MPE)
        phy_reg |= BM_RCTL_MPE;
    
    phy_reg &= ~(BM_RCTL_MO_MASK);
    
    if (mac_reg & E1000_RCTL_MO_3)
        phy_reg |= (((mac_reg & E1000_RCTL_MO_3) >> E1000_RCTL_MO_SHIFT) << BM_RCTL_MO_SHIFT);
    
    if (mac_reg & E1000_RCTL_BAM)
        phy_reg |= BM_RCTL_BAM;
    
    if (mac_reg & E1000_RCTL_PMCF)
        phy_reg |= BM_RCTL_PMCF;
    
    mac_reg = intelReadMem32(E1000_CTRL);
    
    if (mac_reg & E1000_CTRL_RFCE)
        phy_reg |= BM_RCTL_RFCE;
    
    /* Disable slave access to activate filters */
    phy_reg &= ~BM_RCTL_SAE;

    wuc = E1000_WUC_PME_EN;
    
    if (wufc & (E1000_WUFC_MAG | E1000_WUFC_LNKC))
        wuc |= E1000_WUC_APME;
    
    /*
     * Enable wakeup by ARP request and directed IPv4/IPv6 packets.
     */
    if (addrData->ipV4Count > 0)
        wufc |= (E1000_WUFC_EX | E1000_WUFC_ARP | E1000_WUFC_IP4);
    
    if (addrData->ipV6Count > 0)
        wufc |= (E1000_WUFC_EX | E1000_WUFC_IP6);

    /* enable PHY wakeup in MAC register */
    intelWriteMem32(E1000_WUFC, wufc);
    intelWriteMem32(E1000_WUC, (E1000_WUC_PHY_WAKE | E1000_WUC_APMPME | E1000_WUC_PME_STATUS | wuc));
    
    /*
     * Setup IPv4 and IPv6 wakeup address registers with the
     * retrieved address list.
     */
    av = 0x0000;
    num = (addrData->ipV4Count > kMaxAddrV4) ? kMaxAddrV4 : addrData->ipV4Count;
    
    for (i = 0; i < num; i++) {
        //av |= BIT(i + 1);
        ad = addrData->ipV4Addr[i];
        
        hw->phy.ops.write_reg_page(hw, BM_IP4AT0(i), (u16)(ad & 0xFFFF));
        hw->phy.ops.write_reg_page(hw, BM_IP4AT1(i), (u16)((ad >> 16) & 0xFFFF));
    }
    num = (addrData->ipV6Count > kMaxAddrV6) ? kMaxAddrV6 : addrData->ipV6Count;

    for (i = 0; i < num; i++) {
        av |= BIT(7 - i);
        
        ad = addrData->ipV6Addr[i].s6_addr32[0];
        hw->phy.ops.write_reg_page(hw, BM_IP6AT0(i), (u16)(ad & 0xFFFF));
        hw->phy.ops.write_reg_page(hw, BM_IP6AT1(i), (u16)((ad >> 16) & 0xFFFF));
        
        ad = addrData->ipV6Addr[i].s6_addr32[1];
        hw->phy.ops.write_reg_page(hw, BM_IP6AT2(i), (u16)(ad & 0xFFFF));
        hw->phy.ops.write_reg_page(hw, BM_IP6AT3(i), (u16)((ad >> 16) & 0xFFFF));
        
        ad = addrData->ipV6Addr[i].s6_addr32[2];
        hw->phy.ops.write_reg_page(hw, BM_IP6AT4(i), (u16)(ad & 0xFFFF));
        hw->phy.ops.write_reg_page(hw, BM_IP6AT5(i), (u16)((ad >> 16) & 0xFFFF));
        
        ad = addrData->ipV6Addr[i].s6_addr32[3];
        hw->phy.ops.write_reg_page(hw, BM_IP6AT6(i), (u16)(ad & 0xFFFF));
        hw->phy.ops.write_reg_page(hw, BM_IP6AT7(i), (u16)((ad >> 16) & 0xFFFF));
    }
    /*
     * Fix address valid mask as bit 15 is a duplicate of bit 7
     * and write to IPAV register.
     */
    if (av & BIT(7)) {
        av |= BIT(15);
    }
    hw->phy.ops.write_reg_page(hw, BM_IPAV, av);
    DebugLog("[IntelMausi]: PHY IPAV = 0x%04x.\n", av);
    DebugLog("[IntelMausi]: PHY WUFC = 0x%04x.\n", wufc);
    DebugLog("[IntelMausi]: PHY WUC = 0x%04x.\n", wuc);

    /* Configure PHY rx control */
    DebugLog("[IntelMausi]: PHY RCTL = 0x%04x.\n", phy_reg);
    hw->phy.ops.write_reg_page(hw, BM_RCTL, phy_reg);

    /* Configure and enable PHY wakeup in PHY registers */
    hw->phy.ops.write_reg_page(hw, BM_WUFC, wufc);
    hw->phy.ops.write_reg_page(hw, BM_WUC, wuc);
    
    /* activate PHY wakeup */
    wuc_enable |= BM_WUC_ENABLE_BIT | BM_WUC_HOST_WU_BIT;
    error = e1000_disable_phy_wakeup_reg_access_bm(hw, &wuc_enable);
    
    if (error)
        DebugLog("[IntelMausi]: Failed to set PHY Host Wakeup bit.\n");
    
release:
    hw->phy.ops.release(hw);
}

void IntelMausi::intelInitMacWakeup(UInt32 wufc, struct IntelAddrData *addrData)
{
    UInt32 av, num, i;
    
    /* Enable wakeup by ARP request and directed IPv4 packets. */
    if (addrData->ipV4Count > 0)
        wufc |= (E1000_WUFC_EX | E1000_WUFC_ARP | E1000_WUFC_IP4);
    
    /* Configure IPv4 wakeup address registers. */
    av = 0;
    num = (addrData->ipV4Count > kMaxAddrV4) ? kMaxAddrV4 : addrData->ipV4Count;
    
    for (i = 0; i < num; i++) {
        av |= BIT(i + 1);
        intelWriteMem32(E1000_IP4AT(i), addrData->ipV4Addr[i]);
    }

    /* Configure IPv6 wakeup address registers. */
    if (addrData->ipV6Count > 0) {
        wufc |= (E1000_WUFC_EX | E1000_WUFC_IP6);
        av |= BIT(16);

        for (i = 0; i < 4; i++) {
            intelWriteMem32(E1000_IP6AT(i), addrData->ipV6Addr[0].s6_addr32[i]);
        }
    }
    intelWriteMem32(E1000_IPAV, av);
    intelWriteMem32(E1000_WUFC, wufc);
    intelWriteMem32(E1000_WUC, (E1000_WUC_PME_EN | E1000_WUC_PME_STATUS));
}

void IntelMausi::intelSetupAdvForMedium(const IONetworkMedium *medium)
{
    struct e1000_hw *hw = &adapterData.hw;
    struct e1000_mac_info *mac = &hw->mac;
    
    if (adapterData.flags2 & FLAG2_HAS_EEE)
        hw->dev_spec.ich8lan.eee_disable = true;
    
    switch (medium->getIndex()) {
        case MEDIUM_INDEX_10HD:
            mac->forced_speed_duplex = ADVERTISE_10_HALF;
            hw->mac.autoneg = 0;
            break;
            
        case MEDIUM_INDEX_10FD:
            mac->forced_speed_duplex = ADVERTISE_10_FULL;
            hw->mac.autoneg = 0;
            break;
            
        case MEDIUM_INDEX_100HD:
            hw->phy.autoneg_advertised = ADVERTISED_100baseT_Half;
            hw->mac.autoneg = 1;
            hw->fc.requested_mode = e1000_fc_none;
            break;
            
        case MEDIUM_INDEX_100FD:
            hw->phy.autoneg_advertised = ADVERTISED_100baseT_Full;
            hw->mac.autoneg = 1;
            hw->fc.requested_mode = e1000_fc_none;
            break;
            
        case MEDIUM_INDEX_100FDFC:
            hw->phy.autoneg_advertised = ADVERTISED_100baseT_Full;
            hw->mac.autoneg = 1;
            hw->fc.requested_mode = e1000_fc_full;
            break;
            
        case MEDIUM_INDEX_1000FD:
            hw->phy.autoneg_advertised = ADVERTISED_1000baseT_Full;
            hw->mac.autoneg = 1;
            hw->fc.requested_mode = e1000_fc_none;
            break;
            
        case MEDIUM_INDEX_1000FDFC:
            hw->phy.autoneg_advertised = ADVERTISED_1000baseT_Full;
            hw->mac.autoneg = 1;
            hw->fc.requested_mode = e1000_fc_full;
            break;
            
        case MEDIUM_INDEX_1000FDEEE:
            hw->phy.autoneg_advertised = ADVERTISED_1000baseT_Full;
            hw->mac.autoneg = 1;
            hw->fc.requested_mode = e1000_fc_none;
            hw->dev_spec.ich8lan.eee_disable = false;
            break;
            
        case MEDIUM_INDEX_1000FDFCEEE:
            hw->phy.autoneg_advertised = ADVERTISED_1000baseT_Full;
            hw->mac.autoneg = 1;
            hw->fc.requested_mode = e1000_fc_full;
            hw->dev_spec.ich8lan.eee_disable = false;
            break;
            
        case MEDIUM_INDEX_100FDEEE:
            hw->phy.autoneg_advertised = ADVERTISED_100baseT_Full;
            hw->mac.autoneg = 1;
            hw->fc.requested_mode = e1000_fc_none;
            hw->dev_spec.ich8lan.eee_disable = false;
            break;
            
        case MEDIUM_INDEX_100FDFCEEE:
            hw->phy.autoneg_advertised = ADVERTISED_100baseT_Full;
            hw->mac.autoneg = 1;
            hw->fc.requested_mode = e1000_fc_full;
            hw->dev_spec.ich8lan.eee_disable = false;
            break;
            
        default:
            if (hw->phy.media_type == e1000_media_type_fiber) {
                hw->phy.autoneg_advertised = ADVERTISED_1000baseT_Full | ADVERTISED_FIBRE | ADVERTISED_Autoneg;
            } else {
                hw->phy.autoneg_advertised = (ADVERTISED_10baseT_Half | ADVERTISED_10baseT_Full |
                                              ADVERTISED_100baseT_Full | ADVERTISED_100baseT_Half |
                                              ADVERTISED_1000baseT_Full | ADVERTISED_Autoneg |
                                              ADVERTISED_TP | ADVERTISED_MII);
                
                if (adapterData.fc_autoneg)
                    hw->fc.requested_mode = e1000_fc_default;
                
                if (adapterData.flags2 & FLAG2_HAS_EEE)
                    hw->dev_spec.ich8lan.eee_disable = false;
            }
            hw->mac.autoneg = 1;
            break;
    }
    /* clear MDI, MDI(-X) override is only allowed when autoneg enabled */
    hw->phy.mdix = AUTO_ALL_MODES;
}

void IntelMausi::intelFlushLPIC()
{
    struct e1000_hw *hw = &adapterData.hw;
    UInt32 error;

#ifdef DEBUG
    UInt32 lpic;
#endif
    
    /* Flush LPIC. */
    error = hw->phy.ops.acquire(hw);
    
    if (error)
        return;

#ifdef DEBUG
    lpic = intelReadMem32(E1000_LPIC);
    DebugLog("[IntelMausi]: LPIC=0x%08x.\n", lpic);
#endif

    hw->phy.ops.release(hw);
}

void IntelMausi::setMaxLatency(UInt32 linkSpeed)
{
    struct e1000_hw *hw = &adapterData.hw;
    UInt32 rxa = intelReadMem32(E1000_PBA) & E1000_PBA_RXA_MASK;
    UInt32 latency;
    
    rxa = rxa << 9;
    latency = (rxa > hw->adapter->max_frame_size) ? (rxa - hw->adapter->max_frame_size) * (16000 / linkSpeed) : 0;
    
    if (maxLatency && (latency > maxLatency))
        latency = maxLatency;
    
    requireMaxBusStall(latency);
    
    DebugLog("[IntelMausi]: requireMaxBusStall(%uns).\n", latency);
}

UInt16 IntelMausi::intelSupportsEEE(struct e1000_adapter *adapter)
{
    struct e1000_hw *hw = &adapter->hw;
    struct e1000_dev_spec_ich8lan *dev_spec;
    SInt32 error;
    UInt16 result = 0;
    UInt16 lpa, adv, advAddr;

    if (!(adapter->flags2 & FLAG2_HAS_EEE))
        goto done;
    
    dev_spec = &hw->dev_spec.ich8lan;
    
    if (hw->dev_spec.ich8lan.eee_disable)
        goto done;
    
    switch (hw->phy.type) {
        case e1000_phy_82579:
            lpa = I82579_EEE_LP_ABILITY;
            advAddr = I82579_EEE_ADVERTISEMENT;
            break;
            
        case e1000_phy_i217:
            lpa = I217_EEE_LP_ABILITY;
            advAddr = I217_EEE_ADVERTISEMENT;
            break;
            
        default:
            goto done;
    }
    error = hw->phy.ops.acquire(hw);
    
    if (error)
        goto done;

    /* Save off link partner's EEE ability */
    error = e1000_read_emi_reg_locked(hw, lpa, &dev_spec->eee_lp_ability);
    
    if (error)
        goto release;
    
    /* Read EEE advertisement */
    error = e1000_read_emi_reg_locked(hw, advAddr, &adv);
    
    if (error)
        goto release;

    /* Enable EEE only for speeds in which the link partner is
     * EEE capable and for which we advertise EEE.
     */
    if (adv & dev_spec->eee_lp_ability & I82579_EEE_1000_SUPPORTED)
        result |= I82579_LPI_CTRL_1000_ENABLE;
    
    if (adv & dev_spec->eee_lp_ability & I82579_EEE_100_SUPPORTED)
        result |= I82579_LPI_CTRL_100_ENABLE;

    DebugLog("[IntelMausi]: EEE mode = 0x%04x, adv=0x%04x, lpa=0x%04x\n", result, adv, dev_spec->eee_lp_ability);

release:
    hw->phy.ops.release(hw);

done:
    return result;
}

SInt32 IntelMausi::intelEnableEEE(struct e1000_hw *hw, UInt16 mode)
{
    SInt32 error = 0;
    UInt16 pcsStatus, lpiCtrl, data;

    switch (hw->phy.type) {
        case e1000_phy_82579:
            pcsStatus = I82579_EEE_PCS_STATUS;
            break;
            
        case e1000_phy_i217:
            pcsStatus = I217_EEE_PCS_STATUS;
            break;
            
        default:
            goto done;
    }
    error = hw->phy.ops.acquire(hw);
    
    if (error)
        goto done;
    
    error = e1e_rphy_locked(hw, I82579_LPI_CTRL, &lpiCtrl);
    
    if (error)
        goto release;

    /* Clear bits that enable EEE in various speeds */
    lpiCtrl &= ~I82579_LPI_CTRL_ENABLE_MASK;

    /* Set the new EEE mode. */
    lpiCtrl |= mode;
    
    if (hw->phy.type == e1000_phy_82579) {
        error = e1000_read_emi_reg_locked(hw, I82579_LPI_PLL_SHUT, &data);
        
        if (error)
            goto release;
        
        data &= ~I82579_LPI_100_PLL_SHUT;
        e1000_write_emi_reg_locked(hw, I82579_LPI_PLL_SHUT, data);
    }
    
    /* R/Clr IEEE MMD 3.1 bits 11:10 - Tx/Rx LPI Received */
    error = e1000_read_emi_reg_locked(hw, pcsStatus, &data);
    
    if (error)
        goto release;
    
    error = e1e_wphy_locked(hw, I82579_LPI_CTRL, lpiCtrl);
    
release:
    hw->phy.ops.release(hw);
    
done:
    return error;
}



