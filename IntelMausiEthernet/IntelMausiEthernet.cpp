/* IntelMausiEthernet.cpp -- IntelMausi driver class implementation.
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

#pragma mark --- function prototypes ---

static inline void prepareTSO4(mbuf_t m, UInt32 *mssHeaderSize, UInt32 *payloadSize);
static inline void prepareTSO6(mbuf_t m, UInt32 *mssHeaderSize, UInt32 *payloadSize);

#pragma mark --- private data ---

static const struct intelDevice deviceTable[] = {
    { .pciDevId = E1000_DEV_ID_ICH8_IFE, .device = board_ich8lan, .deviceName = "82562V", .deviceInfo = &e1000_ich8_info },
	{ .pciDevId = E1000_DEV_ID_ICH8_IFE_G, .device = board_ich8lan, .deviceName = "82562G", .deviceInfo = &e1000_ich8_info },
	{ .pciDevId = E1000_DEV_ID_ICH8_IFE_GT, .device = board_ich8lan, .deviceName = "82562GT", .deviceInfo = &e1000_ich8_info },
	{ .pciDevId = E1000_DEV_ID_ICH8_IGP_AMT, .device = board_ich8lan, .deviceName = "82566DM", .deviceInfo = &e1000_ich8_info },
	{ .pciDevId = E1000_DEV_ID_ICH8_IGP_C, .device = board_ich8lan, .deviceName = "82566DC", .deviceInfo = &e1000_ich8_info },
	{ .pciDevId = E1000_DEV_ID_ICH8_IGP_M, .device = board_ich8lan, .deviceName = "82566MC", .deviceInfo = &e1000_ich8_info },
	{ .pciDevId = E1000_DEV_ID_ICH8_IGP_M_AMT, .device = board_ich8lan, .deviceName = "82566MM", .deviceInfo = &e1000_ich8_info },
	{ .pciDevId = E1000_DEV_ID_ICH8_82567V_3, .device = board_ich8lan, .deviceName = "82567V3", .deviceInfo = &e1000_ich8_info },
    
	{ .pciDevId = E1000_DEV_ID_ICH9_IFE, .device = board_ich9lan, .deviceName = "82562V2", .deviceInfo = &e1000_ich9_info },
	{ .pciDevId = E1000_DEV_ID_ICH9_IFE_G, .device = board_ich9lan, .deviceName = "82562G2", .deviceInfo = &e1000_ich9_info },
	{ .pciDevId = E1000_DEV_ID_ICH9_IFE_GT, .device = board_ich9lan, .deviceName = "82562GT2", .deviceInfo = &e1000_ich9_info },
	{ .pciDevId = E1000_DEV_ID_ICH9_IGP_AMT, .device = board_ich9lan, .deviceName = "82566DM2", .deviceInfo = &e1000_ich9_info },
	{ .pciDevId = E1000_DEV_ID_ICH9_IGP_C, .device = board_ich9lan, .deviceName = "82566DC2", .deviceInfo = &e1000_ich9_info },
	{ .pciDevId = E1000_DEV_ID_ICH9_BM, .device = board_ich9lan, .deviceName = "82567LM4", .deviceInfo = &e1000_ich9_info },
	{ .pciDevId = E1000_DEV_ID_ICH9_IGP_M, .device = board_ich9lan, .deviceName = "82567LF", .deviceInfo = &e1000_ich9_info },
	{ .pciDevId = E1000_DEV_ID_ICH9_IGP_M_AMT, .device = board_ich9lan, .deviceName = "82567LM", .deviceInfo = &e1000_ich9_info },
	{ .pciDevId = E1000_DEV_ID_ICH9_IGP_M_V, .device = board_ich9lan, .deviceName = "82567V", .deviceInfo = &e1000_ich9_info },
    
	{ .pciDevId = E1000_DEV_ID_ICH10_R_BM_LM, .device = board_ich9lan, .deviceName = "82567LM2", .deviceInfo = &e1000_ich9_info },
	{ .pciDevId = E1000_DEV_ID_ICH10_R_BM_LF, .device = board_ich9lan, .deviceName = "82567LF2", .deviceInfo = &e1000_ich9_info },
	{ .pciDevId = E1000_DEV_ID_ICH10_R_BM_V, .device = board_ich9lan, .deviceName = "82567V2", .deviceInfo = &e1000_ich9_info },
    
	{ .pciDevId = E1000_DEV_ID_ICH10_D_BM_LM, .device = board_ich10lan, .deviceName = "82567LM3", .deviceInfo = &e1000_ich10_info },
	{ .pciDevId = E1000_DEV_ID_ICH10_D_BM_LF, .device = board_ich10lan, .deviceName = "82567LF3", .deviceInfo = &e1000_ich10_info },
	{ .pciDevId = E1000_DEV_ID_ICH10_D_BM_V, .device = board_ich10lan, .deviceName = "82567V4", .deviceInfo = &e1000_ich10_info },
    
	{ .pciDevId = E1000_DEV_ID_PCH_M_HV_LM, .device = board_pchlan, .deviceName = "82578LM", .deviceInfo = &e1000_pch_info },
	{ .pciDevId = E1000_DEV_ID_PCH_M_HV_LC, .device = board_pchlan, .deviceName = "82578LC", .deviceInfo = &e1000_pch_info },
	{ .pciDevId = E1000_DEV_ID_PCH_D_HV_DM, .device = board_pchlan, .deviceName = "82578DM", .deviceInfo = &e1000_pch_info },
	{ .pciDevId = E1000_DEV_ID_PCH_D_HV_DC, .device = board_pchlan, .deviceName = "82578DC", .deviceInfo = &e1000_pch_info },
    
	{ .pciDevId = E1000_DEV_ID_PCH2_LV_LM, .device = board_pch2lan, .deviceName = "82579LM", .deviceInfo = &e1000_pch2_info },
	{ .pciDevId = E1000_DEV_ID_PCH2_LV_V, .device = board_pch2lan, .deviceName = "82579V", .deviceInfo = &e1000_pch2_info },
    
	{ .pciDevId = E1000_DEV_ID_PCH_LPT_I217_LM, .device = board_pch_lpt, .deviceName = "I217LM", .deviceInfo = &e1000_pch_lpt_info },
	{ .pciDevId = E1000_DEV_ID_PCH_LPT_I217_V, .device = board_pch_lpt, .deviceName = "I217V", .deviceInfo = &e1000_pch_lpt_info },
	{ .pciDevId = E1000_DEV_ID_PCH_LPTLP_I218_LM, .device = board_pch_lpt, .deviceName = "I218LM", .deviceInfo = &e1000_pch_lpt_info },
	{ .pciDevId = E1000_DEV_ID_PCH_LPTLP_I218_V, .device = board_pch_lpt, .deviceName = "I218V", .deviceInfo = &e1000_pch_lpt_info },
	{ .pciDevId = E1000_DEV_ID_PCH_I218_LM2, .device = board_pch_lpt, .deviceName = "I218LM2", .deviceInfo = &e1000_pch_lpt_info },
	{ .pciDevId = E1000_DEV_ID_PCH_I218_V2, .device = board_pch_lpt, .deviceName = "I218V2", .deviceInfo = &e1000_pch_lpt_info },
	{ .pciDevId = E1000_DEV_ID_PCH_I218_LM3, .device = board_pch_lpt, .deviceName = "I218LM3", .deviceInfo = &e1000_pch_lpt_info },
	{ .pciDevId = E1000_DEV_ID_PCH_I218_V3, .device = board_pch_lpt, .deviceName = "I218V3", .deviceInfo = &e1000_pch_lpt_info },
    { .pciDevId = E1000_DEV_ID_PCH_SPT_I219_LM, .device = board_pch_spt, .deviceName = "I219LM", .deviceInfo = &e1000_pch_spt_info },
    { .pciDevId = E1000_DEV_ID_PCH_SPT_I219_V, .device = board_pch_spt, .deviceName = "I219V", .deviceInfo = &e1000_pch_spt_info },
    { .pciDevId = E1000_DEV_ID_PCH_SPT_I219_LM2, .device = board_pch_spt, .deviceName = "I219LM2", .deviceInfo = &e1000_pch_spt_info },
    { .pciDevId = E1000_DEV_ID_PCH_SPT_I219_V2, .device = board_pch_spt, .deviceName = "I219V2", .deviceInfo = &e1000_pch_spt_info },
    { .pciDevId = E1000_DEV_ID_PCH_LBG_I219_LM3, .device = board_pch_spt, .deviceName = "I219LM3", .deviceInfo = &e1000_pch_spt_info },
    { .pciDevId = E1000_DEV_ID_PCH_SPT_I219_LM4, .device = board_pch_spt, .deviceName = "I219LM4", .deviceInfo = &e1000_pch_spt_info },
    { .pciDevId = E1000_DEV_ID_PCH_SPT_I219_V4, .device = board_pch_spt, .deviceName = "I219V4", .deviceInfo = &e1000_pch_spt_info },
    { .pciDevId = E1000_DEV_ID_PCH_SPT_I219_LM5, .device = board_pch_spt, .deviceName = "I219LM5", .deviceInfo = &e1000_pch_spt_info },
    { .pciDevId = E1000_DEV_ID_PCH_SPT_I219_V5, .device = board_pch_spt, .deviceName = "I219V5", .deviceInfo = &e1000_pch_spt_info },
    { .pciDevId = E1000_DEV_ID_PCH_CNP_I219_LM6, .device = board_pch_cnp, .deviceName = "I219LM6", .deviceInfo = &e1000_pch_cnp_info },
    { .pciDevId = E1000_DEV_ID_PCH_CNP_I219_V6, .device = board_pch_cnp, .deviceName = "I219V6", .deviceInfo = &e1000_pch_cnp_info },
    { .pciDevId = E1000_DEV_ID_PCH_CNP_I219_LM7, .device = board_pch_cnp, .deviceName = "I219LM7", .deviceInfo = &e1000_pch_cnp_info },
    { .pciDevId = E1000_DEV_ID_PCH_CNP_I219_V7, .device = board_pch_cnp, .deviceName = "I219V7", .deviceInfo = &e1000_pch_cnp_info },
    { .pciDevId = E1000_DEV_ID_PCH_ICP_I219_LM8, .device = board_pch_cnp, .deviceName = "I219LM8", .deviceInfo = &e1000_pch_cnp_info },
    { .pciDevId = E1000_DEV_ID_PCH_ICP_I219_V8, .device = board_pch_cnp, .deviceName = "I219V8", .deviceInfo = &e1000_pch_cnp_info },
    { .pciDevId = E1000_DEV_ID_PCH_ICP_I219_LM9, .device = board_pch_cnp, .deviceName = "I219LM9", .deviceInfo = &e1000_pch_cnp_info },
    { .pciDevId = E1000_DEV_ID_PCH_ICP_I219_V9, .device = board_pch_cnp, .deviceName = "I219V9", .deviceInfo = &e1000_pch_cnp_info },

    /* end of table */
    { .pciDevId = 0, .device = 0, .deviceName = NULL, .deviceInfo = NULL }
};

/* Power Management Support */
static IOPMPowerState powerStateArray[kPowerStateCount] =
{
    {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
    {1, kIOPMDeviceUsable, kIOPMPowerOn, kIOPMPowerOn, 0, 0, 0, 0, 0, 0, 0, 0}
};


static const char *speed1GName = "1-Gigabit";
static const char *speed100MName = "100-Megabit";
static const char *speed10MName = "10-Megabit";
static const char *duplexFullName = "Full-duplex";
static const char *duplexHalfName = "Half-duplex";

static const char *flowControlNames[kFlowControlTypeCount] = {
    "No flow-control",
    "Rx flow-control",
    "Tx flow-control",
    "Rx/Tx flow-control",
};

static const char* eeeNames[kEEETypeCount] = {
    "",
    ", energy-efficient-ethernet"
};

#pragma mark --- public methods ---

OSDefineMetaClassAndStructors(IntelMausi, super)

/* IOService (or its superclass) methods. */

bool IntelMausi::init(OSDictionary *properties)
{
    bool result;
    
    result = super::init(properties);
    
    if (result) {
        workLoop = NULL;
        commandGate = NULL;
        pciDevice = NULL;
        mediumDict = NULL;
        txQueue = NULL;
        interruptSource = NULL;
        timerSource = NULL;
        netif = NULL;
        netStats = NULL;
        etherStats = NULL;
        baseMap = NULL;
        baseAddr = NULL;
        flashMap = NULL;
        flashAddr = NULL;
        rxMbufCursor = NULL;
        txMbufCursor = NULL;
        rxPacketHead = NULL;
        rxPacketTail = NULL;
        rxPacketSize = 0;
        mcAddrList = NULL;
        mcListCount = 0;
        isEnabled = false;
        promiscusMode = false;
        multicastMode = false;
        linkUp = false;
        linkOpts = 0;
        polling = false;
        forceReset = false;
        eeeMode = 0;
        chip = 0;
        powerState = 0;
        pciDeviceData.vendor = 0;
        pciDeviceData.device = 0;
        pciDeviceData.subsystem_vendor = 0;
        pciDeviceData.subsystem_device = 0;
        pciDeviceData.revision = 0;
        adapterData.pdev = &pciDeviceData;
        mtu = ETH_DATA_LEN;
        wolCapable = false;
        wolActive = false;
        enableTSO4 = false;
        enableTSO6 = false;
        enableCSO6 = false;
        pciPMCtrlOffset = 0;
        maxLatency = 0;
    }
    
done:
    return result;
}

void IntelMausi::free()
{
    UInt32 i;
    
    DebugLog("free() ===>\n");
    
    if (workLoop) {
        if (interruptSource) {
            workLoop->removeEventSource(interruptSource);
            RELEASE(interruptSource);
        }
        if (timerSource) {
            workLoop->removeEventSource(timerSource);
            RELEASE(timerSource);
        }
        workLoop->release();
        workLoop = NULL;
    }
    RELEASE(commandGate);
    RELEASE(txQueue);
    RELEASE(mediumDict);
    
    for (i = MEDIUM_INDEX_AUTO; i < MEDIUM_INDEX_COUNT; i++)
        mediumTable[i] = NULL;
    
    RELEASE(baseMap);
    baseAddr = NULL;
    adapterData.hw.hw_addr = NULL;
    
    RELEASE(flashMap);
    flashAddr = NULL;
    adapterData.hw.flash_address = NULL;

    RELEASE(pciDevice);
    freeDMADescriptors();
    
    if (mcAddrList) {
        IOFree(mcAddrList, mcListCount * sizeof(IOEthernetAddress));
        mcAddrList = NULL;
        mcListCount = 0;
    }
    
    DebugLog("free() <===\n");
    
    super::free();
}

bool IntelMausi::start(IOService *provider)
{
    bool result;
    
    result = super::start(provider);
    
    if (!result) {
        IOLog("Ethernet [IntelMausi]: IOEthernetController::start failed.\n");
        goto done;
    }
    multicastMode = false;
    promiscusMode = false;
    mcAddrList = NULL;
    mcListCount = 0;
    
    pciDevice = OSDynamicCast(IOPCIDevice, provider);
    
    if (!pciDevice) {
        IOLog("Ethernet [IntelMausi]: No provider.\n");
        goto done;
    }
    pciDevice->retain();
    
    if (!pciDevice->open(this)) {
        IOLog("Ethernet [IntelMausi]: Failed to open provider.\n");
        goto error1;
    }
    if (!initPCIConfigSpace(pciDevice)) {
        goto error2;
    }
    getParams();
    
    if (!intelStart()) {
        goto error2;
    }    
    if (!setupMediumDict()) {
        IOLog("Ethernet [IntelMausi]: Failed to setup medium dictionary.\n");
        goto error2;
    }
    commandGate = getCommandGate();
    
    if (!commandGate) {
        IOLog("Ethernet [IntelMausi]: getCommandGate() failed.\n");
        goto error3;
    }
    commandGate->retain();
    
    if (!setupDMADescriptors()) {
        IOLog("Ethernet [IntelMausi]: Error allocating DMA descriptors.\n");
        goto error3;
    }

    if (!initEventSources(provider)) {
        IOLog("Ethernet [IntelMausi]: initEventSources() failed.\n");
        goto error4;
    }
    
    result = attachInterface(reinterpret_cast<IONetworkInterface**>(&netif));
    
    if (!result) {
        IOLog("Ethernet [IntelMausi]: attachInterface() failed.\n");
        goto error3;
    }
    pciDevice->close(this);
    result = true;
    
done:
    return result;
    
error4:
    freeDMADescriptors();
    
error3:
    RELEASE(commandGate);
    
error2:
    pciDevice->close(this);
    
error1:
    pciDevice->release();
    pciDevice = NULL;
    goto done;
}

void IntelMausi::stop(IOService *provider)
{
    UInt32 i;
    
    if (netif) {
        /*
         * Let the firmware know that the network interface is now closed.
         */
        e1000e_release_hw_control(&adapterData);

        detachInterface(netif);
        netif = NULL;
    }
    if (workLoop) {
        if (interruptSource) {
            workLoop->removeEventSource(interruptSource);
            RELEASE(interruptSource);
        }
        if (timerSource) {
            workLoop->removeEventSource(timerSource);
            RELEASE(timerSource);
        }
        workLoop->release();
        workLoop = NULL;
    }
    RELEASE(commandGate);
    RELEASE(txQueue);
    RELEASE(mediumDict);
    
    for (i = MEDIUM_INDEX_AUTO; i < MEDIUM_INDEX_COUNT; i++)
        mediumTable[i] = NULL;
    
    freeDMADescriptors();
    
    if (mcAddrList) {
        IOFree(mcAddrList, mcListCount * sizeof(IOEthernetAddress));
        mcAddrList = NULL;
        mcListCount = 0;
    }
    RELEASE(baseMap);
    baseAddr = NULL;
    adapterData.hw.hw_addr = NULL;
    
    RELEASE(flashMap);
    flashAddr = NULL;
    adapterData.hw.flash_address = NULL;
    
    RELEASE(pciDevice);
    
    super::stop(provider);
}

IOReturn IntelMausi::registerWithPolicyMaker(IOService *policyMaker)
{
    DebugLog("registerWithPolicyMaker() ===>\n");
    
    powerState = kPowerStateOn;
    
    DebugLog("registerWithPolicyMaker() <===\n");
    
    return policyMaker->registerPowerDriver(this, powerStateArray, kPowerStateCount);
}

IOReturn IntelMausi::setPowerState(unsigned long powerStateOrdinal, IOService *policyMaker)
{
    IOReturn result = IOPMAckImplied;
    
    DebugLog("setPowerState() ===>\n");
    
    if (powerStateOrdinal == powerState) {
        DebugLog("Ethernet [IntelMausi]: Already in power state %lu.\n", powerStateOrdinal);
        goto done;
    }
    DebugLog("Ethernet [IntelMausi]: switching to power state %lu.\n", powerStateOrdinal);
    
    if (powerStateOrdinal == kPowerStateOff)
        commandGate->runAction(setPowerStateSleepAction);
    else
        commandGate->runAction(setPowerStateWakeAction);
    
    powerState = powerStateOrdinal;
    
done:
    DebugLog("setPowerState() <===\n");
    
    return result;
}

void IntelMausi::systemWillShutdown(IOOptionBits specifier)
{
    DebugLog("systemWillShutdown() ===>\n");
    
    if ((kIOMessageSystemWillPowerOff | kIOMessageSystemWillRestart) & specifier) {
        disable(netif);
        
        /* Restore the original MAC address. */
        adapterData.hw.mac.ops.rar_set(&adapterData.hw, adapterData.hw.mac.perm_addr, 0);
        
        /*
         * Let the firmware know that the network interface is now closed
         */
        e1000e_release_hw_control(&adapterData);
    }
    
    DebugLog("systemWillShutdown() <===\n");
    
    /* Must call super on shutdown or system will stall. */
    super::systemWillShutdown(specifier);
}

/* IONetworkController methods. */

IOReturn IntelMausi::enable(IONetworkInterface *netif)
{
    IOReturn result = kIOReturnError;
    
    DebugLog("enable() ===>\n");
    
    if (isEnabled) {
        DebugLog("Ethernet [IntelMausi]: Interface already enabled.\n");
        result = kIOReturnSuccess;
        goto done;
    }
    if (!pciDevice || pciDevice->isOpen()) {
        IOLog("Ethernet [IntelMausi]: Unable to open PCI device.\n");
        goto done;
    }
    pciDevice->open(this);
    
    intelEnable();
    
    /* As we are using an msi the interrupt hasn't been enabled by start(). */
    interruptSource->enable();
    
    rxPacketHead = rxPacketTail = NULL;
    rxPacketSize = 0;

    txDescDoneCount = txDescDoneLast = 0;
    deadlockWarn = 0;

    polling = false;
    isEnabled = true;
    forceReset = false;
    eeeMode = 0;

    result = kIOReturnSuccess;
    
    DebugLog("enable() <===\n");
    
done:
    return result;
}

IOReturn IntelMausi::disable(IONetworkInterface *netif)
{
    IOReturn result = kIOReturnSuccess;
    
    DebugLog("disable() ===>\n");
    
    if (!isEnabled)
        goto done;
    
    netif->stopOutputThread();
    netif->flushOutputQueue();
    
    polling = false;
    isEnabled = false;
    forceReset = false;
    eeeMode = 0;

    timerSource->cancelTimeout();
    txDescDoneCount = txDescDoneLast = 0;
    
    /* We are using MSI so that we have to disable the interrupt. */
    interruptSource->disable();
    
    intelDisable();
    
    if (mcAddrList) {
        IOFree(mcAddrList, mcListCount * sizeof(IOEthernetAddress));
        mcAddrList = NULL;
        mcListCount = 0;
    }
    if (pciDevice && pciDevice->isOpen())
        pciDevice->close(this);
    
    DebugLog("disable() <===\n");
    
done:
    return result;
}

IOReturn IntelMausi::outputStart(IONetworkInterface *interface, IOOptionBits options)
{
    IOPhysicalSegment txSegments[kMaxSegs];
    struct e1000_data_desc *desc;
    struct e1000_context_desc *contDesc;
    mbuf_t m;
    IOReturn result = kIOReturnNoResources;
    UInt32 numDescs;
    UInt32 cmd;
    UInt32 opts;
    UInt32 word2;
    UInt32 len;
    UInt32 mss;
    UInt32 ipConfig;
    UInt32 tcpConfig;
    UInt32 word1;
    UInt32 numSegs;
    UInt32 lastSeg;
    UInt32 index;
    UInt32 offloadFlags;
    UInt16 vlanTag;
    UInt16 i;
    UInt16 count;
    
    //DebugLog("outputStart() ===>\n");
    count = 0;
    
    if (!(isEnabled && linkUp) || forceReset) {
        DebugLog("Ethernet [IntelMausi]: Interface down. Dropping packets.\n");
        goto done;
    }
    while ((txNumFreeDesc >= (kMaxSegs + kTxSpareDescs)) && (interface->dequeueOutputPackets(1, &m, NULL, NULL, NULL) == kIOReturnSuccess)) {
        numDescs = 0;
        cmd = 0;
        opts = (E1000_TXD_CMD_IDE | E1000_TXD_CMD_EOP | E1000_TXD_CMD_IFCS | E1000_TXD_CMD_RS);
        word2 = 0;
        len = 0;
        mss = 0;
        ipConfig = 0;
        tcpConfig = 0;
        offloadFlags = 0;
        
        if (mbuf_get_tso_requested(m, &offloadFlags, &mss)) {
            DebugLog("Ethernet [IntelMausi]: mbuf_get_tso_requested() failed. Dropping packet.\n");
            freePacket(m);
            continue;
        }
        /* First prepare the header and the command bits. */
        if (offloadFlags & (MBUF_TSO_IPV4 | MBUF_TSO_IPV6)) {
            numDescs = 1;
            opts = E1000_TXD_CMD_IFCS;
            
            if (offloadFlags & MBUF_TSO_IPV4) {
                /* Correct the pseudo header checksum and extract the header size. */
                prepareTSO4(m, &mss, &len);
                
                /* Prepare the context descriptor. */
                ipConfig = ((kIPv4CSumEnd << 16) | (kIPv4CSumOffset << 8) | kIPv4CSumStart);
                tcpConfig = ((kTCPv4CSumEnd << 16) | (kTCPv4CSumOffset << 8) | kTCPv4CSumStart);
                len |= (E1000_TXD_CMD_DEXT | E1000_TXD_CMD_TSE | E1000_TXD_CMD_IP | E1000_TXD_CMD_TCP);
                
                //DebugLog("Ethernet [IntelMausi]: TSO4 mssHeaderLen=0x%08x, payload=0x%08x\n", mss, len);
                
                /* Setup the command bits for TSO over IPv4. */
                cmd = (E1000_TXD_CMD_DEXT | E1000_TXD_CMD_TSE | E1000_TXD_DTYP_D);
                word2 = (E1000_TXD_OPTS_TXSM | E1000_TXD_OPTS_IXSM);
            } else {
                /* Correct the pseudo header checksum and extract the header size. */
                prepareTSO6(m, &mss, &len);
                
                /* Prepare the context descriptor. */
                ipConfig = ((kIPv6CSumEnd << 16) | (kIPv6CSumOffset << 8) | kIPv6CSumStart);
                tcpConfig = ((kTCPv6CSumEnd << 16) | (kTCPv6CSumOffset << 8) | kTCPv6CSumStart);
                len |= (E1000_TXD_CMD_DEXT | E1000_TXD_CMD_TSE | E1000_TXD_CMD_TCP);
                
                /* Setup the command bits for TSO over IPv6. */
                cmd = (E1000_TXD_CMD_DEXT | E1000_TXD_CMD_TSE | E1000_TXD_DTYP_D);
                word2 = E1000_TXD_OPTS_TXSM;
            }
        } else {
            mbuf_get_csum_requested(m, &offloadFlags, &mss);
            
            if (offloadFlags & (kChecksumUDPIPv6 | kChecksumTCPIPv6 | kChecksumIP | kChecksumUDP | kChecksumTCP)) {
                numDescs = 1;
                cmd = (E1000_TXD_CMD_DEXT | E1000_TXD_DTYP_D);
                
                if (offloadFlags & kChecksumTCP) {
                    ipConfig = ((kIPv4CSumEnd << 16) | (kIPv4CSumOffset << 8) | kIPv4CSumStart);
                    tcpConfig = ((kTCPv4CSumEnd << 16) | (kTCPv4CSumOffset << 8) | kTCPv4CSumStart);
                    len = (E1000_TXD_CMD_DEXT | E1000_TXD_CMD_IP | E1000_TXD_CMD_TCP);
                    mss = 0;
                    
                    word2 = (E1000_TXD_OPTS_TXSM | E1000_TXD_OPTS_IXSM);
                } else if (offloadFlags & kChecksumUDP) {
                    ipConfig = ((kIPv4CSumEnd << 16) | (kIPv4CSumOffset << 8) | kIPv4CSumStart);
                    tcpConfig = ((kUDPv4CSumEnd << 16) | (kUDPv4CSumOffset << 8) | kUDPv4CSumStart);
                    len = (E1000_TXD_CMD_DEXT | E1000_TXD_CMD_IP);
                    mss = 0;
                    
                    word2 = (E1000_TXD_OPTS_TXSM | E1000_TXD_OPTS_IXSM);
                } else if (offloadFlags & kChecksumIP) {
                    ipConfig = ((kIPv4CSumEnd << 16) | (kIPv4CSumOffset << 8) | kIPv4CSumStart);
                    tcpConfig = 0;
                    mss = 0;
                    len = (E1000_TXD_CMD_DEXT | E1000_TXD_CMD_IP);
                    
                    word2 = E1000_TXD_OPTS_IXSM;
                } else if (offloadFlags & kChecksumTCPIPv6) {
                    ipConfig = ((kIPv6CSumEnd << 16) | (kIPv6CSumOffset << 8) | kIPv6CSumStart);
                    tcpConfig = ((kTCPv6CSumEnd << 16) | (kTCPv6CSumOffset << 8) | kTCPv6CSumStart);
                    len = (E1000_TXD_CMD_DEXT | E1000_TXD_CMD_TCP);
                    mss = 0;
                    
                    word2 = E1000_TXD_OPTS_TXSM;
                } else if (offloadFlags & kChecksumUDPIPv6) {
                    ipConfig = ((kIPv6CSumEnd << 16) | (kIPv6CSumOffset << 8) | kIPv6CSumStart);
                    tcpConfig = ((kUDPv6CSumEnd << 16) | (kUDPv6CSumOffset << 8) | kUDPv6CSumStart);
                    len = E1000_TXD_CMD_DEXT;
                    mss = 0;
                    
                    word2 = E1000_TXD_OPTS_TXSM;
                }
            }
        }
        /* Next get the VLAN tag and command bit. */
        if (!mbuf_get_vlan_tag(m, &vlanTag)) {
            opts |= E1000_TXD_CMD_VLE;
            word2 |= (vlanTag << E1000_TX_FLAGS_VLAN_SHIFT);
        }
        /* Finally get the physical segments. */
        numSegs = txMbufCursor->getPhysicalSegmentsWithCoalesce(m, &txSegments[0], kMaxSegs);
        numDescs += numSegs;
        
        if (!numSegs) {
            DebugLog("Ethernet [IntelMausi]: getPhysicalSegmentsWithCoalesce() failed. Dropping packet.\n");
            etherStats->dot3TxExtraEntry.resourceErrors++;
            freePacket(m);
            continue;
        }
        OSAddAtomic(-numDescs, &txNumFreeDesc);
        index = txNextDescIndex;
        txNextDescIndex = (txNextDescIndex + numDescs) & kTxDescMask;
        lastSeg = numSegs - 1;
        
        /* Setup the context descriptor for TSO or checksum offload. */
        if (offloadFlags) {
            contDesc = (struct e1000_context_desc *)&txDescArray[index];
            
            txBufArray[index].mbuf = NULL;
            txBufArray[index].numDescs = 0;
            
#ifdef DEBUG
            txBufArray[index].pad = numSegs;
#endif
            
            contDesc->lower_setup.ip_config = OSSwapHostToLittleInt32(ipConfig);
            contDesc->upper_setup.tcp_config = OSSwapHostToLittleInt32(tcpConfig);
            contDesc->cmd_and_length = OSSwapHostToLittleInt32(len);
            contDesc->tcp_seg_setup.data = OSSwapHostToLittleInt32(mss);
            
            ++index &= kTxDescMask;
        }
        /* And finally fill in the data descriptors. */
        if (offloadFlags & (MBUF_TSO_IPV4 | MBUF_TSO_IPV6)) {
            for (i = 0; i < numSegs; i++) {
                desc = &txDescArray[index];
                word1 = (cmd | (txSegments[i].length & 0x000fffff));
                
                if (i == 0) {
                    word1 |= opts;
                    txBufArray[index].mbuf = NULL;
                    txBufArray[index].numDescs = 0;
                } else if (i == lastSeg) {
                    word1 |= (E1000_TXD_CMD_IDE | E1000_TXD_CMD_EOP | E1000_TXD_CMD_RS);
                    txBufArray[index].mbuf = m;
                    txBufArray[index].numDescs = numDescs;
                } else {
                    txBufArray[index].mbuf = NULL;
                    txBufArray[index].numDescs = 0;
                }
                
#ifdef DEBUG
                txBufArray[index].pad = (UInt32)txSegments[i].length;
#endif
                
                desc->buffer_addr = OSSwapHostToLittleInt64(txSegments[i].location);
                desc->lower.data = OSSwapHostToLittleInt32(word1);
                desc->upper.data = OSSwapHostToLittleInt32(word2);
                
                ++index &= kTxDescMask;
            }
        } else {
            for (i = 0; i < numSegs; i++) {
                desc = &txDescArray[index];
                word1 = (cmd | (txSegments[i].length & 0x000fffff));
                
                if (i == lastSeg) {
                    word1 |= opts;
                    txBufArray[index].mbuf = m;
                    txBufArray[index].numDescs = numDescs;
                } else {
                    txBufArray[index].mbuf = NULL;
                    txBufArray[index].numDescs = 0;
                }
                
#ifdef DEBUG
                txBufArray[index].pad = (UInt32)txSegments[i].length;
#endif
                
                desc->buffer_addr = OSSwapHostToLittleInt64(txSegments[i].location);
                desc->lower.data = OSSwapHostToLittleInt32(word1);
                desc->upper.data = OSSwapHostToLittleInt32(word2);
                
                ++index &= kTxDescMask;
            }
        }
        count++;
    }
    if (count)
        intelUpdateTxDescTail(txNextDescIndex);
    
    result = (txNumFreeDesc >= (kMaxSegs + kTxSpareDescs)) ? kIOReturnSuccess : kIOReturnNoResources;
    
    //DebugLog("outputStart() <===\n");
    
done:
    return result;
}

void IntelMausi::getPacketBufferConstraints(IOPacketBufferConstraints *constraints) const
{
    DebugLog("getPacketBufferConstraints() ===>\n");
    
	constraints->alignStart = kIOPacketBufferAlign1;
	constraints->alignLength = kIOPacketBufferAlign1;
    
    DebugLog("getPacketBufferConstraints() <===\n");
}

IOOutputQueue* IntelMausi::createOutputQueue()
{
    DebugLog("createOutputQueue() ===>\n");
    
    DebugLog("createOutputQueue() <===\n");
    
    return IOBasicOutputQueue::withTarget(this);
}

const OSString* IntelMausi::newVendorString() const
{
    DebugLog("newVendorString() ===>\n");
    
    DebugLog("newVendorString() <===\n");
    
    return OSString::withCString("Intel");
}

const OSString* IntelMausi::newModelString() const
{
    DebugLog("newModelString() ===>\n");
    DebugLog("newModelString() <===\n");
    
    return OSString::withCString(deviceTable[chip].deviceName);
}

bool IntelMausi::configureInterface(IONetworkInterface *interface)
{
    char modelName[kNameLenght];
    IONetworkData *data;
    IOReturn error;

    bool result;
    
    DebugLog("configureInterface() ===>\n");
    
    result = super::configureInterface(interface);
    
    if (!result)
        goto done;
	
    /* Get the generic network statistics structure. */
    data = interface->getParameter(kIONetworkStatsKey);
    
    if (data) {
        netStats = (IONetworkStats *)data->getBuffer();
        
        if (!netStats) {
            IOLog("Ethernet [IntelMausi]: Error getting IONetworkStats\n.");
            result = false;
            goto done;
        }
    }
    /* Get the Ethernet statistics structure. */
    data = interface->getParameter(kIOEthernetStatsKey);
    
    if (data) {
        etherStats = (IOEthernetStats *)data->getBuffer();
        
        if (!etherStats) {
            IOLog("Ethernet [IntelMausi]: Error getting IOEthernetStats\n.");
            result = false;
            goto done;
        }
    }
    error = interface->configureOutputPullModel(384, 0, 0, IONetworkInterface::kOutputPacketSchedulingModelNormal);
    
    if (error != kIOReturnSuccess) {
        IOLog("Ethernet [IntelMausi]: configureOutputPullModel() failed\n.");
        result = false;
        goto done;
    }
    error = interface->configureInputPacketPolling(kNumRxDesc, kIONetworkWorkLoopSynchronous);
    
    if (error != kIOReturnSuccess) {
        IOLog("Ethernet [IntelMausi]: configureInputPacketPolling() failed\n.");
        result = false;
        goto done;
    }
    snprintf(modelName, kNameLenght, "Intel %s PCI Express Gigabit Ethernet", deviceTable[chip].deviceName);
    setProperty("model", modelName);
    
done:
    DebugLog("configureInterface() <===\n");

    return result;
}

bool IntelMausi::createWorkLoop()
{
    DebugLog("createWorkLoop() ===>\n");
    
    workLoop = IOWorkLoop::workLoop();
    
    DebugLog("createWorkLoop() <===\n");
    
    return workLoop ? true : false;
}

IOWorkLoop* IntelMausi::getWorkLoop() const
{
    DebugLog("getWorkLoop() ===>\n");
    
    DebugLog("getWorkLoop() <===\n");
    
    return workLoop;
}

IOReturn IntelMausi::setPromiscuousMode(bool active)
{
    struct e1000_hw *hw = &adapterData.hw;
    UInt32 rxControl;
    
    DebugLog("setPromiscuousMode() ===>\n");
    
    rxControl = intelReadMem32(E1000_RCTL);
    rxControl &= ~(E1000_RCTL_UPE | E1000_RCTL_MPE);

    if (active) {
        DebugLog("Ethernet [IntelMausi]: Promiscuous mode enabled.\n");
        rxControl |= (E1000_RCTL_UPE | E1000_RCTL_MPE);
    } else {
        DebugLog("Ethernet [IntelMausi]: Promiscuous mode disabled.\n");
        hw->mac.ops.update_mc_addr_list(hw, (UInt8 *)mcAddrList, mcListCount);
    }
    intelWriteMem32(E1000_RCTL, rxControl);
    promiscusMode = active;

    DebugLog("setPromiscuousMode() <===\n");
    
    return kIOReturnSuccess;
}

IOReturn IntelMausi::setMulticastMode(bool active)
{
    struct e1000_hw *hw = &adapterData.hw;
    UInt32 rxControl;

    DebugLog("setMulticastMode() ===>\n");

    rxControl = intelReadMem32(E1000_RCTL);
    rxControl &= ~(E1000_RCTL_UPE | E1000_RCTL_MPE);
    
    if (active)
        hw->mac.ops.update_mc_addr_list(hw, (UInt8 *)mcAddrList, mcListCount);
    else
        hw->mac.ops.update_mc_addr_list(hw, NULL, 0);

    intelWriteMem32(E1000_RCTL, rxControl);
    multicastMode = active;

    DebugLog("setMulticastMode() <===\n");
    
    return kIOReturnSuccess;
}

IOReturn IntelMausi::setMulticastList(IOEthernetAddress *addrs, UInt32 count)
{
    struct e1000_hw *hw = &adapterData.hw;
    IOEthernetAddress *newList;
    vm_size_t newSize;
    IOReturn result = kIOReturnNoMemory;

    DebugLog("setMulticastList() ===>\n");

    if (count) {
        newSize = count * sizeof(IOEthernetAddress);
        newList = (IOEthernetAddress *)IOMalloc(newSize);
        
        if (newList) {
            if (mcAddrList)
                IOFree(mcAddrList, mcListCount * sizeof(IOEthernetAddress));

            memcpy(newList, addrs, newSize);
            mcAddrList = newList;
            mcListCount = count;
            hw->mac.ops.update_mc_addr_list(hw, (UInt8 *)newList, count);

            result = kIOReturnSuccess;
        }
    } else {
        if (mcAddrList) {
            IOFree(mcAddrList, mcListCount * sizeof(IOEthernetAddress));
            mcAddrList = NULL;
            mcListCount = 0;
        }
        hw->mac.ops.update_mc_addr_list(hw, NULL, 0);

        result = kIOReturnSuccess;
    }

    DebugLog("setMulticastList() <===\n");
    
    return result;
}

IOReturn IntelMausi::getChecksumSupport(UInt32 *checksumMask, UInt32 checksumFamily, bool isOutput)
{
    IOReturn result = kIOReturnUnsupported;
    
    DebugLog("getChecksumSupport() ===>\n");
    
    if ((checksumFamily == kChecksumFamilyTCPIP) && checksumMask) {
        if (isOutput) {
            *checksumMask = (kChecksumTCP | kChecksumUDP | kChecksumIP);
            
            if (enableCSO6)
                *checksumMask |= (kChecksumTCPIPv6 | kChecksumUDPIPv6);
        } else {
            *checksumMask = (kChecksumTCP | kChecksumUDP | kChecksumIP | kChecksumTCPIPv6 | kChecksumUDPIPv6);
        }
        result = kIOReturnSuccess;
    }
    DebugLog("getChecksumSupport() <===\n");
    
    return result;
}

UInt32 IntelMausi::getFeatures() const
{
    UInt32 features = (kIONetworkFeatureMultiPages | kIONetworkFeatureHardwareVlan);
    
    DebugLog("getFeatures() ===>\n");
    
    if (enableTSO4)
        features |= kIONetworkFeatureTSOIPv4;
    
    if (enableTSO6)
        features |= kIONetworkFeatureTSOIPv6;
        
    DebugLog("getFeatures() <===\n");
    
    return features;
}

IOReturn IntelMausi::setWakeOnMagicPacket(bool active)
{
    IOReturn result = kIOReturnUnsupported;
    
    DebugLog("setWakeOnMagicPacket() ===>\n");
    
    if (wolCapable) {
        wolActive = active;
        DebugLog("Ethernet [IntelMausi]: Wake on magic packet %s.\n", active ? "enabled" : "disabled");
        result = kIOReturnSuccess;
    }
    
    DebugLog("setWakeOnMagicPacket() <===\n");
    
    return result;
}

IOReturn IntelMausi::getPacketFilters(const OSSymbol *group, UInt32 *filters) const
{
    IOReturn result = kIOReturnSuccess;
    
    DebugLog("getPacketFilters() ===>\n");
    
    if ((group == gIOEthernetWakeOnLANFilterGroup) && wolCapable) {
        *filters = kIOEthernetWakeOnMagicPacket;
        DebugLog("Ethernet [IntelMausi]: kIOEthernetWakeOnMagicPacket added to filters.\n");
    } else {
        result = super::getPacketFilters(group, filters);
    }
    
    DebugLog("getPacketFilters() <===\n");
    
    return result;
}

IOReturn IntelMausi::setHardwareAddress(const IOEthernetAddress *addr)
{
    struct e1000_hw *hw = &adapterData.hw;
    IOReturn result = kIOReturnError;
    
    DebugLog("setHardwareAddress() ===>\n");
    
    if (addr && is_valid_ether_addr(&addr->bytes[0])) {
        memcpy(hw->mac.addr, &addr->bytes[0], kIOEthernetAddressSize);
        
        hw->mac.ops.rar_set(hw, hw->mac.addr, 0);
        
        result = kIOReturnSuccess;
    }
    
    DebugLog("setHardwareAddress() <===\n");
    
    return result;
}

/* Methods inherited from IOEthernetController. */
IOReturn IntelMausi::getHardwareAddress(IOEthernetAddress *addr)
{
    IOReturn result = kIOReturnError;
    
    DebugLog("getHardwareAddress() ===>\n");
    
    //IOLog("Ethernet [IntelMausi]: RAH = 0x%x, RAL = 0x%x\n", intelReadMem32(E1000_RAH(0)), intelReadMem32(E1000_RAL(0)));
    
    if (addr) {
        memcpy(&addr->bytes[0], adapterData.hw.mac.addr, kIOEthernetAddressSize);
        
        if (is_valid_ether_addr(&addr->bytes[0]))
            result = kIOReturnSuccess;
    }
    
    DebugLog("getHardwareAddress() <===\n");
    
    return result;
}

IOReturn IntelMausi::selectMedium(const IONetworkMedium *medium)
{
    IOReturn result = kIOReturnSuccess;
    
    DebugLog("selectMedium() ===>\n");

    if (medium) {
        intelSetupAdvForMedium(medium);
        setCurrentMedium(medium);
        
        timerSource->cancelTimeout();
        updateStatistics(&adapterData);
        intelRestart();
    }

    DebugLog("selectMedium() <===\n");
    
done:
    return result;
}

IOReturn IntelMausi::getMaxPacketSize(UInt32 * maxSize) const
{
    DebugLog("getMaxPacketSize() ===>\n");
    
    *maxSize = kMaxPacketSize;
    
    DebugLog("getMaxPacketSize() <===\n");
    
    return kIOReturnSuccess;
}

IOReturn IntelMausi::setMaxPacketSize(UInt32 maxSize)
{
    IOReturn result = kIOReturnError;
    
    DebugLog("setMaxPacketSize() ===>\n");
    
    if (maxSize <= kMaxPacketSize) {
        mtu = maxSize - (ETH_HLEN + ETH_FCS_LEN);
        adapterData.max_frame_size = maxSize;

        DebugLog("Ethernet [IntelMausi]: maxSize: %u, mtu: %u\n", maxSize, mtu);
        
        /* Force reinitialization. */
        setLinkDown();
        timerSource->cancelTimeout();
        updateStatistics(&adapterData);
        intelRestart();
        
        result = kIOReturnSuccess;
    }
    
    DebugLog("setMaxPacketSize() <===\n");
    
    return result;
}

#pragma mark --- common interrupt methods ---

void IntelMausi::txInterrupt()
{
    UInt32 descStatus;
    SInt32 cleaned;
    
    while (txDirtyIndex != txCleanBarrierIndex) {
        if (txBufArray[txDirtyIndex].mbuf) {
            descStatus = OSSwapLittleToHostInt32(txDescArray[txDirtyIndex].upper.data);
            
            if (!(descStatus & E1000_TXD_STAT_DD))
                goto done;
            
            /* First free the attached mbuf and clean up the buffer info. */
            freePacket(txBufArray[txDirtyIndex].mbuf);
            txBufArray[txDirtyIndex].mbuf = NULL;
            
            cleaned = txBufArray[txDirtyIndex].numDescs;
            txBufArray[txDirtyIndex].numDescs = 0;
            
            /* Finally update the number of free descriptors. */
            OSAddAtomic(cleaned, &txNumFreeDesc);
            txDescDoneCount += cleaned;
        }
        /* Increment txDirtyIndex. */
        ++txDirtyIndex &= kTxDescMask;
    }

    //DebugLog("Ethernet [IntelMausi]: txInterrupt oldIndex=%u newIndex=%u\n", oldDirtyIndex, txDirtyDescIndex);
    
done:
    if (txNumFreeDesc > kTxQueueWakeTreshhold)
        netif->signalOutputThread();
}

UInt32 IntelMausi::rxInterrupt(IONetworkInterface *interface, uint32_t maxCount, IOMbufQueue *pollQueue, void *context)
{
    IOPhysicalSegment rxSegment;
    union e1000_rx_desc_extended *desc;
    mbuf_t bufPkt, newPkt;
    UInt64 addr;
    UInt32 status;
    UInt32 goodPkts = 0;
    UInt32 pktSize;
    UInt32 n;
    UInt16 vlanTag;
    bool replaced;

    if (rxDescArray == NULL)
        return 0;

    desc = &rxDescArray[rxNextDescIndex];

    while (((status = OSSwapLittleToHostInt32(desc->wb.upper.status_error)) & E1000_RXD_STAT_DD) && (goodPkts < maxCount)) {
        addr = rxBufArray[rxNextDescIndex].phyAddr;
        bufPkt = rxBufArray[rxNextDescIndex].mbuf;
        pktSize = OSSwapLittleToHostInt16(desc->wb.upper.length);
        vlanTag = (status & E1000_RXD_STAT_VP) ? (OSSwapLittleToHostInt16(desc->wb.upper.vlan) & E1000_RXD_SPC_VLAN_MASK) : 0;
        
        /* Skip bad packet. */
        if (status & E1000_RXDEXT_ERR_FRAME_ERR_MASK) {
            DebugLog("Ethernet [IntelMausi]: Bad packet.\n");
            etherStats->dot3StatsEntry.internalMacReceiveErrors++;
            discardPacketFragment();
            goto nextDesc;
        }
        newPkt = replaceOrCopyPacket(&bufPkt, pktSize, &replaced);
        
        if (!newPkt) {
            /* Allocation of a new packet failed so that we must leave the original packet in place. */
            //DebugLog("Ethernet [IntelMausi]: replaceOrCopyPacket() failed.\n");
            etherStats->dot3RxExtraEntry.resourceErrors++;
            discardPacketFragment();
            goto nextDesc;
        }
        
        /* If the packet was replaced we have to update the descriptor's buffer address. */
        if (replaced) {
            n = rxMbufCursor->getPhysicalSegments(bufPkt, &rxSegment, 1);

            if ((n != 1) || (rxSegment.location & 0x07ff)) {
                DebugLog("Ethernet [IntelMausi]: getPhysicalSegments() failed.\n");
                etherStats->dot3RxExtraEntry.resourceErrors++;
                freePacket(bufPkt);
                discardPacketFragment();
                goto nextDesc;
            }
            addr = rxSegment.location;
            rxBufArray[rxNextDescIndex].mbuf = bufPkt;
            rxBufArray[rxNextDescIndex].phyAddr = addr;
        }
        /* Set the length of the buffer. */
        mbuf_setlen(newPkt, pktSize);

        if (status & E1000_RXD_STAT_EOP) {
            if (rxPacketHead) {
                /* This is the last buffer of a jumbo frame. */
                mbuf_setflags_mask(newPkt, 0, MBUF_PKTHDR);
                mbuf_setnext(rxPacketTail, newPkt);
                
                rxPacketSize += pktSize;
                rxPacketTail = newPkt;
            } else {
                /*
                 * We've got a complete packet in one buffer.
                 * It can be enqueued directly.
                 */
                rxPacketHead = newPkt;
                rxPacketSize = pktSize;
            }
            intelGetChecksumResult(rxPacketHead, status);

            /* Also get the VLAN tag if there is any. */
            if (vlanTag)
                setVlanTag(rxPacketHead, vlanTag);

            mbuf_pkthdr_setlen(rxPacketHead, rxPacketSize);
            interface->enqueueInputPacket(rxPacketHead, pollQueue);
            
            rxPacketHead = rxPacketTail = NULL;
            rxPacketSize = 0;

            goodPkts++;
        } else {
            if (rxPacketHead) {
                /* We are in the middle of a jumbo frame. */
                mbuf_setflags_mask(newPkt, 0, MBUF_PKTHDR);
                mbuf_setnext(rxPacketTail, newPkt);

                rxPacketTail = newPkt;
                rxPacketSize += pktSize;
            } else {
                /* This is the first buffer of a jumbo frame. */
                rxPacketHead = rxPacketTail = newPkt;
                rxPacketSize = pktSize;
            }
        }
        
        /* Finally update the descriptor and get the next one to examine. */
    nextDesc:
        desc->read.buffer_addr = OSSwapHostToLittleInt64(addr);
        desc->read.reserved = 0;
        
        ++rxNextDescIndex &= kRxDescMask;
        desc = &rxDescArray[rxNextDescIndex];
        rxCleanedCount++;
    }
    if (rxCleanedCount >= E1000_RX_BUFFER_WRITE) {
        /*
         * Prevent the tail from reaching the head in order to avoid a false
         * buffer queue full condition.
         */
        if (adapterData.flags2 & FLAG2_PCIM2PCI_ARBITER_WA)
            intelUpdateRxDescTail((rxNextDescIndex - 1) & kRxDescMask);
        else
            intelWriteMem32(E1000_RDT(0), (rxNextDescIndex - 1) & kRxDescMask);
        
        rxCleanedCount = 0;
    }
    return goodPkts;
}

void IntelMausi::checkLinkStatus()
{
	struct e1000_hw *hw = &adapterData.hw;
    bool link;
    
    hw->mac.get_link_status = true;

    /* ICH8 workaround-- Call gig speed drop workaround on cable
     * disconnect (LSC) before accessing any PHY registers
     */
    if ((adapterData.flags & FLAG_LSC_GIG_SPEED_DROP) && (!(intelReadMem32(E1000_STATUS) & E1000_STATUS_LU))) {
        if (!test_bit(__E1000_DOWN, &adapterData.state))
            e1000e_gig_downshift_workaround_ich8lan(hw);
    }
    /* Now check the link state. */
    link = intelCheckLink(&adapterData);
    
    DebugLog("Ethernet [IntelMausi]: checkLinkStatus() returned %u.\n", link);

    if (linkUp) {
        if (link) {
            /* The link partner must have changed some setting. Initiate renegotiation
             * of the link parameters to make sure that the MAC is programmed correctly.
             */
            timerSource->cancelTimeout();
            updateStatistics(&adapterData);
            intelRestart();
        } else {
            /* Stop watchdog and statistics updates. */
            timerSource->cancelTimeout();
            setLinkDown();
        }
    } else {
        if (link) {
            /* Start rx/tx and inform upper layers that the link is up now. */
            setLinkUp();
            timerSource->setTimeoutMS(kTimeoutMS);
        }
    }
}

void IntelMausi::interruptOccurred(OSObject *client, IOInterruptEventSource *src, int count)
{
	struct e1000_hw *hw = &adapterData.hw;
    UInt32 icr = intelReadMem32(E1000_ICR); /* read ICR disables interrupts using IAM */

    UInt32 packets;

    if (!polling) {
        if (icr & (E1000_ICR_TXDW | E1000_ICR_TXQ0)) {
            txInterrupt();
            etherStats->dot3TxExtraEntry.interrupts++;
        }

        if (icr & (E1000_ICR_RXQ0 | E1000_ICR_RXT0 | E1000_ICR_RXDMT0)) {
            packets = rxInterrupt(netif, kNumRxDesc, NULL, NULL);
            etherStats->dot3RxExtraEntry.interrupts++;

            if (packets)
                netif->flushInputQueue();
        }
    }
	/* Reset on uncorrectable ECC error */
    if ((icr & E1000_ICR_ECCER) && (hw->mac.type >= e1000_pch_lpt)) {
        UInt32 pbeccsts = intelReadMem32(E1000_PBECCSTS);

        etherStats->dot3StatsEntry.internalMacReceiveErrors += (pbeccsts & E1000_PBECCSTS_UNCORR_ERR_CNT_MASK) >>E1000_PBECCSTS_UNCORR_ERR_CNT_SHIFT;
        etherStats->dot3TxExtraEntry.resets++;

        IOLog("Ethernet [IntelMausi]: Uncorrectable ECC error. Reseting chip.\n");
		intelRestart();
        return;
	}
	if (icr & (E1000_ICR_LSC | E1000_IMS_RXSEQ)) {
        checkLinkStatus();
	}
    /* Reenable interrupts by setting the bits in the mask register. */
    intelWriteMem32(E1000_IMS, icr);
}

#pragma mark --- rx poll methods ---

IOReturn IntelMausi::setInputPacketPollingEnable(IONetworkInterface *interface, bool enabled)
{
    //DebugLog("setInputPacketPollingEnable() ===>\n");
    
    if (isEnabled) {
        if (enabled) {
            intelWriteMem32(E1000_IMC, ~(E1000_ICR_LSC | E1000_IMS_RXSEQ));
            intelFlush();
        } else {
            intelEnableIRQ(intrMask);
        }
        polling = enabled;
    }
    //DebugLog("input polling %s.\n", enabled ? "enabled" : "disabled");
    
    //DebugLog("setInputPacketPollingEnable() <===\n");
    
    return kIOReturnSuccess;
}

void IntelMausi::pollInputPackets(IONetworkInterface *interface, uint32_t maxCount, IOMbufQueue *pollQueue, void *context )
{
    //DebugLog("pollInputPackets() ===>\n");
    
    if (polling) {
        rxInterrupt(interface, maxCount, pollQueue, context);
        
        /* Finally cleanup the transmitter ring. */
        txInterrupt();
    }
    
    //DebugLog("pollInputPackets() <===\n");
}

#pragma mark --- hardware specific methods ---

void IntelMausi::setLinkUp()
{
    struct e1000_hw *hw = &adapterData.hw;
	struct e1000_phy_info *phy = &hw->phy;
    const char *flowName;
    const char *speedName;
    const char *duplexName;
    const char *eeeName;
    UInt64 mediumSpeed;
    UInt32 mediumIndex = MEDIUM_INDEX_AUTO;
    UInt32 fcIndex;
    UInt32 tctl, rctl, ctrl;
    
    eeeMode = 0;
    eeeName = eeeNames[kEEETypeNo];
    
    /* update snapshot of PHY registers on LSC */
    intelPhyReadStatus(&adapterData);
    hw->mac.ops.get_link_up_info(hw, &adapterData.link_speed, &adapterData.link_duplex);
    
    /* check if SmartSpeed worked */
    e1000e_check_downshift(hw);
    
    if (phy->speed_downgraded)
        IOLog("Ethernet [IntelMausi]: Link Speed was downgraded by SmartSpeed\n");
    
    /* On supported PHYs, check for duplex mismatch only
     * if link has autonegotiated at 10/100 half
     */
    if ((hw->phy.type == e1000_phy_igp_3 || hw->phy.type == e1000_phy_bm) &&
        hw->mac.autoneg && (adapterData.link_speed == SPEED_10 || adapterData.link_speed == SPEED_100) &&
        (adapterData.link_duplex == HALF_DUPLEX)) {
        UInt16 autoneg_exp;
        
        e1e_rphy(hw, MII_EXPANSION, &autoneg_exp);
        
        if (!(autoneg_exp & EXPANSION_NWAY))
            IOLog("Ethernet [IntelMausi]: Autonegotiated half duplex but link partner cannot autoneg.  Try forcing full duplex if link gets many collisions.\n");
    }
    /* Enable transmits in the hardware. */
    tctl = intelReadMem32(E1000_TCTL);
    tctl |= E1000_TCTL_EN;
    intelWriteMem32(E1000_TCTL, tctl);

    /* Enable the receiver too. */
    rctl = intelReadMem32(E1000_RCTL);
    rctl |= E1000_RCTL_EN;
    intelWriteMem32(E1000_RCTL, rctl);

    /* Perform any post-link-up configuration before
     * reporting link up.
     */
    if (phy->ops.cfg_on_link_up)
        phy->ops.cfg_on_link_up(hw);

    intelEnableIRQ(intrMask);
    
    /* Get link speed, duplex and flow-control mode. */
    ctrl = intelReadMem32(E1000_CTRL) & (E1000_CTRL_RFCE | E1000_CTRL_TFCE);
    
    switch (ctrl) {
        case (E1000_CTRL_RFCE | E1000_CTRL_TFCE):
            fcIndex = kFlowControlTypeRxTx;
            break;
            
        case E1000_CTRL_RFCE:
            fcIndex = kFlowControlTypeRx;
            break;
            
        case E1000_CTRL_TFCE:
            fcIndex = kFlowControlTypeTx;
            break;
            
        default:
            fcIndex = kFlowControlTypeNone;
            break;
    }
    flowName = flowControlNames[fcIndex];

    if (adapterData.link_speed == SPEED_1000) {
        mediumSpeed = kSpeed1000MBit;
        speedName = speed1GName;
        duplexName = duplexFullName;

        eeeMode = intelSupportsEEE(&adapterData);
        
        if (fcIndex == kFlowControlTypeNone) {
            if (eeeMode) {
                mediumIndex = MEDIUM_INDEX_1000FDEEE;
                eeeName = eeeNames[kEEETypeYes];
            } else {
                mediumIndex = MEDIUM_INDEX_1000FD;
            }
        } else {
            if (eeeMode) {
                mediumIndex = MEDIUM_INDEX_1000FDFCEEE;
                eeeName = eeeNames[kEEETypeYes];
            } else {
                mediumIndex = MEDIUM_INDEX_1000FDFC;
            }
        }
    } else if (adapterData.link_speed == SPEED_100) {
        mediumSpeed = kSpeed100MBit;
        speedName = speed100MName;
        
        if (adapterData.link_duplex != DUPLEX_FULL) {
            duplexName = duplexFullName;

            eeeMode = intelSupportsEEE(&adapterData);

            if (fcIndex == kFlowControlTypeNone) {
                if (eeeMode) {
                    mediumIndex = MEDIUM_INDEX_100FDEEE;
                    eeeName = eeeNames[kEEETypeYes];
                } else {
                    mediumIndex = MEDIUM_INDEX_100FD;
                }
            } else {
                if (eeeMode) {
                    mediumIndex = MEDIUM_INDEX_100FDFCEEE;
                    eeeName = eeeNames[kEEETypeYes];
                } else {
                    mediumIndex = MEDIUM_INDEX_100FDFC;
                }
            }
        } else {
            mediumIndex = MEDIUM_INDEX_100HD;
            duplexName = duplexHalfName;
        }
    } else {
        mediumSpeed = kSpeed10MBit;
        speedName = speed10MName;
        
        if (adapterData.link_duplex != DUPLEX_FULL) {
            mediumIndex = MEDIUM_INDEX_10FD;
            duplexName = duplexFullName;
        } else {
            mediumIndex = MEDIUM_INDEX_10HD;
            duplexName = duplexHalfName;
        }
    }
    linkUp = true;
    
    setLinkStatus((kIONetworkLinkValid | kIONetworkLinkActive | linkOpts), mediumTable[mediumIndex], mediumSpeed, NULL);
    linkOpts = 0;

    /* Update poll params according to link speed. */
    bzero(&pollParams, sizeof(IONetworkPacketPollingParameters));
    
    if (adapterData.link_speed == SPEED_10) {
        pollParams.lowThresholdPackets = 2;
        pollParams.highThresholdPackets = 8;
        pollParams.lowThresholdBytes = 0x400;
        pollParams.highThresholdBytes = 0x1800;
        pollParams.pollIntervalTime = 1000000;  /* 1ms */
    } else {
        pollParams.lowThresholdPackets = 10;
        pollParams.highThresholdPackets = 40;
        pollParams.lowThresholdBytes = 0x1000;
        pollParams.highThresholdBytes = 0x10000;
        pollParams.pollIntervalTime = (adapterData.link_speed == SPEED_1000) ? 170000 : 1000000;  /* 170µs / 1ms */
    }
    netif->setPacketPollingParameters(&pollParams, 0);
    DebugLog("Ethernet [IntelMausi]: pollIntervalTime: %lluus\n", (pollParams.pollIntervalTime / 1000));

    /* Start output thread, statistics update and watchdog. */
    netif->startOutputThread();
    IOLog("Ethernet [IntelMausi]: Link up on en%u, %s, %s, %s%s\n", netif->getUnitNumber(), speedName, duplexName, flowName, eeeName);

    if (chipType >= board_pch_lpt)
        setMaxLatency(adapterData.link_speed);
    
    DebugLog("Ethernet [IntelMausi]: CTRL=0x%08x\n", intelReadMem32(E1000_CTRL));
    DebugLog("Ethernet [IntelMausi]: CTRL_EXT=0x%08x\n", intelReadMem32(E1000_CTRL_EXT));
    DebugLog("Ethernet [IntelMausi]: STATUS=0x%08x\n", intelReadMem32(E1000_STATUS));
    DebugLog("Ethernet [IntelMausi]: RCTL=0x%08x\n", intelReadMem32(E1000_RCTL));
    DebugLog("Ethernet [IntelMausi]: PSRCTL=0x%08x\n", intelReadMem32(E1000_PSRCTL));
    DebugLog("Ethernet [IntelMausi]: FCRTL=0x%08x\n", intelReadMem32(E1000_FCRTL));
    DebugLog("Ethernet [IntelMausi]: FCRTH=0x%08x\n", intelReadMem32(E1000_FCRTH));
    DebugLog("Ethernet [IntelMausi]: RDLEN(0)=0x%08x\n", intelReadMem32(E1000_RDLEN(0)));
    DebugLog("Ethernet [IntelMausi]: RDTR=0x%08x\n", intelReadMem32(E1000_RDTR));
    DebugLog("Ethernet [IntelMausi]: RADV=0x%08x\n", intelReadMem32(E1000_RADV));
    DebugLog("Ethernet [IntelMausi]: RXCSUM=0x%08x\n", intelReadMem32(E1000_RXCSUM));
    DebugLog("Ethernet [IntelMausi]: RFCTL=0x%08x\n", intelReadMem32(E1000_RFCTL));
    DebugLog("Ethernet [IntelMausi]: RXDCTL(0)=0x%08x\n", intelReadMem32(E1000_RXDCTL(0)));
    DebugLog("Ethernet [IntelMausi]: RAL(0)=0x%08x\n", intelReadMem32(E1000_RAL(0)));
    DebugLog("Ethernet [IntelMausi]: RAH(0)=0x%08x\n", intelReadMem32(E1000_RAH(0)));
    DebugLog("Ethernet [IntelMausi]: MRQC=0x%08x\n", intelReadMem32(E1000_MRQC));
    DebugLog("Ethernet [IntelMausi]: TARC(0)=0x%08x\n", intelReadMem32(E1000_TARC(0)));
    DebugLog("Ethernet [IntelMausi]: TARC(1)=0x%08x\n", intelReadMem32(E1000_TARC(1)));
    DebugLog("Ethernet [IntelMausi]: TCTL=0x%08x\n", intelReadMem32(E1000_TCTL));
    DebugLog("Ethernet [IntelMausi]: TXDCTL(0)=0x%08x\n", intelReadMem32(E1000_TXDCTL(0)));
    DebugLog("Ethernet [IntelMausi]: TXDCTL(1)=0x%08x\n", intelReadMem32(E1000_TXDCTL(1)));
    DebugLog("Ethernet [IntelMausi]: TADV=0x%08x\n", intelReadMem32(E1000_TADV));
    DebugLog("Ethernet [IntelMausi]: TIDV=0x%08x\n", intelReadMem32(E1000_TIDV));
    DebugLog("Ethernet [IntelMausi]: MANC=0x%08x\n", intelReadMem32(E1000_MANC));
    DebugLog("Ethernet [IntelMausi]: MANC2H=0x%08x\n", intelReadMem32(E1000_MANC2H));
    DebugLog("Ethernet [IntelMausi]: LTRV=0x%08x\n", intelReadMem32(E1000_LTRV));
    DebugLog("Ethernet [IntelMausi]: FWSM=0x%08x\n", intelReadMem32(E1000_FWSM));
    DebugLog("Ethernet [IntelMausi]: SWSM2=0x%08x\n", intelReadMem32(E1000_SWSM2));
}

void IntelMausi::setLinkDown()
{
    deadlockWarn = 0;
    
    /* Stop output thread and flush output queue. */
    netif->stopOutputThread();
    netif->flushOutputQueue();
    
    /* Update link status. */
    linkUp = false;
    setLinkStatus(kIONetworkLinkValid);
    
    intelDown(&adapterData, true);
    intelConfigure(&adapterData);
    clear_bit(__E1000_DOWN, &adapterData.state);
	intelEnableIRQ(intrMask);

    IOLog("Ethernet [IntelMausi]: Link down on en%u\n", netif->getUnitNumber());
}

inline void IntelMausi::intelGetChecksumResult(mbuf_t m, UInt32 status)
{
    if (!(status & (E1000_RXDEXT_STATERR_IPE | E1000_RXDEXT_STATERR_TCPE))) {
        mbuf_csum_performed_flags_t performed = 0;
        UInt32 value = 0;

        if (status & E1000_RXD_STAT_IPPCS)
            performed |= (MBUF_CSUM_DID_IP | MBUF_CSUM_IP_GOOD);
        
        if (status & (E1000_RXD_STAT_TCPCS | E1000_RXD_STAT_UDPCS)) {
            performed |= (MBUF_CSUM_DID_DATA | MBUF_CSUM_PSEUDO_HDR);
            value = 0xffff; // fake a valid checksum value
        }
        if (performed)
            mbuf_set_csum_performed(m, performed, value);
    }
}

bool IntelMausi::intelIdentifyChip()
{
    const struct e1000_info *ei;
    UInt32 i = 0;
    UInt16 id = deviceTable[i].pciDevId;
    bool result = false;
    
    while (id) {
        if (id == pciDeviceData.device) {
            chip = i;
            chipType = deviceTable[i].device;
            ei = deviceTable[i].deviceInfo;
            adapterData.ei = ei;
            adapterData.pba = ei->pba;
            adapterData.flags = ei->flags;
            adapterData.flags2 = ei->flags2;
            adapterData.hw.adapter = &adapterData;
            adapterData.hw.mac.type = ei->mac;
            adapterData.max_hw_frame_size = ei->max_hw_frame_size;
            adapterData.bd_number = 0;
            
            /* Set default EEE advertisement */
            if (adapterData.flags2 & FLAG2_HAS_EEE)
                adapterData.eee_advert = MDIO_EEE_100TX | MDIO_EEE_1000T;
            
            result = true;
            break;
        }
        id = deviceTable[++i].pciDevId;
    }
    
done:
    return result;
}

bool IntelMausi::intelStart()
{
    struct e1000_hw *hw = &adapterData.hw;
    const struct e1000_info *ei = adapterData.ei;
    struct e1000_mac_info *mac = &hw->mac;
    SInt32 rval = 0;
    UInt16 eepromData = 0;
	UInt16 eepromApmeMask = E1000_EEPROM_APME;
    int error, i;
    bool result = false;
    
    /* Setup some default values. */
    adapterData.rx_buffer_len = kRxBufferPktSize;
	adapterData.max_frame_size = mtu + ETH_HLEN + ETH_FCS_LEN;
	adapterData.min_frame_size = ETH_ZLEN + ETH_FCS_LEN;
    
    adapterData.rx_int_delay = 0;
    adapterData.rx_abs_int_delay = 0;

    adapterData.tx_int_delay = 0x1c;
    adapterData.tx_abs_int_delay = 0x1c;

    if ((adapterData.flags & FLAG_HAS_SMART_POWER_DOWN))
        adapterData.flags |= FLAG_SMART_POWER_DOWN;

    adapterData.flags2 |= (FLAG2_CRC_STRIPPING | FLAG2_DFLT_CRC_STRIPPING);
    adapterData.flags |= FLAG_READ_ONLY_NVM;
    
    if (adapterData.flags2 & FLAG2_HAS_EEE)
        hw->dev_spec.ich8lan.eee_disable = false;

    linkOpts = 0;

    initPCIPowerManagment(pciDevice, ei);
    
	/* Explicitly disable IRQ since the NIC can be in any state. */
	intelDisableIRQ();
    
	set_bit(__E1000_DOWN, &adapterData.state);
    
    memcpy(&hw->mac.ops, ei->mac_ops, sizeof(hw->mac.ops));
	memcpy(&hw->nvm.ops, ei->nvm_ops, sizeof(hw->nvm.ops));
	memcpy(&hw->phy.ops, ei->phy_ops, sizeof(hw->phy.ops));
    
    if (hw->mac.type == e1000_ich8lan)
        e1000e_set_kmrn_lock_loss_workaround_ich8lan(hw, true);
    
	hw->phy.autoneg_wait_to_complete = 0;
    
    error = ei->get_variants(&adapterData);
    
	if (error) {
        IOLog("Ethernet [IntelMausi]: Failed to get adapter data with error %d.\n", error);
		goto done;
    }
	if ((adapterData.flags & FLAG_IS_ICH) &&
	    (adapterData.flags & FLAG_READ_ONLY_NVM) &&
        (hw->mac.type < e1000_pch_spt))
		e1000e_write_protect_nvm_ich8lan(hw);
    
	hw->phy.autoneg_wait_to_complete = 0;
    
	/* Copper options */
	if (hw->phy.media_type == e1000_media_type_copper) {
		hw->phy.mdix = AUTO_ALL_MODES;
		hw->phy.disable_polarity_correction = 0;
		hw->phy.ms_type = e1000_ms_hw_default;
	}
	if (hw->phy.ops.check_reset_block && hw->phy.ops.check_reset_block(hw))
		IOLog("Ethernet [IntelMausi]: PHY reset is blocked due to SOL/IDER session.\n");

    if (intelEnableMngPassThru(hw))
		adapterData.flags |= FLAG_MNG_PT_ENABLED;

	/* before reading the NVM, reset the controller to
	 * put the device in a known good starting state
	 */
	hw->mac.ops.reset_hw(hw);
    
	/* systems with ASPM and others may see the checksum fail on the first
	 * attempt. Let's give it a few tries
	 */
	for (i = 0;; i++) {
		if (e1000_validate_nvm_checksum(hw) >= 0)
			break;
        
		if (i == 2) {
			IOLog("Ethernet [IntelMausi]: The NVM Checksum Is Not Valid.\n");
			goto error_eeprom;
            break;
		}
	}
	//intelEEPROMChecks(&adapterData);
    
	/* copy the MAC address */
	if (e1000e_read_mac_addr(hw))
		IOLog("Ethernet [IntelMausi]: NVM Read Error while reading MAC address.\n");
    
	if (!is_valid_ether_addr(mac->addr)) {
		IOLog("Ethernet [IntelMausi]: Invalid MAC Address: %pM\n", mac->addr);
		goto error_eeprom;
	}
	/* Initialize link parameters. User can change them with ethtool */
	hw->mac.autoneg = 1;
	adapterData.fc_autoneg = true;
	hw->fc.requested_mode = e1000_fc_default;
	hw->fc.current_mode = e1000_fc_default;
	hw->phy.autoneg_advertised = 0x2f;
    
	/* Initial Wake on LAN setting - If APM wake is enabled in
	 * the EEPROM, enable the ACPI Magic Packet filter
	 */
	if (adapterData.flags & FLAG_APME_IN_WUC) {
		/* APME bit in EEPROM is mapped to WUC.APME */
		eepromData = intelReadMem32(E1000_WUC);
		eepromApmeMask = E1000_WUC_APME;
        
		if ((hw->mac.type > e1000_ich10lan) &&
		    (eepromData & E1000_WUC_PHY_WAKE))
			adapterData.flags2 |= FLAG2_HAS_PHY_WAKEUP;
	} else if (adapterData.flags & FLAG_APME_IN_CTRL3) {
		if (adapterData.flags & FLAG_APME_CHECK_PORT_B && (adapterData.hw.bus.func == 1))
			rval = e1000_read_nvm(hw,  NVM_INIT_CONTROL3_PORT_B, 1, &eepromData);
		else
			rval = e1000_read_nvm(hw, NVM_INIT_CONTROL3_PORT_A, 1, &eepromData);
	}
    
	/* fetch WoL from EEPROM */
	if (rval)
		DebugLog("Ethernet [IntelMausi]: NVM read error getting WoL initial values: %d\n", rval);
	else if (eepromData & eepromApmeMask)
		adapterData.eeprom_wol |= E1000_WUFC_MAG;
    
	/* now that we have the eeprom settings, apply the special cases
	 * where the eeprom may be wrong or the board simply won't support
	 * wake on lan on a particular port
	 */
	if (!(adapterData.flags & FLAG_HAS_WOL))
		adapterData.eeprom_wol = 0;
    
	/* initialize the wol settings based on the eeprom settings */
	adapterData.wol = adapterData.eeprom_wol;
    
	/* make sure adapter isn't asleep if manageability is enabled */
	if (adapterData.wol || (adapterData.flags & FLAG_MNG_PT_ENABLED) || (hw->mac.ops.check_mng_mode(hw)))
        pciDevice->enablePCIPowerManagement(kPCIPMCSPowerStateD0);
    
	/* save off EEPROM version number */
	rval = e1000_read_nvm(hw, 5, 1, &adapterData.eeprom_vers);
    
	if (rval) {
		DebugLog("Ethernet [IntelMausi]: NVM read error getting EEPROM version: %d\n", rval);
		adapterData.eeprom_vers = 0;
	}
	/* reset the hardware with the new settings */
	intelReset(&adapterData);
    
	/* Let the f/w know that the h/w is now under the control of the driver
     * even in case the device has AMT in order to avoid problems after wakeup.
	 */
	e1000e_get_hw_control(&adapterData);
        
    intrMask = IMS_ENABLE_MASK;
    
    if ((hw->mac.type == e1000_pch_lpt) || (hw->mac.type == e1000_pch_spt)) {
        intrMask |= E1000_IMS_ECCER;
    }
    IOLog("Ethernet [IntelMausi]: %s (Rev. %u), %02x:%02x:%02x:%02x:%02x:%02x\n",
          deviceTable[chip].deviceName, pciDeviceData.revision,
          mac->addr[0], mac->addr[1], mac->addr[2], mac->addr[3], mac->addr[4], mac->addr[5]);
    result = true;
    
done:
    return result;
    
error_eeprom:
	if (hw->phy.ops.check_reset_block && !hw->phy.ops.check_reset_block(hw))
		e1000_phy_hw_reset(hw);
    
    goto done;
}

#pragma mark --- timer action methods ---

void IntelMausi::timerAction(IOTimerEventSource *timer)
{
    struct e1000_hw *hw = &adapterData.hw;

    if (!linkUp) {
        DebugLog("Ethernet [IntelMausi]: Timer fired while link down.\n");
        goto done;
    }
    intelUpdateAdaptive(&adapterData.hw);
    
    /* Check for tx deadlock. */
    if (checkForDeadlock())
        goto done;
    
    /* Enable EEE on 82579 after link up. */
    if (eeeMode) {
        e1000_get_phy_info(hw);
        
        if (hw->phy.type >= e1000_phy_82579)
            intelEnableEEE(hw, eeeMode);

        eeeMode = 0;
    }
    updateStatistics(&adapterData);
    timerSource->setTimeoutMS(kTimeoutMS);
    
done:
    txDescDoneLast = txDescDoneCount;
    
    //DebugLog("timerAction() <===\n");
}

void IntelMausi::updateStatistics(struct e1000_adapter *adapter)
{
    struct e1000_hw *hw = &adapter->hw;
    
	adapter->stats.crcerrs += intelReadMem32(E1000_CRCERRS);
	adapter->stats.gprc += intelReadMem32(E1000_GPRC);
	adapter->stats.gorc += intelReadMem32(E1000_GORCL);
	intelReadMem32(E1000_GORCH);		/* Clear gorc */
	adapter->stats.bprc += intelReadMem32(E1000_BPRC);
	adapter->stats.mprc += intelReadMem32(E1000_MPRC);
	adapter->stats.roc += intelReadMem32(E1000_ROC);
    
	adapter->stats.mpc += intelReadMem32(E1000_MPC);
    
	/* Half-duplex statistics */
	if (adapter->link_duplex == HALF_DUPLEX) {
		if (adapter->flags2 & FLAG2_HAS_PHY_STATS) {
			e1000e_update_phy_stats(adapter);
		} else {
			adapter->stats.scc += intelReadMem32(E1000_SCC);
			adapter->stats.ecol += intelReadMem32(E1000_ECOL);
			adapter->stats.mcc += intelReadMem32(E1000_MCC);
			adapter->stats.latecol += intelReadMem32(E1000_LATECOL);
			adapter->stats.dc += intelReadMem32(E1000_DC);
            
			hw->mac.collision_delta = intelReadMem32(E1000_COLC);
            
			if ((hw->mac.type != e1000_82574) &&
			    (hw->mac.type != e1000_82583))
				adapter->stats.tncrs += intelReadMem32(E1000_TNCRS);
		}
		adapter->stats.colc += hw->mac.collision_delta;
	}
    
	adapter->stats.xonrxc += intelReadMem32(E1000_XONRXC);
	adapter->stats.xontxc += intelReadMem32(E1000_XONTXC);
	adapter->stats.xoffrxc += intelReadMem32(E1000_XOFFRXC);
	adapter->stats.xofftxc += intelReadMem32(E1000_XOFFTXC);
	adapter->stats.gptc += intelReadMem32(E1000_GPTC);
	adapter->stats.gotc += intelReadMem32(E1000_GOTCL);
	intelReadMem32(E1000_GOTCH);		/* Clear gotc */
	adapter->stats.rnbc += intelReadMem32(E1000_RNBC);
	adapter->stats.ruc += intelReadMem32(E1000_RUC);
    
	adapter->stats.mptc += intelReadMem32(E1000_MPTC);
	adapter->stats.bptc += intelReadMem32(E1000_BPTC);
    
	/* used for adaptive IFS */
    
	hw->mac.tx_packet_delta = intelReadMem32(E1000_TPT);
	adapter->stats.tpt += hw->mac.tx_packet_delta;
    
	adapter->stats.algnerrc += intelReadMem32(E1000_ALGNERRC);
	adapter->stats.rxerrc += intelReadMem32(E1000_RXERRC);
	adapter->stats.cexterr += intelReadMem32(E1000_CEXTERR);
	adapter->stats.tsctc += intelReadMem32(E1000_TSCTC);
	adapter->stats.tsctfc += intelReadMem32(E1000_TSCTFC);
    
    netStats->inputPackets = (UInt32)adapter->stats.gprc;
    netStats->inputErrors = (UInt32)(adapter->stats.rxerrc + adapter->stats.crcerrs
        + adapter->stats.algnerrc + adapter->stats.ruc + adapter->stats.roc
        + adapter->stats.cexterr);
    netStats->outputPackets = (UInt32)adapter->stats.gptc;
    netStats->outputErrors = (UInt32)(adapter->stats.ecol + adapter->stats.latecol);
    netStats->collisions = (UInt32)adapter->stats.colc;

    etherStats->dot3StatsEntry.alignmentErrors = (UInt32)adapter->stats.algnerrc;
    etherStats->dot3StatsEntry.fcsErrors = (UInt32)adapter->stats.crcerrs;
    etherStats->dot3StatsEntry.singleCollisionFrames = (UInt32)adapter->stats.scc;
    etherStats->dot3StatsEntry.multipleCollisionFrames = (UInt32)adapter->stats.mcc;
    etherStats->dot3StatsEntry.deferredTransmissions = (UInt32)adapter->stats.dc ;
    etherStats->dot3StatsEntry.lateCollisions = (UInt32)adapter->stats.latecol;
    etherStats->dot3StatsEntry.excessiveCollisions = (UInt32)adapter->stats.ecol;
    etherStats->dot3StatsEntry.carrierSenseErrors = (UInt32)adapter->stats.cexterr;
    etherStats->dot3StatsEntry.frameTooLongs = (UInt32)adapter->stats.roc;
    etherStats->dot3StatsEntry.missedFrames = (UInt32)adapter->stats.mpc;
    
    etherStats->dot3RxExtraEntry.frameTooShorts = (UInt32)adapter->stats.ruc;
}

bool IntelMausi::checkForDeadlock()
{
    bool deadlock = false;
    
    if (forceReset) {
        etherStats->dot3TxExtraEntry.resets++;
        intelRestart();
        deadlock = true;
        eeeMode = 0;
    }
    
    if ((txDescDoneCount == txDescDoneLast) && (txNumFreeDesc < kNumTxDesc)) {
        if (++deadlockWarn >= kTxDeadlockTreshhold) {
            mbuf_t m = txBufArray[txDirtyIndex].mbuf;
            UInt32 pktSize;
            UInt16 index;
            
#ifdef DEBUG
            UInt16 i;
            UInt16 stalledIndex = txDirtyIndex;
#endif
            //UInt8 data;
            
            IOLog("Ethernet [IntelMausi]: Tx stalled? Resetting chipset. txDirtyDescIndex=%u, STATUS=0x%08x, TCTL=0x%08x.\n", txDirtyIndex, intelReadMem32(E1000_STATUS), intelReadMem32(E1000_TCTL));

#ifdef DEBUG
            for (i = 0; i < 30; i++) {
                index = ((stalledIndex - 20 + i) & kTxDescMask);

                IOLog("Ethernet [IntelMausi]: desc[%u]: lower=0x%08x, upper=0x%08x, addr=0x%016llx, mbuf=0x%016llx, len=%u.\n", index, txDescArray[index].lower.data, txDescArray[index].upper.data, txDescArray[index].buffer_addr, (UInt64)txBufArray[index].mbuf, txBufArray[index].pad);
            }
#endif
            if (m) {
                pktSize = (UInt32)mbuf_pkthdr_len(m);
                IOLog("Ethernet [IntelMausi]: packet size=%u, header size=%u.\n", pktSize, (UInt32)mbuf_len(m));
/*
#ifdef DEBUG
                IOLog("Ethernet [IntelMausi]: MAC-header: ");
                for (i = 0; i < 14; i++) {
                    mbuf_copydata(m, i, 1, &data);
                    IOLog(" 0x%02x", data);
                }
                IOLog("\n");
                
                IOLog("Ethernet [IntelMausi]: IP-header: ");
                for (i = 14; i < 34; i++) {
                    mbuf_copydata(m, i, 1, &data);
                    IOLog(" 0x%02x", data);
                }
                IOLog("\n");
                
                IOLog("Ethernet [IntelMausi]: TCP-Header / Data: ");
                for (i = 34; i < 100; i++) {
                    mbuf_copydata(m, i, 1, &data);
                    IOLog(" 0x%02x", data);
                }
                IOLog("\n");
#endif
*/
            }
            etherStats->dot3TxExtraEntry.resets++;
            intelRestart();
            deadlock = true;
        } else {
            DebugLog("Ethernet [IntelMausi]: Check tx ring for progress. txNumFreeDesc=%u\n", txNumFreeDesc);
            /* Flush pending tx descriptors. */
            intelFlushDescriptors();
            /* Check the transmitter ring. */
            txInterrupt();
        }
    } else {
        deadlockWarn = 0;
    }
    return deadlock;
}

#pragma mark --- miscellaneous functions ---

static inline void prepareTSO4(mbuf_t m, UInt32 *mssHeaderSize, UInt32 *payloadSize)
{
    struct iphdr *ipHdr = (struct iphdr *)((UInt8 *)mbuf_data(m) + ETHER_HDR_LEN);
    struct tcphdr *tcpHdr = (struct tcphdr *)((UInt8 *)ipHdr + sizeof(struct iphdr));
    UInt16 *addr = (UInt16 *)&ipHdr->saddr;
    UInt32 csum32 = 6;
    UInt32 plen = (UInt32)mbuf_pkthdr_len(m);
    UInt32 hlen = (tcpHdr->th_off << 2) + kMinL4HdrOffsetV4;
    
    ipHdr->tot_len = 0;

    csum32 += ntohs(*addr++);
    csum32 += ntohs(*addr++);
    csum32 += ntohs(*addr++);
    csum32 += ntohs(*addr);
    csum32 += (csum32 >> 16);
    tcpHdr->th_sum = htons((UInt16)csum32);
    
    /* Flush the cache lines in order to enforce writeback. */
    asm volatile ("clflush %0" : "=m" (ipHdr->tot_len));
    asm volatile ("clflush %0" : "=m" (tcpHdr->th_sum));

    *mssHeaderSize = ((*mssHeaderSize << 16) | (hlen << 8));
    *payloadSize = plen - hlen;
}

static inline void prepareTSO6(mbuf_t m, UInt32 *mssHeaderSize, UInt32 *payloadSize)
{
    struct ip6_hdr *ip6Hdr = (struct ip6_hdr *)((UInt8 *)mbuf_data(m) + ETHER_HDR_LEN);
    struct tcphdr *tcpHdr = (struct tcphdr *)((UInt8 *)ip6Hdr + sizeof(struct ip6_hdr));
    UInt16 *addr = (UInt16 *)&ip6Hdr->ip6_src;
    UInt32 csum32 = 6;
    UInt32 plen = (UInt32)mbuf_pkthdr_len(m);
    UInt32 hlen = (tcpHdr->th_off << 2) + kMinL4HdrOffsetV6;
    UInt32 i;
    
    ip6Hdr->ip6_ctlun.ip6_un1.ip6_un1_plen = 0;

    for (i = 0; i < 16; i++)
        csum32 += ntohs(*addr++);
    
    csum32 += (csum32 >> 16);
    tcpHdr->th_sum = htons((UInt16)csum32);
    
    /* Flush the cache lines in order to enforce writeback. */
    asm volatile ("clflush %0" : "=m" (ip6Hdr->ip6_ctlun.ip6_un1.ip6_un1_plen));
    asm volatile ("clflush %0" : "=m" (tcpHdr->th_sum));
    
    *mssHeaderSize = ((*mssHeaderSize << 16) | (hlen << 8));
    *payloadSize = plen - hlen;
}


