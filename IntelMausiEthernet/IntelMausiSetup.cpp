/* IntelMausiSetup.cpp -- IntelMausi driver data structure setup.
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

#pragma mark --- private data ---

static IOMediumType mediumTypeArray[MEDIUM_INDEX_COUNT] = {
    kIOMediumEthernetAuto,
    (kIOMediumEthernet10BaseT | kIOMediumOptionHalfDuplex),
    (kIOMediumEthernet10BaseT | kIOMediumOptionFullDuplex),
    (kIOMediumEthernet100BaseTX | kIOMediumOptionHalfDuplex),
    (kIOMediumEthernet100BaseTX | kIOMediumOptionFullDuplex),
    (kIOMediumEthernet100BaseTX | kIOMediumOptionFullDuplex | kIOMediumOptionFlowControl),
    (kIOMediumEthernet1000BaseT | kIOMediumOptionFullDuplex),
    (kIOMediumEthernet1000BaseT | kIOMediumOptionFullDuplex | kIOMediumOptionFlowControl),
    (kIOMediumEthernet1000BaseT | kIOMediumOptionFullDuplex | kIOMediumOptionEEE),
    (kIOMediumEthernet1000BaseT | kIOMediumOptionFullDuplex | kIOMediumOptionFlowControl | kIOMediumOptionEEE),
    (kIOMediumEthernet100BaseTX | kIOMediumOptionFullDuplex | kIOMediumOptionEEE),
    (kIOMediumEthernet100BaseTX | kIOMediumOptionFullDuplex | kIOMediumOptionFlowControl | kIOMediumOptionEEE)
};

static UInt32 mediumSpeedArray[MEDIUM_INDEX_COUNT] = {
    0,
    10 * MBit,
    10 * MBit,
    100 * MBit,
    100 * MBit,
    100 * MBit,
    1000 * MBit,
    1000 * MBit,
    1000 * MBit,
    1000 * MBit,
    100 * MBit,
    100 * MBit
};

#ifdef DEBUG
static const char *onName = "enabled";
static const char *offName = "disabled";
#endif

#pragma mark --- data structure initialization methods ---

void IntelMausi::getParams()
{
    OSDictionary *params;
    OSString *versionString;
    OSNumber *num;
    OSBoolean *csoV6;
    OSBoolean *wom;
    UInt32 newIntrRate10;
    UInt32 newIntrRate100;
    UInt32 newIntrRate1000;

    versionString = OSDynamicCast(OSString, getProperty(kDriverVersionName));

    params = OSDynamicCast(OSDictionary, getProperty(kParamName));
    
    if (params) {
        csoV6 = OSDynamicCast(OSBoolean, params->getObject(kEnableCSO6Name));
        enableCSO6 = (csoV6) ? csoV6->getValue() : false;
        
        DebugLog("[IntelMausi]: TCP/IPv6 checksum offload %s.\n", enableCSO6 ? onName : offName);
        
        wom = OSDynamicCast(OSBoolean, params->getObject(kEnableWoMName));
        enableWoM = (wom) ? wom->getValue() : false;

        DebugLog("[IntelMausi]: Wake on address match %s.\n", enableWoM ? onName : offName);

        /* Get maximum interrupt rate for 10M. */
        num = OSDynamicCast(OSNumber, params->getObject(kIntrRate10Name));
        newIntrRate10 = 3000;

        if (num)
            newIntrRate10 = num->unsigned32BitValue();

        if (newIntrRate10 < 2500)
            newIntrRate10 = 2500;
        else if (newIntrRate10 > 10000)
            newIntrRate10 = 10000;

        intrThrValue10 = (3906250 / (newIntrRate10 + 1));

        /* Get maximum interrupt rate for 100M. */
        num = OSDynamicCast(OSNumber, params->getObject(kIntrRate100Name));
        newIntrRate100 = 5000;

        if (num)
            newIntrRate100 = num->unsigned32BitValue();

        if (newIntrRate100 < 2500)
            newIntrRate100 = 2500;
        else if (newIntrRate100 > 10000)
            newIntrRate100 = 10000;

        intrThrValue100 = (3906250 / (newIntrRate100 + 1));

        /* Get maximum interrupt rate for 1000M. */
        num = OSDynamicCast(OSNumber, params->getObject(kIntrRate1000Name));
        newIntrRate1000 = 7000;

        if (num)
            newIntrRate1000 = num->unsigned32BitValue();

        if (newIntrRate1000 < 2500)
            newIntrRate1000 = 2500;
        else if (newIntrRate1000 > 10000)
            newIntrRate1000 = 10000;

        intrThrValue1000 = (3906250 / (newIntrRate1000 + 1));

        /* Get rxAbsTime10 from config data */
        num = OSDynamicCast(OSNumber, params->getObject(kRxAbsTime10Name));

        if (num) {
            rxAbsTime10 = num->unsigned32BitValue();

            if (rxAbsTime10 > 500)
                rxAbsTime10 = 0;
        } else {
            rxAbsTime10 = 0;
        }
        /* Get rxAbsTime100 from config data */
        num = OSDynamicCast(OSNumber, params->getObject(kRxAbsTime100Name));

        if (num) {
            rxAbsTime100 = num->unsigned32BitValue();

            if (rxAbsTime100 > 500)
                rxAbsTime100 = 0;
        } else {
            rxAbsTime100 = 0;
        }
        /* Get rxAbsTime1000 from config data */
        num = OSDynamicCast(OSNumber, params->getObject(kRxAbsTime1000Name));

        if (num) {
            rxAbsTime1000 = num->unsigned32BitValue();

            if (rxAbsTime1000 > 500)
                rxAbsTime1000 = 0;
        } else {
            rxAbsTime1000 = 0;
        }

        /* Get rxDelayTime10 from config data */
        num = OSDynamicCast(OSNumber, params->getObject(kRxDelayTime10Name));

        if (num) {
            rxDelayTime10 = num->unsigned32BitValue();

            if (rxDelayTime10 > 100)
                rxDelayTime10 = 0;
        } else {
            rxDelayTime10 = 0;
        }
        /* Get rxDelayTime100 from config data */
        num = OSDynamicCast(OSNumber, params->getObject(kRxDelayTime100Name));

        if (num) {
            rxDelayTime100 = num->unsigned32BitValue();

            if (rxDelayTime100 > 100)
                rxDelayTime100 = 0;
        } else {
            rxDelayTime100 = 0;
        }
        /* Get rxDelayTime1000 from config data */
        num = OSDynamicCast(OSNumber, params->getObject(kRxDelayTime1000Name));

        if (num) {
            rxDelayTime1000 = num->unsigned32BitValue();

            if (rxDelayTime1000 > 100)
                rxDelayTime1000 = 0;
        } else {
            rxDelayTime1000 = 0;
        }
    } else {
        /* Use default values in case of missing config data. */
        enableCSO6 = false;
        enableWoM = false;
        newIntrRate10 = 3000;
        newIntrRate100 = 5000;
        newIntrRate1000 = 7000;
        intrThrValue10 = (3906250 / (newIntrRate10 + 1));
        intrThrValue100 = (3906250 / (newIntrRate100 + 1));
        intrThrValue1000 = (3906250 / (newIntrRate1000 + 1));
        rxAbsTime10 = 0;
        rxAbsTime100 = 0;
        rxAbsTime1000 = 0;
        rxDelayTime10 = 0;
        rxDelayTime100 = 0;
        rxDelayTime1000 = 0;
    }

    DebugLog("[IntelMausi]: rxAbsTime10=%u, rxAbsTime100=%u, rxAbsTime1000=%u, rxDelayTime10=%u, rxDelayTime100=%u, rxDelayTime1000=%u. \n", rxAbsTime10, rxAbsTime100, rxAbsTime1000, rxDelayTime10, rxDelayTime100, rxDelayTime1000);

    if (versionString)
        DebugLog("[IntelMausi]: Version %s using max interrupt rates [%u; %u; %u].\n", versionString->getCStringNoCopy(), newIntrRate10, newIntrRate100, newIntrRate1000);
    else
        DebugLog("[IntelMausi]: Using max interrupt rates [%u; %u; %u].\n", intrThrValue10, intrThrValue100, intrThrValue1000);
}

bool IntelMausi::setupMediumDict()
{
	IONetworkMedium *medium;
    UInt32 count;
    UInt32 i;
    bool result = false;
    
    if (adapterData.hw.phy.media_type == e1000_media_type_fiber) {
        count = 1;
    } else if (adapterData.flags2 & FLAG2_HAS_EEE) {
        count = MEDIUM_INDEX_COUNT;
    } else {
        count = MEDIUM_INDEX_COUNT - 4;
    }
    mediumDict = OSDictionary::withCapacity(count + 1);
    
    if (mediumDict) {
        for (i = MEDIUM_INDEX_AUTO; i < count; i++) {
            medium = IONetworkMedium::medium(mediumTypeArray[i], mediumSpeedArray[i], 0, i);
            
            if (!medium)
                goto error1;
            
            result = IONetworkMedium::addMedium(mediumDict, medium);
            medium->release();
            
            if (!result)
                goto error1;
            
            mediumTable[i] = medium;
        }
    }
    result = publishMediumDictionary(mediumDict);
    
    if (!result)
        goto error1;
    
done:
    return result;
    
error1:
    IOLog("[IntelMausi]: Error creating medium dictionary.\n");
    mediumDict->release();
    
    for (i = MEDIUM_INDEX_AUTO; i < MEDIUM_INDEX_COUNT; i++)
        mediumTable[i] = NULL;
    
    goto done;
}

bool IntelMausi::initEventSources(IOService *provider)
{
    int msiIndex = -1;
    int intrIndex = 0;
    int intrType = 0;
    bool result = false;
    
    txQueue = reinterpret_cast<IOBasicOutputQueue *>(getOutputQueue());
    
    if (txQueue == NULL) {
        IOLog("[IntelMausi]: Failed to get output queue.\n");
        goto done;
    }
    txQueue->retain();
    
    while (pciDevice->getInterruptType(intrIndex, &intrType) == kIOReturnSuccess) {
        if (intrType & kIOInterruptTypePCIMessaged){
            msiIndex = intrIndex;
            break;
        }
        intrIndex++;
    }
    if (msiIndex != -1) {
        DebugLog("[IntelMausi]: MSI interrupt index: %d\n", msiIndex);
        
        interruptSource = IOInterruptEventSource::interruptEventSource(this, OSMemberFunctionCast(IOInterruptEventSource::Action, this, &IntelMausi::interruptOccurred), provider, msiIndex);
    }
    if (!interruptSource) {
        IOLog("[IntelMausi]: MSI interrupt could not be enabled.\n");
        goto error1;
    }
    workLoop->addEventSource(interruptSource);
    
    timerSource = IOTimerEventSource::timerEventSource(this, OSMemberFunctionCast(IOTimerEventSource::Action, this, &IntelMausi::timerAction));
    
    if (!timerSource) {
        IOLog("[IntelMausi]: Failed to create IOTimerEventSource.\n");
        goto error2;
    }
    workLoop->addEventSource(timerSource);
    
    result = true;
    
done:
    return result;
    
error2:
    workLoop->removeEventSource(interruptSource);
    RELEASE(interruptSource);
    
error1:
    IOLog("[IntelMausi]: Error initializing event sources.\n");
    txQueue->release();
    txQueue = NULL;
    goto done;
}

bool IntelMausi::setupDMADescriptors()
{
    IODMACommand::Segment64 seg;
    IOPhysicalSegment rxSegment;
    mbuf_t spareMbuf[kRxNumSpareMbufs];
    mbuf_t m;
    UInt64 offset = 0;
    UInt32 numSegs = 1;
    UInt32 i;
    UInt32 n;
    bool result = false;
    
    /* Create transmitter descriptor array. */
    txBufDesc = IOBufferMemoryDescriptor::inTaskWithPhysicalMask(kernel_task, (kIODirectionInOut | kIOMemoryPhysicallyContiguous | kIOMapInhibitCache), kTxDescSize, 0xFFFFFFFFFFFFF000ULL);
    
    if (!txBufDesc) {
        IOLog("[IntelMausi]: Couldn't alloc txBufDesc.\n");
        goto done;
    }
    if (txBufDesc->prepare() != kIOReturnSuccess) {
        IOLog("[IntelMausi]: txBufDesc->prepare() failed.\n");
        goto error1;
    }
    txDescArray = (struct e1000_data_desc *)txBufDesc->getBytesNoCopy();
    
    /* I don't know if it's really necessary but the documenation says so and Apple's drivers are also doing it this way. */
    txDescDmaCmd = IODMACommand::withSpecification(kIODMACommandOutputHost64, 64, 0, IODMACommand::kMapped, 0, 1);
    
    if (!txDescDmaCmd) {
        IOLog("[IntelMausi]: Couldn't alloc txDescDmaCmd.\n");
        goto error2;
    }
    
    if (txDescDmaCmd->setMemoryDescriptor(txBufDesc) != kIOReturnSuccess) {
        IOLog("[IntelMausi]: setMemoryDescriptor() failed.\n");
        goto error3;
    }
    
    if (txDescDmaCmd->gen64IOVMSegments(&offset, &seg, &numSegs) != kIOReturnSuccess) {
        IOLog("[IntelMausi]: gen64IOVMSegments() failed.\n");
        goto error4;
    }
    /* Now get tx ring's physical address. */
    txPhyAddr = seg.fIOVMAddr;

    /* Initialize txDescArray. */
    bzero(txDescArray, kTxDescSize);
    
    for (i = 0; i < kNumTxDesc; i++) {
        txBufArray[i].mbuf = NULL;
        txBufArray[i].numDescs = 0;
        txBufArray[i].pad = 0;
    }
    txNextDescIndex = txDirtyIndex = txCleanBarrierIndex = 0;
    txNumFreeDesc = kNumTxDesc;
    txMbufCursor = IOMbufNaturalMemoryCursor::withSpecification(0x4000, kMaxSegs);
    
    if (!txMbufCursor) {
        IOLog("[IntelMausi]: Couldn't create txMbufCursor.\n");
        goto error4;
    }
    
    /* Create receiver descriptor array. */
    rxBufDesc = IOBufferMemoryDescriptor::inTaskWithPhysicalMask(kernel_task, (kIODirectionInOut | kIOMemoryPhysicallyContiguous | kIOMapInhibitCache), kRxDescSize, 0xFFFFFFFFFFFFF000ULL);
    
    if (!rxBufDesc) {
        IOLog("[IntelMausi]: Couldn't alloc rxBufDesc.\n");
        goto error5;
    }
    
    if (rxBufDesc->prepare() != kIOReturnSuccess) {
        IOLog("[IntelMausi]: rxBufDesc->prepare() failed.\n");
        goto error6;
    }
    rxDescArray = (union e1000_rx_desc_extended *)rxBufDesc->getBytesNoCopy();
    
    /* I don't know if it's really necessary but the documenation says so and Apple's drivers are also doing it this way. */
    rxDescDmaCmd = IODMACommand::withSpecification(kIODMACommandOutputHost64, 64, 0, IODMACommand::kMapped, 0, 1);
    
    if (!rxDescDmaCmd) {
        IOLog("[IntelMausi]: Couldn't alloc rxDescDmaCmd.\n");
        goto error7;
    }
    
    if (rxDescDmaCmd->setMemoryDescriptor(rxBufDesc) != kIOReturnSuccess) {
        IOLog("[IntelMausi]: setMemoryDescriptor() failed.\n");
        goto error8;
    }
    offset = 0;
    numSegs = 1;
    
    if (rxDescDmaCmd->gen64IOVMSegments(&offset, &seg, &numSegs) != kIOReturnSuccess) {
        IOLog("[IntelMausi]: gen64IOVMSegments() failed.\n");
        goto error9;
    }
    /* And the rx ring's physical address too. */
    rxPhyAddr = seg.fIOVMAddr;
    
    /* Initialize rxDescArray. */
    bzero((void *)rxDescArray, kRxDescSize);
    
    for (i = 0; i < kNumRxDesc; i++) {
        rxBufArray[i].mbuf = NULL;
        rxBufArray[i].phyAddr = 0;
    }
    rxCleanedCount = rxNextDescIndex = 0;
    
    rxMbufCursor = IOMbufNaturalMemoryCursor::withSpecification(PAGE_SIZE, 1);
    
    if (!rxMbufCursor) {
        IOLog("[IntelMausi]: Couldn't create rxMbufCursor.\n");
        goto error9;
    }
    /* Alloc receive buffers. */
    for (i = 0; i < kNumRxDesc; i++) {
        m = allocatePacket(kRxBufferPktSize);
        
        if (!m) {
            IOLog("[IntelMausi]: Couldn't alloc receive buffer.\n");
            goto error10;
        }
        rxBufArray[i].mbuf = m;
        
        n = rxMbufCursor->getPhysicalSegments(m, &rxSegment, 1);
        
        if ((n != 1) || (rxSegment.location & 0x07ff)) {
            IOLog("[IntelMausi]: getPhysicalSegments() for receive buffer failed.\n");
            goto error10;
        }
        /* We have to keep the physical address of the buffer too
         * as descriptor write back overwrites it in the descriptor
         * so that it must be refreshed when the descriptor is
         * prepared for reuse.
         */
        rxBufArray[i].phyAddr = rxSegment.location;
        
        rxDescArray[i].read.buffer_addr = OSSwapHostToLittleInt64(rxSegment.location);
        rxDescArray[i].read.reserved = 0;
    }
    /* Allocate some spare mbufs and free them in order to increase the buffer pool.
     * This seems to avoid the replaceOrCopyPacket() errors under heavy load.
     */
    for (i = 0; i < kRxNumSpareMbufs; i++)
        spareMbuf[i] = allocatePacket(kRxBufferPktSize);
    
    for (i = 0; i < kRxNumSpareMbufs; i++) {
        if (spareMbuf[i])
            freePacket(spareMbuf[i]);
    }
    result = true;
    
done:
    return result;
    
error10:
    for (i = 0; i < kNumRxDesc; i++) {
        if (rxBufArray[i].mbuf) {
            freePacket(rxBufArray[i].mbuf);
            rxBufArray[i].mbuf = NULL;
        }
    }
    RELEASE(rxMbufCursor);
    
error9:
    rxDescDmaCmd->clearMemoryDescriptor();

error8:
    RELEASE(rxDescDmaCmd);

error7:
    rxBufDesc->complete();
    
error6:
    rxBufDesc->release();
    rxDescArray = NULL;
    rxBufDesc = NULL;
    
error5:
    RELEASE(txMbufCursor);
    
error4:
    txDescDmaCmd->clearMemoryDescriptor();
    
error3:
    RELEASE(txDescDmaCmd);
    
error2:
    txBufDesc->complete();
    
error1:
    txBufDesc->release();
    txBufDesc = NULL;
    goto done;
}

void IntelMausi::freeDMADescriptors()
{
    UInt32 i;
    
    if (txBufDesc) {
        txBufDesc->complete();
        txBufDesc->release();
        txBufDesc = NULL;
        txPhyAddr = 0;
    }
    if (txDescDmaCmd) {
        txDescDmaCmd->clearMemoryDescriptor();
        txDescDmaCmd->release();
        txDescDmaCmd = NULL;
    }
    RELEASE(txMbufCursor);
    
    if (rxBufDesc) {
        rxBufDesc->complete();
        rxBufDesc->release();
        rxDescArray = NULL;
        rxBufDesc = NULL;
        rxPhyAddr = 0;
    }
    if (rxDescDmaCmd) {
        rxDescDmaCmd->clearMemoryDescriptor();
        rxDescDmaCmd->release();
        rxDescDmaCmd = NULL;
    }
    RELEASE(rxMbufCursor);
    
    for (i = 0; i < kNumRxDesc; i++) {
        if (rxBufArray[i].mbuf) {
            freePacket(rxBufArray[i].mbuf);
            rxBufArray[i].mbuf = NULL;
        }
    }
}

void IntelMausi::clearDescriptors()
{
    mbuf_t m;
    UInt32 i;
    
    DebugLog("[IntelMausi]: clearDescriptors() ===>\n");
    
    /* First cleanup the tx descriptor ring. */
    for (i = 0; i < kNumTxDesc; i++) {
        m = txBufArray[i].mbuf;
        
        if (m) {
            freePacket(m);
            txBufArray[i].mbuf = NULL;
            txBufArray[i].numDescs = 0;
        }
    }
    txNextDescIndex = txDirtyIndex = txCleanBarrierIndex = 0;
    txNumFreeDesc = kNumTxDesc;
    
    /* On descriptor writeback the buffer addresses are overwritten so that
     * we must restore them in order to make sure that we leave the ring in
     * a usable state.
     */
    if (rxDescArray) {
        for (i = 0; i < kNumRxDesc; i++) {
            rxDescArray[i].read.buffer_addr = OSSwapHostToLittleInt64(rxBufArray[i].phyAddr);
            rxDescArray[i].read.reserved = 0;
        }
    }
    rxCleanedCount = rxNextDescIndex = 0;

    /* Free packet fragments which haven't been upstreamed yet.  */
    discardPacketFragment();
    
    DebugLog("[IntelMausi]: clearDescriptors() <===\n");
}

void IntelMausi::discardPacketFragment(bool extended)
{
    /*
     * In case there is a packet fragment which hasn't been enqueued yet
     * we have to free it in order to prevent a memory leak.
     */
    if (rxPacketHead) {
        if (extended)
            freePacketEx(rxPacketHead);
        else
            freePacket(rxPacketHead);
    }
    
    rxPacketHead = rxPacketTail = NULL;
    rxPacketSize = 0;
}

/*
 * Retrieve a list of IPv4 and IVv6 addresses of the interface which
 * are required by the ARP and IP wakeup filters. As hardware supports
 * a maximum of 3 IPv4 and 4 IPv6 addresses, we have to ignore
 * excess addresses and limit IPv6 addresses to Link-Local and
 * Unique Local Addresses.
 */

void IntelMausi::getAddressList(struct IntelAddrData *addrData)
{
    ifnet_t interface = netif->getIfnet();
    ifaddr_t *addresses;
    ifaddr_t addr;
    struct sockaddr_in addr4;
    struct sockaddr_in6 addr6;
    sa_family_t family;
    u_int32_t i = 0, prefix;
    
    addrData->ipV6Count = 0;
    addrData->ipV4Count = 0;
    
    if (enableWoM) {
        if (!ifnet_get_address_list(interface, &addresses)) {
            while ((addr = addresses[i++]) != NULL) {
                family = ifaddr_address_family(addr);
                
                switch (family) {
                    case AF_INET:
                        if (!ifaddr_address(addr, (struct sockaddr *) &addr4, sizeof(struct sockaddr_in))) {
                            if (addrData->ipV4Count < kMaxAddrV4) {
                                addrData->ipV4Addr[addrData->ipV4Count++] = addr4.sin_addr.s_addr;
                                
                                DebugLog("[IntelMausi]: IPv4 address 0x%08x.\n", OSSwapBigToHostInt32(addr4.sin_addr.s_addr));
                            }
                        }
                        break;
                        
                    case AF_INET6:
                        if (!ifaddr_address(addr, (struct sockaddr *) &addr6, sizeof(struct sockaddr_in6))) {
                            prefix = OSSwapBigToHostInt32(addr6.sin6_addr.s6_addr32[0]);
                            
                            if ((addrData->ipV6Count < kMaxAddrV6) && (((prefix & kULAMask) == kULAPrefix) || ((prefix & kLLAMask) == kLLAPrefix))) {
                                addrData->ipV6Addr[addrData->ipV6Count].s6_addr32[0] = addr6.sin6_addr.s6_addr32[0];
                                addrData->ipV6Addr[addrData->ipV6Count].s6_addr32[1] = addr6.sin6_addr.s6_addr32[1];
                                addrData->ipV6Addr[addrData->ipV6Count].s6_addr32[2] = addr6.sin6_addr.s6_addr32[2];
                                addrData->ipV6Addr[addrData->ipV6Count++].s6_addr32[3] = addr6.sin6_addr.s6_addr32[3];
                                
                                DebugLog("[IntelMausi]: IPv6 address 0x%08x 0x%08x 0x%08x 0x%08x.\n", OSSwapBigToHostInt32(addr6.sin6_addr.s6_addr32[0]), OSSwapBigToHostInt32(addr6.sin6_addr.s6_addr32[1]), OSSwapBigToHostInt32(addr6.sin6_addr.s6_addr32[2]), OSSwapBigToHostInt32(addr6.sin6_addr.s6_addr32[3]));
                            }
                        }
                        break;
                        
                    default:
                        break;
                }
            }
            ifnet_free_address_list(addresses);
        }
    }
}
