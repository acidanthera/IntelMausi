/* IntelMausiEthernet.h -- IntelMausi driver class definition.
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

extern "C" {
    #include "e1000.h"
    #include <kdp/kdp_support.h>
}

#ifdef DEBUG
#define DebugLog(args...) IOLog(args)
#else
#define DebugLog(args...)
#endif

#define	RELEASE(x)	if(x){(x)->release();(x)=NULL;}

#define intelWriteMem8(reg, val8)       _OSWriteInt8((baseAddr), (reg), (val8))
#define intelWriteMem16(reg, val16)     OSWriteLittleInt16((baseAddr), (reg), (val16))
#define intelWriteMem32(reg, val32)     OSWriteLittleInt32((baseAddr), (reg), (val32))
#define intelReadMem8(reg)              _OSReadInt8((baseAddr), (reg))
#define intelReadMem16(reg)             OSReadLittleInt16((baseAddr), (reg))
#define intelReadMem32(reg)             OSReadLittleInt32((baseAddr), (reg))
#define intelFlush()                    OSReadLittleInt32((baseAddr), (E1000_STATUS))

#define super IOEthernetController

enum
{
	MEDIUM_INDEX_AUTO = 0,
	MEDIUM_INDEX_10HD,
	MEDIUM_INDEX_10FD,
	MEDIUM_INDEX_100HD,
    MEDIUM_INDEX_100FD,
    MEDIUM_INDEX_100FDFC,
    MEDIUM_INDEX_1000FD,
    MEDIUM_INDEX_1000FDFC,
    MEDIUM_INDEX_1000FDEEE,
    MEDIUM_INDEX_1000FDFCEEE,
    MEDIUM_INDEX_100FDEEE,
    MEDIUM_INDEX_100FDFCEEE,
	MEDIUM_INDEX_COUNT
};

#define MBit 1000000

enum {
    kSpeed1000MBit = 1000*MBit,
    kSpeed100MBit = 100*MBit,
    kSpeed10MBit = 10*MBit,
};

enum {
    kFlowControlTypeNone = 0,
    kFlowControlTypeRx = 1,
    kFlowControlTypeTx = 2,
    kFlowControlTypeRxTx = 3,
    kFlowControlTypeCount
};

enum {
    kEEETypeNo = 0,
    kEEETypeYes = 1,
    kEEETypeCount
};

#define kTransmitQueueCapacity  1000

/* With up to 40 segments we should be on the save side. */
#define kMaxSegs 40

#define kTxSpareDescs   16

/* The number of descriptors must be a power of 2. */
#define kNumTxDesc      1024        /* Number of Tx descriptors */
#define kNumRxDesc      512         /* Number of Rx descriptors */
#define kNumKdpDesc     kNumTxDesc  /* Number of Kdp descriptors */
#define kTxLastDesc    (kNumTxDesc - 1)
#define kRxLastDesc    (kNumRxDesc - 1)
#define kTxDescMask    (kNumTxDesc - 1)
#define kRxDescMask    (kNumRxDesc - 1)
#define kTxDescSize    (kNumTxDesc*sizeof(struct e1000_data_desc))
#define kRxDescSize    (kNumRxDesc*sizeof(union e1000_rx_desc_extended))

/* This is the receive buffer size (must be large enough to hold a packet). */
#define kRxBufferPktSize 2048
#define kRxNumSpareMbufs 100
#define kMCFilterLimit 32
#define kMaxRxQueques 1
#define kMaxMtu 9000
#define kMaxPacketSize (kMaxMtu + ETH_HLEN + ETH_FCS_LEN)

/* statitics timer period in ms. */
#define kTimeoutMS 1000

/* Treshhold value to wake a stalled queue */
#define kTxQueueWakeTreshhold (kNumTxDesc / 4)

/* transmitter deadlock treshhold in seconds. */
#define kTxDeadlockTreshhold 2

/* Maximum DMA latency in ns. */
#define kMaxDmaLatency 75000

/* IP specific stuff */
#define kMinL4HdrOffsetV4 34
#define kMinL4HdrOffsetV6 54

#define kIPv4CSumStart      ETH_HLEN
#define kIPv4CSumOffset     (offsetof(struct iphdr, check) + kIPv4CSumStart)
#define kIPv4CSumEnd        (sizeof(struct iphdr) + kIPv4CSumStart - 1)

#define kUDPv4CSumStart     kMinL4HdrOffsetV4
#define kUDPv4CSumOffset    (offsetof(struct udphdr, uh_sum) + kMinL4HdrOffsetV4)
#define kUDPv4CSumEnd       0

#define kTCPv4CSumStart     kMinL4HdrOffsetV4
#define kTCPv4CSumOffset    (offsetof(struct tcphdr, th_sum) + kMinL4HdrOffsetV4)
#define kTCPv4CSumEnd       0

#define kIPv6CSumStart      ETH_HLEN
#define kIPv6CSumOffset     0
#define kIPv6CSumEnd        0

#define kTCPv6CSumStart     kMinL4HdrOffsetV6
#define kTCPv6CSumOffset    (offsetof(struct tcphdr, th_sum) + kMinL4HdrOffsetV6)
#define kTCPv6CSumEnd       0

#define kUDPv6CSumStart     kMinL4HdrOffsetV6
#define kUDPv6CSumOffset    (offsetof(struct udphdr, uh_sum) + kMinL4HdrOffsetV6)
#define kUDPv6CSumEnd       0

#define SPEED_MODE_BIT (1 << 21)
#define E1000_TARC_QUEUE_EN   0x00000400

#define E1000_RXD_STAT_IPPCS        0x40            /* IP xsum calculated */

#define E1000_RXDLGC_ERR_CE        0x0100    /* CRC Error */
#define E1000_RXDLGC_ERR_SE        0x0200    /* Symbol Error */
#define E1000_RXDLGC_ERR_SEQ       0x0400    /* Sequence Error */
#define E1000_RXDLGC_ERR_CXE       0x1000    /* Carrier Extension Error */
#define E1000_RXDLGC_ERR_TCPE      0x2000    /* TCP/UDP Checksum Error */
#define E1000_RXDLGC_ERR_IPE       0x4000    /* IP Checksum Error */
#define E1000_RXDLGC_ERR_RXE       0x8000    /* Rx Data Error */

#define E1000_RXDEXT_STATERR_TCPE   0x20000000
#define E1000_RXDEXT_STATERR_IPE    0x40000000

/* mask to determine if packets should be dropped due to frame errors */
#define E1000_RXDLGC_ERR_FRAME_ERR_MASK ( \
    E1000_RXDLGC_ERR_CE  |		\
    E1000_RXDLGC_ERR_SE  |		\
    E1000_RXDLGC_ERR_SEQ |		\
    E1000_RXDLGC_ERR_CXE |		\
    E1000_RXDLGC_ERR_RXE)

#define E1000_TX_FLAGS_CSUM		0x00000001
#define E1000_TX_FLAGS_VLAN		0x00000002
#define E1000_TX_FLAGS_TSO		0x00000004
#define E1000_TX_FLAGS_IPV4		0x00000008
#define E1000_TX_FLAGS_NO_FCS		0x00000010
#define E1000_TX_FLAGS_HWTSTAMP		0x00000020
#define E1000_TX_FLAGS_VLAN_MASK	0xffff0000
#define E1000_TX_FLAGS_VLAN_SHIFT	16

#define E1000_TXD_OPTS_IXSM     0x00000100
#define E1000_TXD_OPTS_TXSM     0x00000200

#define E1000_RCTL_FLXB_SHIFT   27

#define E1000_ICR_TXQE          0x00000002      /* Transmit queue empty */
#define E1000_IMS_TXQE          E1000_ICR_TXQE  /* Transmit queue empty */

/* Transmit Interrupt Delay */
#define DEFAULT_TIDV 8

/* Transmit Absolute Interrupt Delay in units of 1.024 microseconds */
#define DEFAULT_TADV 32

/*
#define E1000_SRPD      0x02C00
#define E1000_RAID      0x02C08
#define E1000_IMS_RSPD  0x00010000
#define E1000_IMS_ACK   0x00020000
*/

/* These definitions should have been in IOPCIDevice.h. */
enum
{
    kIOPCIPMCapability = 2,
    kIOPCIPMControl = 4,
};

enum
{
    kIOPCIEDeviceControl = 8,
    kIOPCIELinkCapability = 12,
    kIOPCIELinkControl = 16,
};

enum
{
    kIOPCIELinkCtlASPM = 0x0003,    /* ASPM Control */
    kIOPCIELinkCtlL0s = 0x0001,     /* L0s Enable */
    kIOPCIELinkCtlL1 = 0x0002,      /* L1 Enable */
    kIOPCIELinkCtlCcc = 0x0040,     /* Common Clock Configuration */
    kIOPCIELinkCtlClkReqEn = 0x100, /* Enable clkreq */
};

enum
{
    kIOPCIEDevCtlReadQ = 0x7000,
};

enum
{
    kPowerStateOff = 0,
    kPowerStateOn,
    kPowerStateCount
};

#define kParamName "Driver Parameters"
#define kEnableCSO6Name "enableCSO6"
#define kEnableWoMName "enableWakeOnAddrMatch"
#define kIntrRate10Name "maxIntrRate10"
#define kIntrRate100Name "maxIntrRate100"
#define kIntrRate1000Name "maxIntrRate1000"
#define kDriverVersionName "Driver_Version"
#define kNameLenght 64

#define kRxAbsTime10Name "rxAbsTime10"
#define kRxAbsTime100Name "rxAbsTime100"
#define kRxAbsTime1000Name "rxAbsTime1000"

#define kRxDelayTime10Name "rxDelayTime10"
#define kRxDelayTime100Name "rxDelayTime100"
#define kRxDelayTime1000Name "rxDelayTime1000"

struct intelDevice {
    UInt16 pciDevId;
    UInt16 device;
    const char *deviceName;
    const struct e1000_info *deviceInfo;
};

#define kInvalidRingIndex 0xffffffff;

struct intelTxBufferInfo {
    mbuf_t mbuf;
    UInt32 numDescs;
    UInt32 pad;
};
struct intelRxBufferInfo {
    mbuf_t mbuf;
    IOPhysicalAddress64 phyAddr;
};

struct IntelRxDesc {
    UInt64 bufferAddr;
    UInt64 status;
};

/*
 * Keep alive address data as supported by hardware with a
 * maximum of:
 *  3 IPv4 addresses
 *  4 IPv6 addresses
 */
#define kMaxAddrV4 3
#define kMaxAddrV6 4

#define kLLAPrefix  0xfe800000
#define kLLAMask    0xffc00000
#define kULAPrefix  0xfc000000
#define kULAMask    0xfe000000

struct IntelAddrData {
    UInt16 ipV6Count;
    UInt16 ipV4Count;
    struct in6_addr ipV6Addr[kMaxAddrV6];
    UInt32 ipV4Addr[kMaxAddrV4];
};

class IntelMausi : public super
{
	
	OSDeclareDefaultStructors(IntelMausi)
	
public:
	/* IOService (or its superclass) methods. */
	virtual bool start(IOService *provider) APPLE_KEXT_OVERRIDE;
	virtual void stop(IOService *provider) APPLE_KEXT_OVERRIDE;
	virtual bool init(OSDictionary *properties) APPLE_KEXT_OVERRIDE;
	virtual void free() APPLE_KEXT_OVERRIDE;
	
	/* Power Management Support */
	virtual IOReturn registerWithPolicyMaker(IOService *policyMaker) APPLE_KEXT_OVERRIDE;
    virtual IOReturn setPowerState(unsigned long powerStateOrdinal, IOService *policyMaker) APPLE_KEXT_OVERRIDE;
	virtual void systemWillShutdown(IOOptionBits specifier) APPLE_KEXT_OVERRIDE;
    
	/* IONetworkController methods. */
    virtual IOReturn enable(IONetworkInterface *netif) APPLE_KEXT_OVERRIDE;
    virtual IOReturn disable(IONetworkInterface *netif) APPLE_KEXT_OVERRIDE;
    virtual IOReturn enable(IOKernelDebugger *debugger) APPLE_KEXT_OVERRIDE;
    virtual IOReturn disable(IOKernelDebugger *debugger) APPLE_KEXT_OVERRIDE;

    virtual void receivePacket(void *pkt, UInt32 *pktSizeOut, UInt32 timeout) APPLE_KEXT_OVERRIDE;
    virtual void sendPacket(void *pkt, UInt32 pktSize) APPLE_KEXT_OVERRIDE;

#ifdef __PRIVATE_SPI__
    virtual IOReturn outputStart(IONetworkInterface *interface, IOOptionBits options) APPLE_KEXT_OVERRIDE;
    virtual IOReturn setInputPacketPollingEnable(IONetworkInterface *interface, bool enabled) APPLE_KEXT_OVERRIDE;
    virtual void pollInputPackets(IONetworkInterface *interface, uint32_t maxCount, IOMbufQueue *pollQueue, void *context) APPLE_KEXT_OVERRIDE;
#else
    virtual UInt32 outputPacket(mbuf_t m, void *param) APPLE_KEXT_OVERRIDE;
#endif /* __PRIVATE_SPI__ */
	
	virtual void getPacketBufferConstraints(IOPacketBufferConstraints *constraints) const APPLE_KEXT_OVERRIDE;
	
	virtual IOOutputQueue* createOutputQueue() APPLE_KEXT_OVERRIDE;
	
	virtual const OSString* newVendorString() const APPLE_KEXT_OVERRIDE;
	virtual const OSString* newModelString() const APPLE_KEXT_OVERRIDE;
	
	virtual IOReturn selectMedium(const IONetworkMedium *medium) APPLE_KEXT_OVERRIDE;
	virtual bool configureInterface(IONetworkInterface *interface) APPLE_KEXT_OVERRIDE;
	
	virtual bool createWorkLoop() APPLE_KEXT_OVERRIDE;
	virtual IOWorkLoop* getWorkLoop() const APPLE_KEXT_OVERRIDE;
	
	/* Methods inherited from IOEthernetController. */
	virtual IOReturn getHardwareAddress(IOEthernetAddress *addr) APPLE_KEXT_OVERRIDE;
	virtual IOReturn setHardwareAddress(const IOEthernetAddress *addr) APPLE_KEXT_OVERRIDE;
	virtual IOReturn setPromiscuousMode(bool active) APPLE_KEXT_OVERRIDE;
	virtual IOReturn setMulticastMode(bool active) APPLE_KEXT_OVERRIDE;
	virtual IOReturn setMulticastList(IOEthernetAddress *addrs, UInt32 count) APPLE_KEXT_OVERRIDE;
	virtual IOReturn getChecksumSupport(UInt32 *checksumMask, UInt32 checksumFamily, bool isOutput) APPLE_KEXT_OVERRIDE;
	virtual IOReturn getMinPacketSize(UInt32 *minSize) const APPLE_KEXT_OVERRIDE;
    virtual IOReturn setWakeOnMagicPacket(bool active) APPLE_KEXT_OVERRIDE;
    virtual IOReturn getPacketFilters(const OSSymbol *group, UInt32 *filters) const APPLE_KEXT_OVERRIDE;
    
    virtual UInt32 getFeatures() const APPLE_KEXT_OVERRIDE;
    virtual IOReturn getMaxPacketSize(UInt32 * maxSize) const APPLE_KEXT_OVERRIDE;
    virtual IOReturn setMaxPacketSize(UInt32 maxSize) APPLE_KEXT_OVERRIDE;

private:
    IOReturn driverEnable();
    IOReturn driverDisable();
    bool initPCIConfigSpace(IOPCIDevice *provider);
    void initPCIPowerManagment(IOPCIDevice *provider, const struct e1000_info *ei);
    inline void intelEnablePCIDevice(IOPCIDevice *provider);
    static IOReturn setPowerStateWakeAction(OSObject *owner, void *arg1, void *arg2, void *arg3, void *arg4);
    static IOReturn setPowerStateSleepAction(OSObject *owner, void *arg1, void *arg2, void *arg3, void *arg4);
    void getParams();
    bool setupMediumDict();
    bool initEventSources(IOService *provider);
    void interruptOccurred(OSObject *client, IOInterruptEventSource *src, int count);
    void txInterrupt(IOOptionBits options = 0);
    void freePacketEx(mbuf_t pkt, IOOptionBits options = 0);
    void kdpStartup();
    bool isKdpPacket(UInt8 *data, UInt32 len);

#ifdef __PRIVATE_SPI__
    UInt32 rxInterrupt(IONetworkInterface *interface, uint32_t maxCount, IOMbufQueue *pollQueue, void *context);
#else
    void rxInterrupt();
#endif /* __PRIVATE_SPI__ */

    bool setupDMADescriptors();
    void freeDMADescriptors();
    void clearDescriptors();
    void checkLinkStatus();
    void updateStatistics(struct e1000_adapter *adapter);
    void setLinkUp();
    void setLinkDown();
    bool checkForDeadlock();
    
    /* Jumbo frame support methods */
    void discardPacketFragment(bool extended = false);
    
    /* Hardware specific methods */
    bool intelIdentifyChip();
    bool intelStart();
    void intelEEPROMChecks(struct e1000_adapter *adapter);
    void intelInitTxRing();
    void intelInitRxRing();
    void intelEnableIRQ(UInt32 newMask);
    void intelDisableIRQ();
    void intelUpdateTxDescTail(UInt32 index);
    void intelUpdateRxDescTail(UInt32 index);

    void intelEnable();
    void intelDisable();
    void intelConfigure(struct e1000_adapter *adapter);
    void intelConfigureTx(struct e1000_adapter *adapter);
    void intelSetupRxControl(struct e1000_adapter *adapter);
    void intelConfigureRx(struct e1000_adapter *adapter);
    void intelDown(struct e1000_adapter *adapter, bool reset);
    void intelInitManageabilityPt(struct e1000_adapter *adapter);
    void intelReset(struct e1000_adapter *adapter);
    void intelPowerDownPhy(struct e1000_adapter *adapter);
    bool intelEnableMngPassThru(struct e1000_hw *hw);
    void intelResetAdaptive(struct e1000_hw *hw);
    void intelUpdateAdaptive(struct e1000_hw *hw);
    void intelVlanStripDisable(struct e1000_adapter *adapter);
    void intelVlanStripEnable(struct e1000_adapter *adapter);
    void intelSetupRssHash(struct e1000_adapter *adapter);

    void intelRestart();
    bool intelCheckLink(struct e1000_adapter *adapter);
    void intelFlushDescriptors();
    void intelFlushTxRing(struct e1000_adapter *adapter);
    void intelFlushRxRing(struct e1000_adapter *adapter);
    void intelFlushDescRings(struct e1000_adapter *adapter);
    void intelPhyReadStatus(struct e1000_adapter *adapter);
    void intelInitPhyWakeup(UInt32 wufc, struct IntelAddrData *addrData);
    void intelInitMacWakeup(UInt32 wufc, struct IntelAddrData *addrData);
    void intelSetupAdvForMedium(const IONetworkMedium *medium);
    void intelFlushLPIC();
    void setMaxLatency(UInt32 linkSpeed);
    
    UInt16 intelSupportsEEE(struct e1000_adapter *adapter);
    SInt32 intelEnableEEE(struct e1000_hw *hw, UInt16 mode);
    
    inline void intelGetChecksumResult(mbuf_t m, UInt32 status);

    void getAddressList(struct IntelAddrData *addr);

    /* timer action */
    void timerAction(IOTimerEventSource *timer);
    
private:
	IOWorkLoop *workLoop;
    IOCommandGate *commandGate;
	IOPCIDevice *pciDevice;
	OSDictionary *mediumDict;
	IONetworkMedium *mediumTable[MEDIUM_INDEX_COUNT];
	IOBasicOutputQueue *txQueue;
	
	IOInterruptEventSource *interruptSource;
	IOTimerEventSource *timerSource;
	IOEthernetInterface *netif;
	IOMemoryMap *baseMap;
    volatile void *baseAddr;
	IOMemoryMap *flashMap;
    volatile void *flashAddr;
    
    /* transmitter data */
    IODMACommand *txDescDmaCmd;
    IOBufferMemoryDescriptor *txBufDesc;
    IOPhysicalAddress64 txPhyAddr;
    struct e1000_data_desc *txDescArray;
    IOMbufNaturalMemoryCursor *txMbufCursor;
    UInt64 txDescDoneCount;
    UInt64 txDescDoneLast;
    SInt32 txNumFreeDesc;
    UInt32 mtu;
    UInt32 maxLatency;
    UInt16 txNextDescIndex;
    UInt16 txDirtyIndex;
    UInt16 txCleanBarrierIndex;
    
    /* receiver data */
    IODMACommand *rxDescDmaCmd;
    IOBufferMemoryDescriptor *rxBufDesc;
    IOPhysicalAddress64 rxPhyAddr;
    union e1000_rx_desc_extended *rxDescArray;
	IOMbufNaturalMemoryCursor *rxMbufCursor;
    mbuf_t rxPacketHead;
    mbuf_t rxPacketTail;
    UInt32 rxPacketSize;
    IOEthernetAddress *mcAddrList;
    UInt32 mcListCount;
    UInt16 rxNextDescIndex;
    UInt16 rxCleanedCount;
    
    /* power management data */
    unsigned long powerState;
    
    /* statistics data */
    UInt32 deadlockWarn;
    IONetworkStats *netStats;
	IOEthernetStats *etherStats;
    
    UInt32 chip;
    UInt32 chipType;
    UInt32 intrMask;
    UInt32 intrThrValue10;
    UInt32 intrThrValue100;
    UInt32 intrThrValue1000;
    struct e1000_adapter adapterData;
    struct pci_dev pciDeviceData;

    /* fine granular interrupt delay values */
    UInt32 rxAbsTime10;
    UInt32 rxAbsTime100;
    UInt32 rxAbsTime1000;
    UInt32 rxDelayTime10;
    UInt32 rxDelayTime100;
    UInt32 rxDelayTime1000;

    UInt16 eeeMode;
    UInt8 pcieCapOffset;
    UInt8 pciPMCtrlOffset;

#ifdef __PRIVATE_SPI__
    IONetworkPacketPollingParameters pollParams;
#endif /* __PRIVATE_SPI__ */

    /* flags */
    bool isEnabled;
	bool promiscusMode;
	bool multicastMode;
    bool linkUp;

#ifdef __PRIVATE_SPI__
    bool polling;
#else
    bool stalled;
#endif /* __PRIVATE_SPI__ */

    bool forceReset;
    bool wolCapable;
    bool wolActive;
    bool enableCSO6;
    bool enableWoM;
    
    /* mbuf_t arrays */
    struct intelTxBufferInfo txBufArray[kNumTxDesc];
    struct intelRxBufferInfo rxBufArray[kNumRxDesc];

    /* debugger array pool */
    mbuf_t kdpBufArray[kNumKdpDesc];
    IOKernelDebugger *debugger;
    bool hasDebugger;
};
