#ifndef __ETH_H
#define __ETH_H  /* #defined if this .h file has been included */

/* Standard C-native includes  */
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

#define UINT8 uint8_t
#define UINT16 uint16_t
#define UINT32 uint32_t
#define UINT64 uint64_t

#define INT8 int8_t
#define INT16 int16_t
#define INT32 int32_t
#define INT64 int64_t

typedef unsigned int uint;

#define bzero(p,n) memset((p),(0),(n))

#ifdef BIGENDIAN
// Data Host/Network Byte Order Conversion
#define HNC16(a) (a)
#define HNC32(a) (a)

// 32 Bit Data Macros (from 16 bit aligned data)
#define RdNet32(x)   (((UINT32)(*(UINT16 *)(x))<<16)|(UINT32)(*(UINT16 *)(((UINT8 *)(x))+2)))
#define WrNet32(x,y) *(UINT16 *)(x)=(UINT16)((y)>>16); *(UINT16 *)(((UINT8 *)(x))+2)=(UINT16)(y)

// READ/WRITE Macros (aligned)
#define READ32(x)    (*(volatile unsigned int *)x)
#define WRITE32(x,y) (*(volatile unsigned int *)(x) = (y))

#else // LITTLE ENDIAN

// Data Host/Network Byte Order Conversion
#define HNC16(a) ( (((a)>>8)&0xff) + (((a)<<8)&0xff00) )
#define HNC32(a) ( (((a)>>24)&0xff) + (((a)>>8)&0xff00) +   \
                 (((a)<<8)&0xff0000) + (((a)<<24)&0xff000000) )

// 32 Bit Data Macros (from 16 bit aligned data)
#define RdNet32(x)   ((UINT32)(*(UINT16 *)(x))|((UINT32)(*(UINT16 *)(((UINT8 *)(x))+2))<<16))
#define WrNet32(x,y) *(UINT16 *)(x)=(UINT16)(y); *(UINT16 *)(((UINT8 *)(x))+2)=(UINT16)((y)>>16)

// READ/WRITE Macros (aligned)
#define READ32(x)    (*(volatile unsigned int *)x)
#define WRITE32(x,y) (*(volatile unsigned int *)(x) = (y))

#endif

typedef void * PBM_Handle;

#define PBM_getBufferLen(hPkt)      (((PBM_Pkt*)hPkt)->BufferLen)
#define PBM_getDataBuffer(hPkt)     (((PBM_Pkt*)hPkt)->pDataBuffer)
#define PBM_getValidLen(hPkt)       (((PBM_Pkt*)hPkt)->ValidLen)
#define PBM_getDataOffset(hPkt)     (((PBM_Pkt*)hPkt)->DataOffset)
#define PBM_getIFRx(hPkt)           (((PBM_Pkt*)hPkt)->hIFRx)

#define PBM_setValidLen(hPkt,x)     (((PBM_Pkt*)hPkt)->ValidLen=(x))
#define PBM_setDataOffset(hPkt,x)   (((PBM_Pkt*)hPkt)->DataOffset=(x))
#define PBM_setIFRx(hPkt,x)         (((PBM_Pkt*)hPkt)->hIFRx=(x))

// Packet Buffer Manager Initialization Functions
extern uint        PBM_open();
extern void        PBM_close();

// Packet Buffer Functions (re-entrant and "kernel mode" agnostic)
extern PBM_Handle  PBM_alloc( uint MaxSize );
extern PBM_Handle  PBM_copy( PBM_Handle hPkt );
extern void        PBM_free( PBM_Handle hPkt );

/* Socket Priority
 *  Each socket can be associated with a specific priority. This priority
 *  can be configured through the setsockopt API. All packets transmitted
 *  through this socket will have the same priority. This is used by the
 *  VLAN drivers in the system to remark the packets with an appropriate
 *  Layer2 User Priority value.
 *  By default; there is no priority associated with the socket. */
#define PRIORITY_UNDEFINED 0xFFFF

/* Interface Name Length */
#define MAX_INTERFACE_NAME_LEN           20

typedef struct {
	uint8_t   DstMac[6];
	uint8_t   SrcMac[6];
	uint16_t  Type;
} ETHHDR;
#define ETHHDR_SIZE     14

//typedef void * STKEVENT_Handle;
/*
 * Pre-Pad Packet Data Offset
 *
 *   The TCP/IP stack library requires that every packet device
 *   include enough L2 header room for all supported headers. In
 *   order to support PPPoE, this requires a 22 byte L2 header.
 *   Thus, since standard Ethernet is only 14 bytes, we must add
 *   on an additional 8 byte offset, or PPPoE can not function
 *   with our driver.
*/
#define     PKT_PREPAD                      8

/* Indicates whether RAM based multicast lists are suported for this
 * EMAC peripheral.
 */
#define     RAM_MCAST                       0

/* Indicates whether HASH based multicasting is suported for this
 * EMAC peripheral.
 */
#define     HASH_MCAST                      0

/* Multicast Address List Size */
#define     PKT_MAX_MCAST                   31

/** Number of ports in the ethernet subsystem */
#define         NUM_PORTS                   3u

/** Number of MAC/GMII ports in the ethernet switch */
#define         NUM_MAC_PORTS               2u


/** Host descriptor size.
 *
 *  Big enough to hold the mandatory fields of the
 *  host descriptor and PA Control Data
 *
 *  = 32 bytes for Host desc + PA Control data + 16 bytes padding
 */
#define     SIZE_HOST_DESC              MAX_DESC_SIZE

/* High Priority QM Rx Interrupt Threshold */
#define     RX_INT_THRESHOLD            4u

/* Accumulator channel to use */
#define     PA_ACC_CHANNEL_NUM          0u

/* Max Number of Rx packets in the Queue before passing to NDK */
#define     MAX_NUM_RAW_PKTS_INQ                    4


/**
 * @brief   External memory start address
 */
#define     EMAC_EXTMEM                          0x80000000
#define     EMAC_LL2SRAM                         0x00800000
#define     EMAC_MSMCSRAM                        0x0c000000

#define ETH_MAX_PAYLOAD  1514

typedef void *         HANDLE;
#if 1
#define mmAlloc malloc
#define mmFree free
#define mmCopy memcpy
#define mmZeroInit bzero
#endif

uint OEMSysCritOn();
void OEMSysCritOff(uint key);

typedef void   (*TimestampFxn)(UINT8 *pIpHdr);
// Packet Buffer Object
typedef struct _PBM_Pkt {
    uint32_t        Type;         // Identifier (Read Only)
    struct _PBM_Pkt *pPrev;       // Previous record
    struct _PBM_Pkt *pNext;       // Next record
    uint8_t         *pDataBuffer; // Pointer to Data Buffer (Read Only)
    uint            BufferLen;    // Physical Length of buffer (Read Only)
    uint            Flags;        // Packet Flags
    uint            ValidLen;     // Length of valid data in buffer
    uint            DataOffset;   // Byte offset to valid data
    uint            EtherType;    // Ether Type Code
    uint            L2HdrLen;     // Length of L2 Hdr (on 'L3' Rx pkts)
    uint            IpHdrLen;     // Length of Ip Hdr
    HANDLE          hIFRx;        // Rx Interface
    HANDLE          hIFTx;        // Tx Interface
    HANDLE          hRoute;       // Handle to Route
    uint16_t        PktPriority;  // Priority of the packet.
    uint32_t        Aux1;         // Aux1 Data
    uint32_t        Aux2;         // Aux2 Data
    TimestampFxn    pTimestampFxn;// Callout function pointer to
                                  // timestamp TX
    uint8_t         *pIpHdr;      // Pointer to IP Header

} PBM_Pkt;

// Packet Buffer Queue Object
typedef struct _PBMQ {
  uint              Count;      // Number of packets in queue
  PBM_Pkt           *pHead;     // Pointer to first packet
  PBM_Pkt           *pTail;     // Pointer to last packet
} PBMQ;

#define            PBMQ_init(pQ)   mmZeroInit( pQ, sizeof(PBMQ) )
#define            PBMQ_count(pQ) ((pQ)->Count)

extern void       PBMQ_enq( PBMQ *pQ, PBM_Handle hPkt );
extern PBM_Handle PBMQ_deq( PBMQ *pQ );

/* Rx queue (one for all PKT devices) */
extern PBMQ    PBMQ_rx;

/**
 * @brief
 *  Packet device information
 *
 * @details
 *  This structure caches the device info.
 */
typedef struct _pdinfo
{
    /**
     * @brief       Physical index of this device (0 to n-1).
     */
    uint32_t            PhysIdx;
    /*
     * @brief       Handle to logical driver.
     */
    HANDLE          hEther;
    /*
     * @brief       Semaphore handle used by NDK stack and driver
     *              to communicate any pending Rx events that need
     *              to be serviced by NDK ethernet stack.
     */
    //STKEVENT_Handle hEvent;
    /*
     * @brief       MAC Address
     */
    uint8_t           bMacAddr[6];
    /*
     * @brief       Current RX filter
     */
    uint32_t            Filter;
    /*
     * @brief       Current MCast Address Countr
     */
    uint32_t            MCastCnt;
    /*
     * @brief       Multicast list configured by the Application.
     */
    uint8_t           bMCast[6*PKT_MAX_MCAST];
    /*
     * @brief       Transmitter "free" flag
     */
    uint32_t            TxFree;
    /*
     * @brief       Tx queue (one for each PKT device)
     */
    PBMQ            PBMQ_tx;

    /*
     * @brief       Raw Pkt Tx queue (one for each PKT device)
     */
    PBMQ            PBMQ_rawtx;

//#ifdef _INCLUDE_NIMU_CODE
    /*
     * @brief       Rx queue (one for each PKT device)
     */
	PBMQ    		PBMQ_rx;

    /*
     * @brief       Raw Pkt Rx queue (one for each PKT device)
     */
	PBMQ    		PBMQ_rawrx;
//#endif
} PDINFO;

/**
 * @brief
 *   EMAC_DATA
 *
 * @details
 *  The structure is used to store the private data for the
 *  EMAC controller.
 */
typedef struct EMAC_DATA
{
/**
  * @brief   Private Information
  */
    PDINFO      pdi;
}EMAC_DATA;


/*********************************************************************
 * STRUCTURE NAME : NETIF_DEVICE
 *********************************************************************
 * DESCRIPTION   :
 *  The structure describes the network interface object. Each device
 *  driver in the system which is attached to the NDK stack should be
 *  associated with an instance of this object. The object describes
 *  the interface between the NDK Core stack and the drivers.
 *********************************************************************/
typedef struct NETIF_DEVICE
{

    /* Reference Counter: This indicates the number of references of the
     * network interface object is held by components. Network Interface
     * Objects can only be removed from the system if there are no
     * references of it held in the System. */
     uint       RefCount;

    /* These are the two identifiers which are associated with each network
     * interface device. The "index" is a numeric representation and the
     * "name" is a more user friendly string representation of the same.
     *
     * NOTES:
     * Driver Authors can specify these; but in the case of conflicts
     * these values will be modified to be unique in the system. Thus if
     * driver authors are using these in their code it is best to re-read
     * these values after the 'registration' process. */
    uint        index;
    char        name[MAX_INTERFACE_NAME_LEN];

    /* This defines the interface flags.
     *
     * NOTES:
     * Driver authors should not set this value as this is used for internal
     * operations inside the NDK stack. */
    uint        flags;

    /* This defines the interface type.
     *
     * NOTES:
     *  For compatibility with the old network interface object; this is set to
     *  HTYPE_ETHER or HTYPE_PPP; depending on the type of network interface
     *  object. Moving forward this field will be obsoleted and instead application
     *  authors should use the field instead. */
    uint        type;

    /* This is the Max. Protocol MTU associated with the device.
     *
     * NOTES:
     * Driver authors should configure this value to the MAX. Data payload
     * that can be carried without the corresponding Layer2 Header. Thus for
     * example in Ethernet this will be 1514 (Max. Data Payload) - 14 (L2
     * Ethernet Header) = 1500. */
    uint        mtu;

    /* MAC Address with the device. */
    uint8_t       mac_address[6];

    /* Pointer to 'private data' associated with the device. This data
     * pointer is opaque to the NDK stack.
     *
     * NOTES:
     * Driver authors can use this to store any additional 'driver'
     * specific data here. Memory allocation and cleanup of this
     * private block is the responsibility of the driver. */
    void*       pvt_data;

}NETIF_DEVICE;

#endif
