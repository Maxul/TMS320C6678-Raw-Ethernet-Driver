/*===============================================================*\
| Project: RTEMS TMS320C6678 BSP                                  |
+-----------------------------------------------------------------+
| Partially based on the code references which are named below.   |
| Adaptions, modifications, enhancements and any recent parts of  |
| the code are:                                                   |
|                 Copyright (c) 2016                              |
|                 Beijing University of Posts & Communications    |
|                 China                                           |
|                 Maxul Lee                                       |
|                 lmy2010lmy@gamil.com                            |
+-----------------------------------------------------------------+
| The license and distribution terms for this file may be         |
| found in the file LICENSE in this distribution or at            |
|                                                                 |
| http://www.rtems.org/license/LICENSE.                           |
|                                                                 |
+-----------------------------------------------------------------+
| this file contains the networking driver                        |
\*===============================================================*/

/*
 * nimu_eth.c
 *
 * Copyright (C) 2011 Texas Instruments Incorporated - http://www.ti.com/
 *
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

/**
 *   @file  nimu_eth.c
 *
 *   @brief
 *  Ethernet Packet Driver rewritten using the NIMU Packet
 *  Architecture guidelines.
 *
 *  Note: The NDK nimu driver interface is built based on the examples from the
 *  PDK. Please refer to PDK examples
 *  (<pdk_install_dir>\packages\ti\drv\pa\example\emacExample) directory to get
 *  the knowledge on the QMSS, CPPI, PA LLD configurations/programs.
 *
 *  Revised for the purpose of rtems porting: Max LEE 2016.12.26
 */


#include <KeyStone_common.h>

#include "nimu_internalX.h"
#include "ethernet.h"
#if NETDBG
#define platform_write printf
#else
#define platform_write
#endif

/* CPPI/QMSS Handles used by the application */
#pragma DATA_SECTION(gPaTxQHnd,       ".nimu_eth_ll2");
#pragma DATA_SECTION(gTxReturnQHnd, ".nimu_eth_ll2");
#pragma DATA_SECTION(gTxFreeQHnd, ".nimu_eth_ll2");
#pragma DATA_SECTION(gRxFreeQHnd, ".nimu_eth_ll2");
#pragma DATA_SECTION(gRxQHnd, ".nimu_eth_ll2");
Qmss_QueueHnd gPaTxQHnd [NUM_PA_TX_QUEUES], gTxReturnQHnd, gTxFreeQHnd, gRxFreeQHnd, gRxQHnd, gTxCmdReturnQHnd, gTxCmdFreeQHnd;

/* Queues used */
/* PA command response queue handle */
#pragma DATA_SECTION(gPaCfgCmdRespQHnd, ".nimu_eth_ll2");
Qmss_QueueHnd                           gPaCfgCmdRespQHnd;

/* CPPI Handles */
#pragma DATA_SECTION(gRxFlowHnd, ".nimu_eth_ll2");
Cppi_FlowHnd                            gRxFlowHnd;

#pragma DATA_SECTION(gPaL2Handles, ".nimu_eth_ll2");
#pragma DATA_SECTION(gPaL3Handles, ".nimu_eth_ll2");
#pragma DATA_SECTION(gPaL4Handles, ".nimu_eth_ll2");
paHandleL2L3_t                          gPaL2Handles[MAX_NUM_L2_HANDLES];
paHandleL2L3_t                          gPaL3Handles[MAX_NUM_L3_HANDLES];
paHandleL4_t                            gPaL4Handles[MAX_NUM_L4_HANDLES];

#pragma DATA_SECTION(gHiPriAccumList, ".nimu_eth_ll2");
#pragma DATA_ALIGN (gHiPriAccumList, 128)
#define MAX_HI_PRI_ACCUM_LIST_SIZE      32
uint32_t                                  gHiPriAccumList[MAX_HI_PRI_ACCUM_LIST_SIZE*2];

#pragma DATA_SECTION(gAccChannelNum, ".nimu_eth_ll2");
uint32_t                                  gAccChannelNum;

/* Various stats  */
#pragma DATA_SECTION(gTxCounter, ".nimu_eth_ll2");
#pragma DATA_SECTION(gRxCounter, ".nimu_eth_ll2");
#pragma DATA_SECTION(gTxDropCounter, ".nimu_eth_ll2");
#pragma DATA_SECTION(gRxDropCounter, ".nimu_eth_ll2");
uint32_t gTxCounter=0, gRxCounter=0, gTxDropCounter = 0, gRxDropCounter=0;

///////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////
/* TX port number */
#pragma DATA_SECTION(gTxPort, ".nimu_eth_ll2");
uint32_t gTxPort;


/**********************************************************************
 ************************** Global Variables **************************
 **********************************************************************/
uint32_t    coreKey [NUM_CORES];
NETIF_DEVICE *GLOBAL_PTR_NET_DEVICE;

/**
 *  @b c6678_emac_start
 *  @n
 *  The function is used to initialize and start the EMAC
 *  controller and device.
 *
 *  @param[in]  ptr_net_device
 *      NETIF_DEVICE structure pointer.
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int c6678_emac_start()
{
	NETIF_DEVICE* ptr_net_device = GLOBAL_PTR_NET_DEVICE;
    EMAC_DATA*      ptr_pvt_data;
    paMacAddr_t     broadcast_mac_addr  = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    paEthInfo_t     ethInfo             = { { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },     /* Src mac = dont care */
                                            { 0x10, 0x11, 0x12, 0x13, 0x14, 0x15 },     /* Default Dest mac */
                                              0,                                        /* vlan = dont care */
                                              0,                                        /* ignore ether type */
                                              0                                         /* MPLS tag = don't care */
                                          };
    paRouteInfo_t   routeInfo           = { pa_DEST_HOST,           /* Route a match to the host */
                                            0,                      /* Flow ID 0 */
                                            0,                      /* Destination queue */
                                            -1,                     /* Multi route disabled */
                                            0xaaaaaaaa,             /* SwInfo 0 */
                                            0,                      /* SwInfo 1 is dont care */
                                            0,                      /* customType = pa_CUSTOM_TYPE_NONE */         \
                                            0,                      /* customIndex: not used */        \
                                            0,                      /* pkyType: for SRIO only */       \
                                            NULL                    /* No commands */
                                          };

    /* Get the pointer to the private data */
    ptr_pvt_data = (EMAC_DATA *)ptr_net_device->pvt_data;

    /* Setup Tx */
    if (Setup_Tx () != 0)
    {
        platform_write ("Tx setup failed \n");
        return -1;
    }

    /* Setup Rx */
    if (Setup_Rx () != 0)
    {
    	platform_write ("Rx setup failed \n");
        return -1;
    }

    memcpy (&ethInfo.dst[0],  ptr_pvt_data->pdi.bMacAddr,  sizeof(paMacAddr_t));
    /* Set up the MAC Address LUT*/
    if (Add_MACAddress (&ethInfo, &routeInfo) != 0)
    {
    	platform_write ("Add_MACAddress failed \n");
        return -1;
    }

    memcpy (&ethInfo.dst[0],  broadcast_mac_addr,  sizeof(paMacAddr_t));
    /* Set up the MAC Address LUT for Broadcast */
    if (Add_MACAddress (&ethInfo, &routeInfo) != 0)
    {
    	platform_write ("Add_MACAddress failed \n");
        return -1;
    }
    /* Verify the Tx and Rx Initializations */
    if (Verify_Init () != 0)
    {
    	platform_write ("Warning:Queue handler Verification failed \n");
    }

    /* Copy the MAC Address into the network interface object here. */
    memcpy(&ptr_net_device->mac_address[0], &ptr_pvt_data->pdi.bMacAddr[0], 6);

    /* Set the 'initial' Receive Filter */
    //ptr_pvt_data->pdi.Filter = ETH_PKTFLT_MULTICAST;
    ptr_pvt_data->pdi.TxFree = 1;

    ///////////////////////////////////////////////////////////////////////
    // /* 2017/3/1 */ Open the packet buffer manager
    PBM_open();
    ///////////////////////////////////////////////////////////////////////

    return 0;
}
#if 0
/**
 *  @b c6678_emac_stop
 *  @n
 *  The function is used to de-initialize and stop the EMAC
 *  controller and device.
 *
 *  @param[in]  ptr_net_device
 *      NETIF_DEVICE structure pointer.
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int c6678_emac_stop (NETIF_DEVICE* ptr_net_device)
{
    EMAC_DATA*  ptr_pvt_data;
    uint16_t         count, i;
    Cppi_Desc*     pCppiDesc;
    uint8_t*         bufaddr;
    uint32_t         bufLen;

    ///////////////////////////////////////////////////////////////////////
    ISR_Level level;
    /* Disable event, interrupt */
#if 0
    EventCombiner_disableEvent(PLATFORM_ETH_EVENTID);
    Hwi_disableInterrupt(PLATFORM_ETH_INTERRUPT);
#endif
    _ISR_Disable( level );
    ///////////////////////////////////////////////////////////////////////

    /* Get the pointer to the private data */
    ptr_pvt_data = (EMAC_DATA *)ptr_net_device->pvt_data;
    Osal_nimuFree((Ptr)ptr_pvt_data, platform_roundup(sizeof(EMAC_DATA), PLATFORM_CACHE_LINE_SIZE));

    count = Qmss_getQueueEntryCount(gRxQHnd);

    /* Free the packet data buffer associated with the Rx Descriptor */
    for (i=0; i < count; i++) {
        /* Need to free up the PBM handle */
        /* free the PBM packet */

        /* Get a free descriptor from the global free queue we setup
         * during initialization.
         */
        if ((QMSS_QPOP (gRxQHnd, QHANDLER_QPOP_FDQ_ATTACHEDBUF, (Cppi_HostDesc **)&pCppiDesc)) != NULL)
        {
        	_ISR_Enable( level );
            return -1;
        }

        /* Get the Address and Length for Free */
        Cppi_getOriginalBufInfo(Cppi_DescType_HOST, pCppiDesc, &bufaddr, &bufLen);
        Osal_nimuFree((Ptr)bufaddr, bufLen);
    }

    count = Qmss_getQueueEntryCount(gRxFreeQHnd);
    /* Free the packet data buffer associated with the Rx Descriptor */
    for (i=0; i < count; i++) {
        /* Need to free up the PBM handle */
        /* free the PBM packet */

        /* Get a free descriptor from the global free queue we setup
         * during initialization.
         */
        if ((QMSS_QPOP (gRxFreeQHnd, QHANDLER_QPOP_FDQ_ATTACHEDBUF, (Cppi_HostDesc **)&pCppiDesc)) != NULL)
        {
        	_ISR_Enable( level );
            return -1;
        }

        /* Get the Address and Length for Free */
        Cppi_getOriginalBufInfo(Cppi_DescType_HOST, pCppiDesc, &bufaddr, &bufLen);
        Osal_nimuFree ((Ptr)bufaddr, bufLen);
    }

    /* tear down queues and dmas */
    res_mgr_stop_qmss();
    Qmss_queueClose (gPaCfgCmdRespQHnd);
    Qmss_queueClose (gTxFreeQHnd);
    Qmss_queueClose (gRxFreeQHnd);
    Qmss_queueClose (gRxQHnd);
    Qmss_queueClose (gTxReturnQHnd);

    for (i = 0; i < NUM_PA_TX_QUEUES; i++)
    	Qmss_queueClose (gPaTxQHnd[i]);

	res_mgr_stop_cppi(CPPI_CFG_PASS);

	///////////////////////////////////////////////////////////////////////
	_ISR_Enable( level );
	///////////////////////////////////////////////////////////////////////
    /* EMAC Controller has been stopped. */
    return 0;
}
#endif

static Bool gIsPingListUsed = 0;

/*
 * Keep in mind: the more rapid, the better!!!
 */
interrupt void GE_Message_ISR()
{
	NETIF_DEVICE*       ptr_net_device = GLOBAL_PTR_NET_DEVICE;
    uint32_t            protocol, pktLen;
    uint8_t*            pBuffer;
    Cppi_HostDesc*      pHostDesc;
    PBM_Handle          hPkt;
    Cppi_Desc*          pCppiDesc;
    uint32_t            count, i;
    uint32_t*           ListAddress;
    EMAC_DATA*          ptr_pvt_data;
    PBM_Pkt*            rx_pbm_pkt;
    void*               key;
    void*               accum_list_ptr;

#if 0
    /* Disable the interrupt */
    coreKey [CSL_chipReadReg (CSL_CHIP_DNUM)] =  Hwi_disableInterrupt(PLATFORM_ETH_INTERRUPT); //Hwi_disable();
#endif
    /* Disable the interrupt */
    coreKey [CSL_chipReadReg (CSL_CHIP_DNUM)] = _ISR_Get_level();
    _ISR_Disable( coreKey [CSL_chipReadReg (CSL_CHIP_DNUM)] );

    /* Begin Critical Section before accessing shared resources. */
    key = Osal_qmssCsEnter ();

   /* Get the pointer to the private data */
    ptr_pvt_data = (EMAC_DATA *)ptr_net_device->pvt_data;

    if (!gIsPingListUsed){
        accum_list_ptr = (void *)&gHiPriAccumList[0];
    }
    else {
        accum_list_ptr = (void *)&gHiPriAccumList[MAX_HI_PRI_ACCUM_LIST_SIZE];
    }

    /* Invalidate cache if needed --
     *   if accumulator is in DDR then INV L2.
     *   if accumulator is in shared RAM (MSMC) invalidate L1
     */
    if((uint32_t)(gHiPriAccumList) & EMAC_EXTMEM ){
        CACHE_invL2(accum_list_ptr, sizeof(gHiPriAccumList)/2, CACHE_WAIT);
    }

    if ((uint32_t)(gHiPriAccumList) & EMAC_MSMCSRAM ) {
        CACHE_invL1d(accum_list_ptr, sizeof(gHiPriAccumList)/2, CACHE_WAIT);
     }

    i           = MAX_HI_PRI_ACCUM_LIST_SIZE - 1 - (RX_INT_THRESHOLD);
    ListAddress = (uint32_t* )Convert_CoreLocal2GlobalAddr((uint32_t) &gHiPriAccumList[i]);

    /* Process ISR.
     *
     * Get the number of entries in accumulator list.
     * The hardware enqueues data alternatively to Ping/Pong buffer lists in
     * the accumulator. Hence, we need to track which list (Ping/Pong)
     * we serviced the last time and accordingly process the other one
     * this time around.
     */
     if (!gIsPingListUsed)
     {
        /* Serviced Pong list last time. So read off the Ping list now */
        count   =   ListAddress[0];
     }
     else
     {
        /* Serviced Pong list last time. So read off the Ping list now */
        count   =   ListAddress[RX_INT_THRESHOLD + 1];
     }

    /* Nothing to receive, so return... */
    if (count == 0) {
        /* End Critical Section */
        Osal_qmssCsExit (key);
#if 0
        Hwi_restoreInterrupt(PLATFORM_ETH_INTERRUPT, coreKey [CSL_chipReadReg (CSL_CHIP_DNUM)]);
#endif
        _ISR_Enable( coreKey [CSL_chipReadReg (CSL_CHIP_DNUM)] );
        /* Clear INTD */
        Qmss_ackInterrupt(PA_ACC_CHANNEL_NUM, 1);
        Qmss_setEoiVector(Qmss_IntdInterruptType_HIGH, PA_ACC_CHANNEL_NUM);
        return ; /* Not enough packets are received */
    }

    /* Process all the Results received
     *
     * Skip the first entry in the list that contains the
     * entry count and proceed processing results.
     */
    for (i = 1; i <= count; i ++)
    {
        gRxCounter ++;

        /* Get the result descriptor.
         * The hardware enqueues data alternatively to Ping/Pong buffer lists in
         * the accumulator. Hence, we need to track which list (Ping/Pong)
         * we serviced the last time and accordingly process the other one
         * this time around.
         */
        if (!gIsPingListUsed)
        {
            /* Serviced Pong list last time. So read off the Ping list now */
            pCppiDesc   =   (Cppi_Desc *) ListAddress [i];
        }
        else
        {
            /* Serviced Ping list last time. So read off the Pong list now
             * Skip over Ping list length to arrive at Pong list start.
             */
            pCppiDesc   =   (Cppi_Desc *) ListAddress [i + RX_INT_THRESHOLD + 1];
        }

        /* Descriptor size appended to the address in the last 4 bits.
         * To get the true descriptor size, always mask off the last
         * 4 bits of the address.
         */
        pCppiDesc = (Ptr) ((uint32_t) pCppiDesc & 0xFFFFFFF0);
        pHostDesc = (Cppi_HostDesc *)pCppiDesc;

        /* Invalidate cache based on where the memory is */
        if((uint32_t)(pHostDesc) & EMAC_EXTMEM ) {
            CACHE_invL2((void *) pHostDesc, sizeof(Cppi_HostDesc), CACHE_WAIT);
        }
        if ((uint32_t)(pHostDesc) & EMAC_MSMCSRAM ) {
            CACHE_invL1d((void *)pHostDesc, sizeof(Cppi_HostDesc), CACHE_WAIT);
        }
        if((uint32_t)(pHostDesc->buffPtr) & EMAC_EXTMEM ){
        	CACHE_invL2((void *) pHostDesc->buffPtr, pHostDesc->buffLen, CACHE_WAIT);
        }
        if ((uint32_t)(pHostDesc->buffPtr) & EMAC_MSMCSRAM ) {
        	CACHE_invL1d((void *)pHostDesc->buffPtr, pHostDesc->buffLen, CACHE_WAIT);
        }

        /*
         * We should not see packets too large but check anyways ...
         * Note that we are subtracting off the FCS the switch added to the frame.
         * If its too large then return it to the free queue.
         */
        if ((pHostDesc->buffLen - 4) > (ptr_net_device->mtu + ETHHDR_SIZE)) {
        	/* lets try the next one... we should record this as a too large.... */
        	gRxDropCounter++;
            pHostDesc->buffLen = pHostDesc->origBufferLen;
            QMSS_QPUSH (gRxFreeQHnd, (Ptr)pHostDesc, pHostDesc->buffLen, SIZE_HOST_DESC, Qmss_Location_TAIL);
            continue;
        }

        /* Allocate the PBM packet for the Max MTU size*/
        hPkt = PBM_alloc(1514);
        if (NULL == hPkt) {
        	/* could not get a free NDK packet, maybe the next time around we can... */
			gRxDropCounter++;
	        pHostDesc->buffLen = pHostDesc->origBufferLen;
            QMSS_QPUSH (gRxFreeQHnd, (Ptr)pHostDesc, pHostDesc->buffLen, SIZE_HOST_DESC, Qmss_Location_TAIL);
			continue;
        }

        rx_pbm_pkt = (PBM_Pkt *) hPkt;

        PBM_setDataOffset((PBM_Handle) hPkt, 0);

        /* removing the FCS length the EMAC switch adds  here */
        pktLen    = (pHostDesc->buffLen - 4);

        /* Set to minimum packet size */
        if (pktLen < 60) {
            pktLen = 64;
        }

        PBM_setValidLen((PBM_Handle) hPkt, pktLen);

        /* Handle raw frames separately, i.e. check the
        * Ethernet Protocol type in this packet and see
        * if its a well known ether type. If so, this is normal
        * IP stream, enqueue this is in usual Rx queue and let the
        * stack know that a packet has arrived for it. However, if
        * the Ethernet type in the packet is not a well known one,
        * this could be a custom raw Ethernet packet, enqueue it
        * separately in the Raw Rx queue and notify stack. The Raw
        * Ethernet packets when being handed over are given
        * preferential treatment and are serviced before the normal
        * IP traffic. Hence the 2 queues.
        */
        pBuffer =  (uint8_t* )Convert_CoreLocal2GlobalAddr((uint32_t) pHostDesc->buffPtr);

        /* Extract the Ethernet type from the packet. */
        protocol = ( pBuffer[12] << 8) | pBuffer[13] ;
        protocol = (protocol & 0xFFFFu);

        PBM_setIFRx((PBM_Handle) hPkt, (HANDLE) protocol );

        /* Copy the data buffer received to the allocated PBM packet */
        memcpy((uint8_t* )rx_pbm_pkt->pDataBuffer, (uint8_t* )pBuffer, pktLen) ;
        static int x = 0; printf("ISR %0d\n", ++x);

#if 0
        /* Is it a standard ethernet type? */
        if (protocol != ETHERTYPE_IP && protocol != ETHERTYPE_IPV6 /*&& protocol != ETHERTYPE_VLAN
            && protocol != ETHERTYPE_PPPOECTL && protocol != ETHERTYPE_PPPOEDATA*/ )
        {
            /* This is a raw packet, enqueue in Raw Rx Queue */
            PBMQ_enq( &ptr_pvt_data->pdi.PBMQ_rawrx, (PBM_Handle) hPkt);
        }
        else
        {   /* This is a normal IP packet. Enqueue in Rx Queue */
            PBMQ_enq( &ptr_pvt_data->pdi.PBMQ_rx, (PBM_Handle) hPkt );
        }
#endif
        PBMQ_enq( &ptr_pvt_data->pdi.PBMQ_rx, (PBM_Handle) hPkt );

       /* Free the packet back to the Rx FDQ */
        pHostDesc->buffLen = pHostDesc->origBufferLen;
        QMSS_QPUSH (gRxFreeQHnd, (Ptr)pHostDesc, pHostDesc->buffLen, SIZE_HOST_DESC, Qmss_Location_TAIL);
    }

    ListAddress = (uint32_t *) Convert_CoreLocal2GlobalAddr((uint32_t) &gHiPriAccumList[0]);

    /* Clear the accumulator list and save whether we used Ping/Pong
     * list information for next time around.
     */
     if (!gIsPingListUsed)
     {
        /* Just processed Ping list */
        gIsPingListUsed  =   1;

        i = sizeof (gHiPriAccumList)/2;

        /* Clear the accumulator list after processing */
        memset ((void *) &gHiPriAccumList[0], 0, i);

        if(
            ((uint32_t)(&gHiPriAccumList[0]) & EMAC_EXTMEM ) ||
            ((uint32_t)(&gHiPriAccumList[0]) & EMAC_MSMCSRAM )
          )
        {
            /* This needs to be enabled if gHiPriAccumList is in External Memory or MSMCMEM */
             CACHE_wbL2 ((void *)&gHiPriAccumList[0], i, CACHE_WAIT);
        }
    }
    else
    {
        /* Just processed Pong list */
        gIsPingListUsed  =   0;
        i = sizeof (gHiPriAccumList)/2;

        /* Clear the accumulator list after processing */
        memset ((void *) &gHiPriAccumList[MAX_HI_PRI_ACCUM_LIST_SIZE], 0, i);

        if(
            ((uint32_t)(&gHiPriAccumList[MAX_HI_PRI_ACCUM_LIST_SIZE]) & EMAC_EXTMEM ) ||
            ((uint32_t)(&gHiPriAccumList[MAX_HI_PRI_ACCUM_LIST_SIZE]) & EMAC_MSMCSRAM )
          )
        {
            /* This needs to be enabled if gHiPriAccumList is in External Memory or MSMCMEM */
            CACHE_wbL2((void *) &gHiPriAccumList[MAX_HI_PRI_ACCUM_LIST_SIZE], i, CACHE_WAIT);
        }
    }

     ///////////////////////////////////////////////////////////////////////
    /* Notify RX daemon of pending Rx Ethernet packet */
    //STKEVENT_signal( ptr_pvt_data->pdi.hEvent, STKEVENT_ETHERNET, 1 );
    //extern rtems_id _gRx_id;

	//rtems_bsdnet_event_send(_gRx_id, START_RECEIVE_EVENT);
	///////////////////////////////////////////////////////////////////////

    /* End Critical Section */
    Osal_qmssCsExit (key);
#if 0
    Hwi_restoreInterrupt(PLATFORM_ETH_INTERRUPT, coreKey [CSL_chipReadReg (CSL_CHIP_DNUM)]);
#endif
    _ISR_Enable( coreKey [CSL_chipReadReg (CSL_CHIP_DNUM)] );

    /* Clear INTD */
    Qmss_ackInterrupt(PA_ACC_CHANNEL_NUM, 1);
    Qmss_setEoiVector(Qmss_IntdInterruptType_HIGH, PA_ACC_CHANNEL_NUM);

	return;
}

/**
 *  @b c6678_emac_init
 *  @n
 *  The function is used to initialize and register the EMAC
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int c6678_emac_init (void)
{
    NETIF_DEVICE*       ptr_device;
    EMAC_DATA*  ptr_pvt_data;
    char names[6][5]={"eth0","eth1","eth2","eth3","eth4","eth5"};

    PLATFORM_EMAC_EXT_info emac_info;

    /* Get the core number. */
    uint32_t coreNum = platform_get_coreid();

    uint32_t i;

    /* Allocate memory for the private data */
    ptr_pvt_data    = Osal_nimuMalloc (platform_roundup(sizeof(EMAC_DATA),
    										PLATFORM_CACHE_LINE_SIZE),
    										PLATFORM_CACHE_LINE_SIZE);

    if (ptr_pvt_data == NULL)
    {
    	platform_write ("Error: Unable to allocate private memory data\n");
        return -1;
    }

    /* Initialize the allocated memory block. */
    mmZeroInit (ptr_pvt_data, sizeof(EMAC_DATA));

    /* Initialize the RX Queue */
    PBMQ_init(&ptr_pvt_data->pdi.PBMQ_rx);
    PBMQ_init(&ptr_pvt_data->pdi.PBMQ_rawrx);

    /* Initialize the private data */
    mmZeroInit(&ptr_pvt_data->pdi, sizeof(PDINFO));

    /* Set physical index */
    ptr_pvt_data->pdi.PhysIdx = coreNum;
    //ptr_pvt_data->pdi.hEvent  = hEvent;

    /* MCast List is EMPTY */
    ptr_pvt_data->pdi.MCastCnt    = 0;

#ifndef SIMULATOR_SUPPORT
    for (i = 0; i < PLATFORM_MAX_EMAC_PORT_NUM; i++)
    {
        platform_get_emac_info (i, &emac_info);
        if (emac_info.mode == PLATFORM_EMAC_PORT_MODE_PHY)
        {
            break;
        }
    }
    if (i < PLATFORM_MAX_EMAC_PORT_NUM)
    {
        gTxPort = emac_info.port_num;
        memcpy(ptr_pvt_data->pdi.bMacAddr, emac_info.mac_address, 6);
        unsigned long long *sourceMAC = ptr_pvt_data->pdi.bMacAddr;
        printf("MAC 0x%012llx\n", *sourceMAC);
    }
    else
    {
        platform_write ("Error: Unable to find a TX EMAC port\n");
        return -1;
    }
#else
    {
        gTxPort = 1;
        platform_get_emac_info (1, &emac_info);
        memcpy(ptr_pvt_data->pdi.bMacAddr, emac_info.mac_address, 6);
    }
#endif

    /* Init Logical Device */
    /* ptr_pvt_data->pdi.hEther = hEther; */

    /* Allocate memory for the EMAC. */
    /*  ptr_device needs to be allocated via mmAlloc since NDK frees it during shutdown */
    ptr_device 	    = mmAlloc(sizeof(NETIF_DEVICE));

    if (ptr_device == NULL)
    {
    	platform_write ("Error: Unable to allocate memory for the EMAC\n");
        return -1;
    }

    /* Initialize the allocated memory block. */
    mmZeroInit (ptr_device, sizeof(NETIF_DEVICE));

    /* Populate the Network Interface Object. */
    strcpy (ptr_device->name, names[coreNum]);
    ptr_device->mtu         = ETH_MAX_PAYLOAD - ETHHDR_SIZE;
    ptr_device->pvt_data    = (void *)ptr_pvt_data;
#if 0
    /* Populate the Driver Interface Functions. */
    ptr_device->start       = EmacStart;
    ptr_device->stop        = EmacStop;
    ptr_device->poll        = EmacPoll;
    ptr_device->send        = EmacSend;
    ptr_device->pkt_service = EmacPktService;
    ptr_device->ioctl       = Emacioctl;
    ptr_device->add_header  = NIMUAddEthernetHeader;
#endif
    /* Init PA LLD */
    if (Init_PASS () != 0)
    {
    	platform_write ("PASS init failed \n");
        return -1;
    }
    else
    {
    	platform_write ("PASS successfully initialized \n");
    }

    /* Initialize the CPSW switch */
    if (Init_Cpsw ((uint32_t) ptr_device->mtu,  ptr_pvt_data->pdi.bMacAddr) != 0)
    {
    	platform_write ("Ethernet subsystem init failed \n");
        return -1;
    }
    else
    {
    	platform_write ("Ethernet subsystem successfully initialized \n");
    }
#if 0
    /* Register the device with NIMU */
    if (NIMURegister (ptr_device) < 0)
    {
    	platform_write ("Error: Unable to register the EMAC\n");
        return -1;
    }
#endif
    GLOBAL_PTR_NET_DEVICE = ptr_device;
    platform_write ("Registration of the EMAC Successful, waiting for link up ..\n");

    return 0;

}


///////////////////////////////////////////////////////////////////////
/*
 * 最小化发射装置，只要把内存地址和长度发到这里就可以发出去！！！
 * 完成！！！Max Lee 2017/3/2
 * 注意：调用者必须要自己负责内存管理。
 */
int c6678_emac_send (/*register*/ uint8_t*   buffer, /*register*/ uint32_t   length)
{
	NETIF_DEVICE* ptr_net_device = GLOBAL_PTR_NET_DEVICE;

	//ISR_Level level;
    EMAC_DATA*          ptr_pvt_data;
    Cppi_Desc*          pCppiDesc;
    uint32_t            dataBufferSize, i,  count=0;

    Cppi_HostDesc*      pHostDesc;
    uint32_t			coreNum;
    Ptr					key;

    /* Make sure the driver does not transmit packet less than min. as per the
     * Ethernet standards. */
    if (length < 60)
    	return -1;

    /* Fill out MAC address. */
    //memcpy (buffer + 6, GLOBAL_PTR_NET_DEVICE->mac_address, 6);

    /* Disable all interrupts. */
    //_ISR_Disable( level );

    /* Begin Critical Section before accessing shared resources. */
    key = Osal_cppiCsEnter();

    /* Get the core number. */
    coreNum = platform_get_coreid();

    count = Qmss_getQueueEntryCount (gTxReturnQHnd);

    for (i = 0; i < count; i++)  {
        /* Get a free descriptor from the global free queue we setup
         * during initialization.
         */
        if ((QMSS_QPOP (gTxReturnQHnd, QHANDLER_QPOP_FDQ_ATTACHEDBUF, (Cppi_HostDesc **)&pCppiDesc)) != NULL)
        {
           goto error_cond;
        }
        pHostDesc = (Cppi_HostDesc *)pCppiDesc;

        /* Clear the PBM Handle set in the descriptor  */
        pHostDesc->softwareInfo0 = NULL;

        /* Push descriptor to Tx free queue */
        QMSS_QPUSH     (gTxFreeQHnd, (Ptr)pHostDesc, pHostDesc->buffLen, SIZE_HOST_DESC, Qmss_Location_TAIL);
    }

    /* Get the pointer to the private data */
    ptr_pvt_data = (EMAC_DATA *)ptr_net_device->pvt_data;

    /* We do not do any packet size checks here. If the packet
     * is too big to fit in the MTU configured on the peripheral,
     * then the driver/CSL layer should catch the error.
     */

    ptr_pvt_data->pdi.PhysIdx = coreNum;

    /* Pass the packet to the controller if the transmitter is free. */
    if(ptr_pvt_data->pdi.TxFree )
    {
    	//printf("emac_send %d\n", length);
#if 0
        int jj;
        for (jj = 0; jj < length; ++jj)
        	printf("%02x ",buffer[jj]);
        puts("");
#endif
        //TODO flush
          /* Clean the cache for external/L2 addresses */
          if( ((uint32_t)buffer & EMAC_EXTMEM  ) ||
              ((uint32_t)buffer & EMAC_LL2SRAM ) ) {
#if 0
              OEMCacheClean( (void *)buffer, length );
#endif
          }

          /* Get a free descriptor from the global free queue we setup
           * during initialization.
           */
          if ((QMSS_QPOP (gTxFreeQHnd, QHANDLER_QPOP_FDQ_NO_ATTACHEDBUF, (Cppi_HostDesc **)&pCppiDesc)) != NULL)
          {
                gTxDropCounter++;
                goto error_cond;
          }

          pHostDesc = (Cppi_HostDesc *)pCppiDesc;
          pHostDesc->softwareInfo0 = (uint32_t) buffer;//hPkt;
          dataBufferSize  =   length;

          Cppi_setData (  Cppi_DescType_HOST,
                          (Cppi_Desc *) pCppiDesc,
                          (uint8_t *) Convert_CoreLocal2GlobalAddr((uint32_t)buffer),
                          dataBufferSize
                       );
          Cppi_setPacketLen (Cppi_DescType_HOST, (Cppi_Desc *)pCppiDesc, dataBufferSize);

          count = Qmss_getQueueEntryCount (gTxReturnQHnd);

#ifdef PA_LOOPBACK
          QMSS_QPUSH (gPaTxQHnd[0], pCppiDesc, dataBufferSize, SIZE_HOST_DESC, Qmss_Location_TAIL);
#else
          Cppi_setPSFlags (Cppi_DescType_HOST, (Cppi_Desc *)pCppiDesc, (1<<gTxPort));
          /* Send the packet out the mac. It will loop back to PA if the mac/switch
           * have been configured properly
           */
          QMSS_QPUSH (gPaTxQHnd[8], pCppiDesc, dataBufferSize, SIZE_HOST_DESC, Qmss_Location_TAIL);
#endif
          /* Increment the application transmit counter */
          gTxCounter ++;
    }
    /* Packet has been successfully transmitted. */
    /* End Critical Section */
    Osal_cppiCsExit (key);
    //_ISR_Enable( level );
    return 0;

error_cond:
    /* End Critical Section */
    Osal_cppiCsExit (key);
    //_ISR_Enable( level );
    return -1;
}
#if 1
// Our Max Multicast (Independent of Low-level MAX)
#define ETH_MIN_PAYLOAD  46
/* Maximum Ethernet Payload Size. */
#ifdef _INCLUDE_JUMBOFRAME_SUPPORT
#define ETH_MAX_PAYLOAD  10236
#else
#define ETH_MAX_PAYLOAD  1514
#endif
void
c6678_eth_recvpacket()
{
	NETIF_DEVICE* ptr_net_device = GLOBAL_PTR_NET_DEVICE;

	EMAC_DATA*  ptr_pvt_data;
	PBM_Handle  hPacket;
	PBM_Pkt *ptr_pkt;
	int pktlen;
	static int total;

    /* Get the pointer to the private data */
    ptr_pvt_data = (EMAC_DATA *)ptr_net_device->pvt_data;

	total = PBMQ_count(&ptr_pvt_data->pdi.PBMQ_rx);
    if (total <= 0)
    {
    	return;
    }
    printf("total : %d\n", total);

    while (total > 0)
    {
    	printf("Packet #%d\n", total);
    	total--;

        /* Dequeue a packet from the driver receive queue. */
        hPacket = PBMQ_deq(&ptr_pvt_data->pdi.PBMQ_rx);

        /* Prepare the packet so that it can be passed up the networking stack.
         * If this 'step' is not done the fields in the packet are not correct
         * and the packet will eventually be dropped.  */
        PBM_setIFRx (hPacket, ptr_net_device);

        ptr_pkt = (PBM_Pkt *)hPacket;

        //TODO: just return???/////////////////////////////////////////////////////////////////
        /* Make sure we received a valid packet. */
        if (ptr_pkt == NULL)
            continue;

        /* Basic validations: The received packet should conform to the Ethernet standards */
        if((ptr_pkt->ValidLen < ETH_MIN_PAYLOAD) || (ptr_pkt->ValidLen > ETH_MAX_PAYLOAD))
        {
            printf("c6678_eth_recvpacket: Bad Size; Payload is %d bytes", ptr_pkt->ValidLen);
            goto recvpacket_out;
        }

        unsigned char *pBuffer = (unsigned char *)(ptr_pkt->pDataBuffer + ptr_pkt->DataOffset);

        /* Copy the received packet into an mbuf */
        //memcpy((char *)m->m_data, (char *)(ptr_pkt->pDataBuffer + ptr_pkt->DataOffset), *plen);

        if (0xff != pBuffer[14+20-1]) {
        	printf("Ethernet Frame Length: %d Bytes\n", ptr_pkt->ValidLen);
//            dbgprintf(pBuffer, 12);
//            dbgprintf(pBuffer+14+20-8, 8);
//            dbgprintf(pBuffer+14+20, 4);
            stack_main(pBuffer);
        }

recvpacket_out:
        /* Release the packets memory */
        PBM_free( ptr_pkt );
    }


}
#endif
///////////////////////////////////////////////////////////////////////

/* Nothing past this point */

/** ============================================================================
 *   @n@b Setup_Tx
 *
 *   @b Description
 *   @n This API sets up all relevant data structures and configuration required
 *      for sending data to PASS/Ethernet. It sets up a Tx free descriptor queue,
 *      PASS Tx queues required for send.
 *
 *   @param[in]
 *   @n None
 *
 *   @return    int32_t
 *              -1      -   Error
 *              0       -   Success
 * =============================================================================
 */
int32_t Setup_Tx (void)
{
    uint8_t			isAllocated, i;
    Qmss_Queue      qRetInfo;
    Ptr             pCppiDesc;
    uint32_t        mySWInfo[] = {0x00000000, 0x00000000, 0x00000000};

    /* Open all Transmit (Tx) queues.
     *
     * These queues are used to send data to PA PDSP/CPSW.
     */
    for (i = 0; i < NUM_PA_TX_QUEUES; i ++)
    {

        if ((gPaTxQHnd[i] = Qmss_queueOpen (Qmss_QueueType_PASS_QUEUE, QMSS_PARAM_NOT_SPECIFIED, &isAllocated)) < 0)
        {
        	platform_write ("Error opening PA Tx queue \n");
            return -1;
        }
    }

    /* Open a Tx Free Descriptor Queue (Tx FDQ).
     *
     * This queue will be used to hold Tx free decriptors that can be filled
     * later with data buffers for transmission onto wire.
     */
    if ((gTxFreeQHnd = Qmss_queueOpen (Qmss_QueueType_STARVATION_COUNTER_QUEUE, QMSS_PARAM_NOT_SPECIFIED, &isAllocated)) < 0)
    {
    	platform_write ("Error opening Tx Free descriptor queue \n");
        return -1;
    }

    /* Open a Tx Return Descriptor Queue (Tx Transmit CompletionQ).
     *
     * This queue will be used to hold Tx completed decriptors that can be filled
     * later with data buffers for transmission onto wire.
     */
    if ((gTxReturnQHnd = Qmss_queueOpen (Qmss_QueueType_GENERAL_PURPOSE_QUEUE, QMSS_PARAM_NOT_SPECIFIED, &isAllocated)) < 0)
    {
    	platform_write ("Error opening Tx Return descriptor queue \n");
        return -1;
    }

    qRetInfo = Qmss_getQueueNumber (gTxReturnQHnd);

    /* Attach some free descriptors to the Tx free queue we just opened. */
    for (i = 0; i < NIMU_NUM_TX_DESC; i++)
    {
        /* Get a free descriptor from the global free queue we setup
         * during initialization.
         */
        if ((QMSS_QPOP (res_mgr_qmss_get_freeq(), QHANDLER_QPOP_FDQ_NO_ATTACHEDBUF, (Cppi_HostDesc **)&pCppiDesc )) != NULL)
        {
            break;
        }

        /* Setup the Completion queue:
         *
         * Setup the return policy for this desc to return to the completion q we just
         * setup instead of the global free queue.
         */
        Cppi_setReturnQueue ((Cppi_DescType) Cppi_DescType_HOST, pCppiDesc, qRetInfo);

        /* Software info is inteded to hold the PBM Pkt Handle for buffer management */
        mySWInfo[1] = i;
        Cppi_setSoftwareInfo (Cppi_DescType_HOST, pCppiDesc, (uint8_t *) mySWInfo);

        /* Push descriptor to Tx free queue */
        QMSS_QPUSHDESCSIZE (gTxFreeQHnd, pCppiDesc, SIZE_HOST_DESC);
    }
    if (i != NIMU_NUM_TX_DESC)
    {
    	platform_write ("Error allocating Tx free descriptors \n");
        return -1;
    }

    /* Open a Tx Command Free Descriptor Queue (Tx CMD FDQ).
     *
     * This queue will be used to hold Tx Command free decriptors
     */
    if ((gTxCmdFreeQHnd = Qmss_queueOpen (Qmss_QueueType_STARVATION_COUNTER_QUEUE, QMSS_PARAM_NOT_SPECIFIED, &isAllocated)) < 0)
    {
    	platform_write ("Error opening Tx Cmd Free descriptor queue \n");
        return -1;
    }

    /* Open a Tx Cmd Return Descriptor Queue (Tx Cmd Completion Queue).
     *
     * This queue will be used to hold Tx command completed decriptors
     */
    if ((gTxCmdReturnQHnd = Qmss_queueOpen (Qmss_QueueType_GENERAL_PURPOSE_QUEUE, QMSS_PARAM_NOT_SPECIFIED, &isAllocated)) < 0)
    {
    	platform_write ("Error opening Tx Return descriptor queue \n");
        return -1;
    }

    qRetInfo = Qmss_getQueueNumber (gTxCmdReturnQHnd);

    /* Attach some free descriptors to the Tx CMD free queue we just opened. */
    for (i = 0; i < NIMU_MAX_NUM_TX_CMD_DESC; i++)
    {
        /* Get a free descriptor from the global free queue we setup
         * during initialization.
         */
        if ((QMSS_QPOP (res_mgr_qmss_get_freeq(), QHANDLER_QPOP_FDQ_NO_ATTACHEDBUF, (Cppi_HostDesc **)&pCppiDesc )) != NULL)
        {
            break;
        }

        /* Setup the Completion queue:
         *
         * Setup the return policy for this desc to return to the completion q we just
         * setup instead of the global free queue.
         */
        Cppi_setReturnQueue ((Cppi_DescType) Cppi_DescType_HOST, pCppiDesc, qRetInfo);


        /* Push descriptor to Tx free queue */
        QMSS_QPUSHDESCSIZE (gTxCmdFreeQHnd, pCppiDesc, SIZE_HOST_DESC);
    }
    if (i != NIMU_MAX_NUM_TX_CMD_DESC)
    {
    	platform_write ("Error allocating Tx Command free descriptors \n");
        return -1;
    }

    /* All done with Rx configuration. Return success. */
    return 0;
}

/** ============================================================================
 *   @n@b Setup_Rx
 *
 *   @b Description
 *   @n This API sets up all relevant data structures and configuration required
 *      for receiving data from PASS/Ethernet. It sets up a Rx free descriptor queue
 *      with some empty pre-allocated buffers to receive data, and an Rx queue
 *      to which the Rxed data is streamed for the example application. This API
 *      also sets up the QM high priority accumulation interrupts required to
 *      receive data from the Rx queue.
 *
 *   @param[in]
 *   @n None
 *
 *   @return    int32_t
 *              -1      -   Error
 *              0       -   Success
 * =============================================================================
 */
int32_t Setup_Rx ()
{
    int32_t				result;
    //int32_t				vectId;
    uint8_t             isAllocated, accChannelNum, i;
    uint16_t			numAccEntries, intThreshold, pktLen;
    Qmss_Queue          rxFreeQInfo, rxQInfo;
    Ptr                 pCppiDesc;
    Qmss_AccCmdCfg      accCfg;
    Cppi_RxFlowCfg      rxFlowCfg;
    //int16_t             eventId;
    Ptr                 pDataBuffer;
    uint32_t            mySWInfo[] = {0x11112222, 0x33334444};
    uint32_t            ListAddress;

    pktLen = 1500/*ptr_net_device->mtu*/ + ETHHDR_SIZE + 4; /* ETh Header + 4 bytes of FCS */

    i           = MAX_HI_PRI_ACCUM_LIST_SIZE - 1 - (RX_INT_THRESHOLD);
    ListAddress = Convert_CoreLocal2GlobalAddr((uint32_t) &gHiPriAccumList[i]);

    /* Open a Receive (Rx) queue.
     *
     * This queue will be used to hold all the packets received by PASS/CPSW
     *
     * Open the next available High Priority Accumulation queue for Rx.
     */
    if ((gRxQHnd = Qmss_queueOpen (Qmss_QueueType_HIGH_PRIORITY_QUEUE, QMSS_PARAM_NOT_SPECIFIED, &isAllocated)) < 0)
    {
    	platform_write ("Error opening a High Priority Accumulation Rx queue \n");
        return -1;
    }

    rxQInfo = Qmss_getQueueNumber (gRxQHnd);

    /* Setup high priority accumulation interrupts on the Rx queue.
     *
     * Let's configure the accumulator with the following settings:
     *      (1) Interrupt pacing disabled.
     *      (2) Interrupt on every received packet
     */
    intThreshold    =   RX_INT_THRESHOLD;
    numAccEntries   =   (intThreshold + 1) * 2;
    accChannelNum   =   PA_ACC_CHANNEL_NUM;

    /* Initialize the accumulator list memory */
    memset ((void *) ListAddress, 0, numAccEntries * 4);

    /* Ensure that the accumulator channel we are programming is not
     * in use currently.
     */
    result = Qmss_disableAccumulator (Qmss_PdspId_PDSP1, accChannelNum);
    if (result != QMSS_ACC_SOK && result != QMSS_ACC_CHANNEL_NOT_ACTIVE)
    {
    	platform_write ("Error Disabling high priority accumulator for channel : %d error code: %d\n",
                      accChannelNum, result);
        return -1;
    }

    /* Setup the accumulator settings */
    accCfg.channel             =   accChannelNum;
    accCfg.command             =   Qmss_AccCmd_ENABLE_CHANNEL;
    accCfg.queueEnMask         =   0;
    accCfg.listAddress         =   ListAddress;
    accCfg.queMgrIndex         =   gRxQHnd;
    accCfg.maxPageEntries      =   (intThreshold + 1); /* Add an extra entry for holding the entry count */
    accCfg.timerLoadCount      =   40;
    accCfg.interruptPacingMode =   Qmss_AccPacingMode_LAST_INTERRUPT;
    accCfg.listEntrySize       =   Qmss_AccEntrySize_REG_D;
    accCfg.listCountMode       =   Qmss_AccCountMode_ENTRY_COUNT;
    accCfg.multiQueueMode      =   Qmss_AccQueueMode_SINGLE_QUEUE;

    /* Program the accumulator */
    if ((result = Qmss_programAccumulator (Qmss_PdspId_PDSP1, &accCfg)) != QMSS_ACC_SOK)
    {
    	platform_write ("Error Programming high priority accumulator for channel : %d queue : %d error code : %d\n",
                        accCfg.channel, accCfg.queMgrIndex, result);
        return -1;
    }

    /* Register interrupts for the system event corresponding to the
     * accumulator channel we are using.
     */
#if 0
    /* System event n - Accumulator Channel 0 */
    eventId     	=   PLATFORM_ETH_EVENTID;

    /* Pick a interrupt vector id to use */
    vectId          =   PLATFORM_ETH_INTERRUPT;

    platform_write ("Ethernet eventId : %d and vectId (Interrupt) : %d \n", eventId, vectId);

    /* Register our ISR handle for this event */
    EventCombiner_dispatchPlug (eventId, (EventCombiner_FuncPtr)EmacRxPktISR, (UArg)ptr_net_device, TRUE);
    EventCombiner_enableEvent(eventId);

    /* Map the event id to hardware interrupt 7. */
    Hwi_eventMap(vectId, eventId >> 5);

    /* Enable interrupt */
    Hwi_enableInterrupt(vectId);
#endif
    //rtems_isr_entry old_handle;

    /* Map the event id to hardware interrupt 7. */
    //gpCGEM_regs->INTMUX1 = (CSL_GEM_QM_INT_HIGH_N<<CSL_CGEM_INTMUX1_INTSEL7_SHIFT);

    /* Enable interrupt */
	//CPU_interrupt_enable( 1 << 7 );

    /* Register our ISR handle for this event */
	//rtems_interrupt_catch(EmacRxPktISR, 7, &old_handle);

    /* Open a Rx Free Descriptor Queue (Rx FDQ).
     *
     * This queue will hold all the Rx free descriptors. These descriptors will be
     * used by the PASS CPDMA to hold data received via CPSW.
     */
    if ((gRxFreeQHnd = Qmss_queueOpen (Qmss_QueueType_STARVATION_COUNTER_QUEUE, QMSS_PARAM_NOT_SPECIFIED, &isAllocated)) < 0)
    {
    	platform_write ("Error opening Rx Free descriptor queue \n");
        return -1;
    }
    rxFreeQInfo = Qmss_getQueueNumber (gRxFreeQHnd);

    /* Attach some free descriptors to the Rx free queue we just opened. */
    for (i = 0; i < NIMU_NUM_RX_DESC; i++)
    {
        /* Get a free descriptor from the global free queue we setup
         * during initialization.
         */
        if ((QMSS_QPOP (res_mgr_qmss_get_freeq(), QHANDLER_QPOP_FDQ_NO_ATTACHEDBUF, (Cppi_HostDesc **)&pCppiDesc)) != NULL)
        {
            break;
        }

        pktLen = platform_roundup(pktLen, PLATFORM_CACHE_LINE_SIZE);
        if ((pDataBuffer = Osal_nimuMalloc (pktLen, PLATFORM_CACHE_LINE_SIZE)) == NULL)
        {
        	platform_write ("Error allocating memory for Rx data buffer \n");
            break;
        }

        /* Populate the Rx free descriptor with the buffer we just allocated. */
        Cppi_setData (Cppi_DescType_HOST, pCppiDesc, (uint8_t *)Convert_CoreLocal2GlobalAddr((uint32_t)pDataBuffer), pktLen);

        /* Save original buffer information */
        Cppi_setOriginalBufInfo (Cppi_DescType_HOST, pCppiDesc, (uint8_t *)Convert_CoreLocal2GlobalAddr((uint32_t)pDataBuffer), pktLen);

        /* Setup the Completion queue:
         *
         * Setup the return policy for this desc to return to the free q we just
         * setup instead of the global free queue.
         */
        Cppi_setReturnQueue (Cppi_DescType_HOST, pCppiDesc, rxFreeQInfo);

        Cppi_setSoftwareInfo (Cppi_DescType_HOST, pCppiDesc, (uint8_t *) mySWInfo);

        Cppi_setPacketLen    (Cppi_DescType_HOST, pCppiDesc, pktLen);

        /* Push descriptor to Rx free queue */
        QMSS_QPUSHDESCSIZE (gRxFreeQHnd, pCppiDesc, SIZE_HOST_DESC);
    }
    if (i != NIMU_NUM_RX_DESC)
    {
    	platform_write ("Error allocating Rx free descriptors \n");
        return -1;
    }

    /* Setup a Rx Flow.
     *
     * A Rx flow encapsulates all relevant data properties that CPDMA would
     * have to know in order to successfully receive data.
     */
    /* Initialize the flow configuration */
    memset (&rxFlowCfg, 0, sizeof(Cppi_RxFlowCfg));

    /* Let CPPI pick the next available flow */
    rxFlowCfg.flowIdNum             =   CPPI_PARAM_NOT_SPECIFIED;

    rxFlowCfg.rx_dest_qmgr          =   rxQInfo.qMgr;
    rxFlowCfg.rx_dest_qnum          =   rxQInfo.qNum;
    rxFlowCfg.rx_desc_type          =   Cppi_DescType_HOST;

    rxFlowCfg.rx_ps_location        =   Cppi_PSLoc_PS_IN_DESC;
    rxFlowCfg.rx_psinfo_present     =   1;    /* Enable PS info */

    rxFlowCfg.rx_error_handling     =   0;    /* Drop the packet, do not retry on starvation by default */
    rxFlowCfg.rx_einfo_present      =   1;    /* EPIB info present */

    rxFlowCfg.rx_dest_tag_lo_sel    =   0;    /* Disable tagging */
    rxFlowCfg.rx_dest_tag_hi_sel    =   0;
    rxFlowCfg.rx_src_tag_lo_sel     =   0;
    rxFlowCfg.rx_src_tag_hi_sel     =   0;

    rxFlowCfg.rx_size_thresh0_en    =   0;    /* By default, we disable Rx Thresholds */
    rxFlowCfg.rx_size_thresh1_en    =   0;    /* By default, we disable Rx Thresholds */
    rxFlowCfg.rx_size_thresh2_en    =   0;    /* By default, we disable Rx Thresholds */
    rxFlowCfg.rx_size_thresh0       =   0x0;
    rxFlowCfg.rx_size_thresh1       =   0x0;
    rxFlowCfg.rx_size_thresh2       =   0x0;

    rxFlowCfg.rx_fdq0_sz0_qmgr      =   rxFreeQInfo.qMgr; /* Setup the Receive free queue for the flow */
    rxFlowCfg.rx_fdq0_sz0_qnum      =   rxFreeQInfo.qNum;
    rxFlowCfg.rx_fdq0_sz1_qnum      =   0x0;
    rxFlowCfg.rx_fdq0_sz1_qmgr      =   0x0;
    rxFlowCfg.rx_fdq0_sz2_qnum      =   0x0;
    rxFlowCfg.rx_fdq0_sz2_qmgr      =   0x0;
    rxFlowCfg.rx_fdq0_sz3_qnum      =   0x0;
    rxFlowCfg.rx_fdq0_sz3_qmgr      =   0x0;

    rxFlowCfg.rx_fdq1_qnum          =   rxFreeQInfo.qNum;  /* Use the Rx Queue to pick descriptors */
    rxFlowCfg.rx_fdq1_qmgr          =   rxFreeQInfo.qMgr;
    rxFlowCfg.rx_fdq2_qnum          =   rxFreeQInfo.qNum;  /* Use the Rx Queue to pick descriptors */
    rxFlowCfg.rx_fdq2_qmgr          =   rxFreeQInfo.qMgr;
    rxFlowCfg.rx_fdq3_qnum          =   rxFreeQInfo.qNum;  /* Use the Rx Queue to pick descriptors */
    rxFlowCfg.rx_fdq3_qmgr          =   rxFreeQInfo.qMgr;

    /* Configure the Rx flow */
    if ((gRxFlowHnd = Cppi_configureRxFlow (res_mgr_cppi_get_passhandle(), &rxFlowCfg, &isAllocated)) == NULL)
    {
    	platform_write ("Error configuring Rx flow \n");
        return -1;
    }

    /* All done with Rx configuration. Return success. */
    return 0;
}

/** ============================================================================
 *   @n@b Init_MAC
 *
 *   @b Description
 *   @n This API initializes the CPGMAC Sliver (MAC Port) port.
 *
 *   @param[in]
 *   @n macPortNum      MAC port number for which the initialization must be done.
 *
 *   @param[in]
 *   @n macAddress      MAC address to configure on this port.
 *
 *   @param[in]
 *   @n mtu             Maximum Frame length to configure on this port.
 *
 *   @return
 *   @n None
 * =============================================================================
 */
void Init_MAC (uint32_t macPortNum, uint8_t macAddress[6], uint32_t mtu)
{
    /* Reset MAC Sliver 0 */
    CSL_CPGMAC_SL_resetMac (macPortNum);
    while (CSL_CPGMAC_SL_isMACResetDone (macPortNum) != TRUE);

    /* Setup the MAC Control Register for this port:
     *      (1) Enable Full duplex
     *      (2) Enable GMII
     *      (3) Enable Gigabit
     *      (4) Enable External Configuration. This enables
     *          the "Full duplex" and "Gigabit" settings to be
     *          controlled externally from SGMII
     *      (5) Don't Enable any control/error frames
     *      (6) Enable short frames
     */
    CSL_CPGMAC_SL_enableFullDuplex (macPortNum);
    CSL_CPGMAC_SL_enableGMII (macPortNum);
    CSL_CPGMAC_SL_enableGigabit (macPortNum);
    CSL_CPGMAC_SL_enableExtControl (macPortNum);

    /* Adding these configurations to allow the switch not to ignore any packets */
    CSL_CPGMAC_SL_enableRxCSF(macPortNum);

    /* Configure the MAC address for this port */
    CSL_CPSW_3GF_setPortMACAddress (macPortNum, macAddress);

    /* Configure VLAN ID/CFI/Priority.
     *
     * For now, we are not using VLANs so just configure them
     * to all zeros.
     */
    CSL_CPSW_3GF_setPortVlanReg (macPortNum, 0, 0, 0);

    /* Configure the Receive Maximum length on this port,
     * i.e., the maximum size the port can receive without
     * any errors.
     *
     * Set the Rx Max length to the MTU configured for the
     * interface.
     */
    CSL_CPGMAC_SL_setRxMaxLen (macPortNum, mtu);

    /* Done setting up the MAC port */
    return;
}

/** ============================================================================
 *   @n@b Init_MDIO
 *
 *   @b Description
 *   @n Not supported at moment. MDIO is not simulated yet.
 *
 *   @param[in]
 *   @n None
 *
 *   @return
 *   @n None
 * =============================================================================
 */
void Init_MDIO (void)
{
    /* There is nothing to be done for C6678 EVM */
     return;
}

/** ============================================================================
 *   @n@b Init_Switch
 *
 *   @b Description
 *   @n This API sets up the ethernet switch subsystem and its Address Lookup
 *      Engine (ALE) in "Switch" mode.
 *
 *   @param[in]
 *   @n mtu             Maximum Frame length to configure on the switch.
 *
 *   @return
 *   @n None
 * =============================================================================
 */
void Init_Switch (uint32_t mtu)
{
    CSL_CPSW_3GF_PORTSTAT               portStatCfg;
    uint32_t  rx_max_len = mtu + ETHHDR_SIZE + 4; /* 4 bytes of FCS */

    /* Enable the CPPI port, i.e., port 0 that does all
     * the data streaming in/out of EMAC.
     */
    CSL_CPSW_3GF_enablePort0 ();
    CSL_CPSW_3GF_disableVlanAware ();
    CSL_CPSW_3GF_setPort0VlanReg (0, 0, 0);
    CSL_CPSW_3GF_setPort0RxMaxLen (rx_max_len);

    /* Enable statistics on both the port groups:
     *
     * MAC Sliver ports -   Port 1, Port 2
     * CPPI Port        -   Port 0
     */
    portStatCfg.p0AStatEnable   =   1;
    portStatCfg.p0BStatEnable   =   1;
    portStatCfg.p1StatEnable    =   1;
    portStatCfg.p2StatEnable    =   1;
    CSL_CPSW_3GF_setPortStatsEnableReg (&portStatCfg);

    /* Setup the Address Lookup Engine (ALE) Configuration:
     *      (1) Enable ALE.
     *      (2) Clear stale ALE entries.
     *      (3) Disable VLAN Aware lookups in ALE since
     *          we are not using VLANs by default.
     *      (4) No Flow control
     *      (5) Configure the Unknown VLAN processing
     *          properties for the switch, i.e., which
     *          ports to send the packets to.
     */
    CSL_CPSW_3GF_enableAle ();
    CSL_CPSW_3GF_clearAleTable ();
    CSL_CPSW_3GF_disableAleVlanAware ();
    CSL_CPSW_3GF_disableAleTxRateLimit ();

    /* Setting the Switch MTU Size to more than needed */
    CSL_CPGMAC_SL_setRxMaxLen(0, rx_max_len);
    CSL_CPGMAC_SL_setRxMaxLen(1, rx_max_len);

//#ifdef SIMULATOR_SUPPORT
    CSL_CPSW_3GF_enableAleBypass();
//#endif
    /* Done with switch configuration */
    return;
}


/** ============================================================================
 *   @n@b Switch_update_addr
 *
 *   @b Description
 *   @n This API add/delete entries in the Address Lookup Engine (ALE) in "Switch" mode.
 *
 *   @param[in]
 *   @n portNum         Switch port number.

 *   @param[in]
 *   @n macAddress      MAC address to configure on the switch.
 *
 *   @param[in]
 *   @n add             0:add; 1:delete.
 *
 *   @return
 *   @n None
 *
 *   @Note  It supports "add" operation only now.
 * =============================================================================
 */
int Switch_update_addr (uint32_t portNum, uint8_t macAddress[6], uint16_t add)
{
    CSL_CPSW_3GF_ALE_PORTCONTROL        alePortControlCfg;
#ifdef SIMULATOR_SUPPORT
    uint32_t                              i;
    CSL_CPSW_3GF_ALE_UNICASTADDR_ENTRY  ucastAddrCfg;
#endif


    /* Configure the address in "Learning"/"Forward" state */
    alePortControlCfg.portState             =   ALE_PORTSTATE_FORWARD;
    alePortControlCfg.dropUntaggedEnable    =   0;
    alePortControlCfg.vidIngressCheckEnable =   0;
    alePortControlCfg.noLearnModeEnable     =   0;
    alePortControlCfg.mcastLimit            =   0;
    alePortControlCfg.bcastLimit            =   0;

    CSL_CPSW_3GF_setAlePortControlReg (portNum, &alePortControlCfg);

#ifdef SIMULATOR_SUPPORT
    /* Program the ALE with the MAC address.
     *
     * The ALE entries determine the switch port to which any
     * matching received packet must be forwarded to.
     */
    /* Get the next free ALE entry to program */
    for (i = 0; i < CSL_CPSW_3GF_NUMALE_ENTRIES; i++)
    {
        if (CSL_CPSW_3GF_getALEEntryType (i) == ALE_ENTRYTYPE_FREE)
        {
            /* Found a free entry */
            break;
        }
    }
    if (i == CSL_CPSW_3GF_NUMALE_ENTRIES)
    {
        /* No free ALE entry found. return error. */
        return -1;
    }
    else
    {
        /* Found a free ALE entry to program our MAC address */
        memcpy (ucastAddrCfg.macAddress, macAddress, 6);    // Set the MAC address
        ucastAddrCfg.ucastType      =      ALE_UCASTTYPE_UCAST_NOAGE;   // Add a permanent unicast address entryALE_UCASTTYPE_UCAST_NOAGE.
        ucastAddrCfg.secureEnable   =      FALSE;
        ucastAddrCfg.blockEnable    =      FALSE;
        ucastAddrCfg.portNumber     =      portNum;   // Add the ALE entry for this port

        /* Setup the ALE entry for this port's MAC address */
        CSL_CPSW_3GF_setAleUnicastAddrEntry (i, &ucastAddrCfg);
    }
#endif
    /* Done with upading address */
    return 0;
}

/** ============================================================================
 *   @n@b Verify_Init
 *
 *   @b Description
 *   @n This API initializes the CPPI LLD, opens the PASS CPDMA and opens up
 *      the Tx, Rx channels required for data transfers.
 *
 *   @param[in]
 *   @n None
 *
 *   @return    int32_t
 *              -1      -   Error
 *              0       -   Success
 * =============================================================================
 */
int32_t Verify_Init (void)
{
    int32_t count, returnVal = 0, i;
    int32_t num_tx_desc = NIMU_NUM_TX_DESC;
    int32_t num_rx_desc = NIMU_NUM_RX_DESC;
    int32_t max_queue_number = QMSS_MAX_LOW_PRIORITY_QUEUE          +
                             QMSS_MAX_PASS_QUEUE                    +
                             QMSS_MAX_LOW_PRIORITY_QUEUE         	+
                             QMSS_MAX_PASS_QUEUE                 	+
                             QMSS_MAX_INTC_QUEUE                 	+
                             QMSS_MAX_SRIO_QUEUE                 	+
                             QMSS_MAX_HIGH_PRIORITY_QUEUE        	+
                             QMSS_MAX_STARVATION_COUNTER_QUEUE   	+
                             QMSS_MAX_INFRASTRUCTURE_QUEUE       	+
                             QMSS_MAX_TRAFFIC_SHAPING_QUEUE      	+
                             QMSS_MAX_GENERAL_PURPOSE_QUEUE      	;

    /*Verify if we got NIMU_NUM_TX_DESC Tx Free Q*/
    if ((count = Qmss_getQueueEntryCount (gTxFreeQHnd)) != num_tx_desc)  {
    	platform_write ("Verify_Init: Expected %d entry count for gTxFreeQHnd queue %d, found %d entries\n", num_tx_desc, gTxFreeQHnd, count);
        returnVal =  -1;
    }

    /* Verify if we got NIMU_NUM_RX_DESC Rx FDQ */
    if ((count = Qmss_getQueueEntryCount (gRxFreeQHnd)) != num_rx_desc)  {
    	platform_write ("Verify_Init: Expected %d entry count for gRxFreeQHnd queue %d, found %d entries\n", num_rx_desc, gRxFreeQHnd, count);
         returnVal = -1;
    }

    /* Verify if we got empty Tx completion Q*/
    if ((count = Qmss_getQueueEntryCount (gTxReturnQHnd)) != 0)  {
    	platform_write ("Verify_Init: Expected 0 entry count for gTxReturnQHnd queue %d, found %d entries\n", gTxReturnQHnd, count);
        returnVal = -1;
    }

    /* Verify if we got NIMU_NUM_RX_DESC Rx FDQ */
    if ((count = Qmss_getQueueEntryCount (gRxQHnd)) != 0)  {
    	platform_write ("Verify_Init: Expected 0 entry count for gRxQHnd= %d, found %d entries\n", gRxQHnd, count);
         returnVal = -1;
    }

    for (i = 0; i < max_queue_number; i++ ) {
        if ( (i == gRxFreeQHnd) || (i == gTxFreeQHnd) || (i == gTxCmdFreeQHnd))
            continue;

        count = Qmss_getQueueEntryCount (i);
        if (count != 0) {
        	platform_write ("Verify_Init: Expected 0 entry count for Queue number = %d, found %d entries\n", i, count);
        }
    }

    /* Verify_Init Done. Return success */
    return (returnVal);
}

/** ============================================================================
 *   @n@b Init_Cpsw
 *
 *   @b Description
 *   @n This API sets up the entire ethernet subsystem and all its associated
 *      components.
 *
 *   @param[in]
 *   @n None
 *
 *   @return
 *   @n None
 * =============================================================================
 */
int32_t Init_Cpsw (uint32_t mtu, uint8_t* myMACAddress)
{
#ifdef SIMULATOR_SUPPORT

    uint8_t bcMACAddress[6] = {0xff,0xff,0xff,0xff,0xff,0xff};
    extern  uint8_t clientMACAddress [6];

    /* Initialize the SGMII/Sliver submodules for the MAC port. */
    Init_SGMII (0);
    Init_MAC (0, myMACAddress, mtu);

    Init_SGMII (1);
    Init_MAC (1, myMACAddress, mtu);

    /* Setup the Ethernet switch finally. */
    Init_Switch (mtu);


   /* Update the ALE Entries and ensure that these are correctly configured.
    * There are 2 Entries created here:
    *  Entry1: My OWN MAC Address goes to Port 0
    *  Entry2: Destination MAC Address is forwarded to Port1
    * If there are more destination MAC Addresses to which packets need to be sent
    * than additional entries need to be configured. */

   /* This is needed only for testing in Simulator*/
   Switch_update_addr(0, myMACAddress, 0);
   Switch_update_addr(1, clientMACAddress, 0);
   Switch_update_addr(1, myMACAddress, 0);  // testing a hybrid between cooked up ping and the original app (cooked up raw message)

   // checking out if adding bc message still work for unicast
   Switch_update_addr(1, bcMACAddress, 0);     // verified needed for BCAST tx

   // add this to see if BC packet response can come into the PA
   Switch_update_addr(0, bcMACAddress, 0);     // verfied needed for BCast Rx

#else

    uint32_t  i;
    uint8_t   backplaneMac[6] = {0x1, 0x1, 0x1, 0x1, 0x1, 0x1}; /* Mask for creating mac address by flipping LSB */
    uint8_t   cppiMac [6] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05};  /* MAC address for (CPPI) Port 0 - we made it up*/


	/* we need to create a mac address for the Backplane Ethernet port
	 * using the Ethernet MAC Adress
	 */
    for (i=0; i < 6; i++) {
        backplaneMac[i] |= myMACAddress[i];
    }
	Init_MAC (0, backplaneMac, mtu);

	/* set the silver port to the stacks ethernet address */
	Init_MAC (1, myMACAddress, mtu);

    /* Setup the Phys by initializing the MDIO - not needed for Simulator*/
    Init_MDIO ();

    /* Setup the Ethernet switch finally. */
    Init_Switch (mtu);

    /* This is a little confusing but different APIs use different numbering */
    Switch_update_addr(0, cppiMac, 0);
    Switch_update_addr(1, backplaneMac, 0);
    Switch_update_addr(2, myMACAddress, 0);
#endif

    /* CPSW subsystem setup done. Return success */
    return 0;
}

/* ============================================================================
 *   @n@b Init_PASS
 *
 *   @b Description
 *   @n This API initializes the PASS/PDSP and opens a queue that the application
 *      can use to receive command responses from the PASS.
 *
 *   @param[in]
 *   @n None
 *
 *   @return    int32_t
 *              -1      -   Error
 *              0       -   Success
 * =============================================================================
 */
int32_t Init_PASS (void)
{
    uint8_t			isAllocated;

    /* Open a PA Command Response Queue.
     *
     * This queue will be used to hold responses from the PA PDSP for all the
     * commands issued by NIMU.
     *
     * This queue is used only when configuring the PA PDSP. We will use it when adding our mac address.
     */
    if ((gPaCfgCmdRespQHnd = Qmss_queueOpen (Qmss_QueueType_GENERAL_PURPOSE_QUEUE, QMSS_PARAM_NOT_SPECIFIED, &isAllocated)) < 0)
    {
    	platform_write ("Error opening a PA Command Response queue \n");
        return -1;
    }

    /* Init done. Return success. */
    return 0;
}


/** ============================================================================
 *   @n@b Add_MACAddress
 *
 *   @b Description
 *   @n This API adds the switch MAC address to the PA PDSP Lookup table. This
 *      ensures that all packets destined for this MAC address get processed
 *      for forwarding to the host.
 *
 *   @param[in]
 *   @n ethInfo         Pointer to PA Ethernet Info table.
 *
 *   @param[in]
 *   @n routeInfo       Pointer to PA Route Info table.
 *
 *   @n None
 *
 *   @return    int32_t
 *              -1      -   Error
 *              0       -   Success
 * =============================================================================
 */
int32_t
Add_MACAddress
(
    paEthInfo_t         *ethInfo,
    paRouteInfo_t       *routeInfo
)
{
    int32_t              j;
    uint16_t			cmdSize;
    Qmss_Queue          cmdReplyQInfo;

    paRouteInfo_t       nFailInfo =     {   pa_DEST_DISCARD,                            /* Toss the packet  */
                                                    0,                                          /* Flow Id = dont care */
                                                    0,                                          /* queue = dont care */
                                                    0,                                          /* mutli route = dont care */
                                                    0,                                          /* swinfo0 = dont care */
                                                    0,                                          /* swinfo1 = dont care */
                                                    0,                                          /* customType = pa_CUSTOM_TYPE_NONE */         \
                                                    0,                                          /* customIndex: not used */        \
                                                    0,                                          /* pkyType: for SRIO only */       \
                                                    NULL                                        /* No commands */
                                                };
    paCmdReply_t        cmdReplyInfo =  {   pa_DEST_HOST,                               /* Replies go to the host */
                                                    0,                                          /* User chosen ID to go to swinfo0 */
                                                    0,                                          /* Destination queue */
                                                    0                                           /* Flow ID */
                                                };
    paReturn_t          retVal;
    paEntryHandle_t     retHandle;
    int32_t				handleType, cmdDest;
    uint32_t            psCmd       =   ((uint32_t)(4 << 5) << 24);
    uint32_t            myswinfo[]  =   {0x11112222, 0x33334444};
    uint32_t            myswinfo_orig[] =   {0x00000000, 0x00000000, 0x00000000};;
    uint8_t*            pCmdDataBuffer;
    Cppi_HostDesc*      pHostDesc = NULL;
    Qmss_Queue          rxQInfo;
    uint32_t            count, cmdbuf_len = 320;
    int32_t             ret_val = 0;

    /* Get a Tx free descriptor to send a command to the PA PDSP */
    if ((QMSS_QPOP (gTxCmdFreeQHnd, QHANDLER_QPOP_FDQ_NO_ATTACHEDBUF, (Cppi_HostDesc **)&pHostDesc )) != NULL)
    {
    	platform_write ("Error obtaining a Tx free descriptor \n");
        return -1;
    }

    /* Allocate a Tx buffer and attach the command info to it. */
    cmdbuf_len = platform_roundup(cmdbuf_len, PLATFORM_CACHE_LINE_SIZE);
    if ((pCmdDataBuffer = (Ptr) Osal_nimuMalloc (cmdbuf_len, PLATFORM_CACHE_LINE_SIZE)) == NULL)
    {
    	platform_write ("Error allocating memory for PA Command data buffer \n");
        return -1;
    }

    /* Populate the Tx free descriptor with the buffer we just allocated. */
    Cppi_setData (Cppi_DescType_HOST, (Cppi_Desc *)pHostDesc, (uint8_t *)Convert_CoreLocal2GlobalAddr((uint32_t)pCmdDataBuffer), cmdbuf_len);

    /* Save original buffer information */
    Cppi_setOriginalBufInfo (Cppi_DescType_HOST, (Cppi_Desc *)pHostDesc, (uint8_t *)Convert_CoreLocal2GlobalAddr((uint32_t)pCmdDataBuffer), cmdbuf_len);

    cmdSize                 =   pHostDesc->buffLen;
    cmdReplyInfo.replyId    =   0x11111111;  /* unique for each add mac command */

    /* Get the PA response queue number and populate the destination queue number
     * in the PA response configuration.
     */
    cmdReplyQInfo           =   Qmss_getQueueNumber (gPaCfgCmdRespQHnd);
    cmdReplyInfo.queue      =   cmdReplyQInfo.qNum;

    /* Setup the Rx queue as destination for the packets */
    rxQInfo                 =   Qmss_getQueueNumber (gRxQHnd);
    routeInfo->queue        =   rxQInfo.qNum;

    retVal  =   Pa_addMac  (res_mgr_get_painstance(),
                            pa_LUT1_INDEX_NOT_SPECIFIED,
                            ethInfo,
                            routeInfo,
                            &nFailInfo,
                            &gPaL2Handles[0],
                            (paCmd_t) pHostDesc->buffPtr,
                            &cmdSize,
                            &cmdReplyInfo,
                            &cmdDest);
    if (retVal != pa_OK)
    {
    	platform_write ("Pa_addMac returned error %d\n", retVal);
        ret_val = -1;
        goto return_fail;
    }


    /* This sets the extended info for descriptors, and this is required so PS info
     * goes to the right spot
     */
    Cppi_setSoftwareInfo (Cppi_DescType_HOST, (Cppi_Desc *)pHostDesc, (uint8_t *)myswinfo);

    /* Set the buffer length to the size used. It will be restored when the descriptor
     * is returned
     */
    Cppi_setPacketLen (Cppi_DescType_HOST, (Cppi_Desc *)pHostDesc, cmdSize);
    pHostDesc->buffLen  =   cmdSize;

    /* Mark the packet as a configuration packet */
    Cppi_setPSData (Cppi_DescType_HOST, (Cppi_Desc *)pHostDesc, (uint8_t *)&psCmd, 4);

    /* Send the command to the PA and wait for the return */
    QMSS_QPUSH    (gPaTxQHnd[cmdDest - pa_CMD_TX_DEST_0],
                    pHostDesc,
                    pHostDesc->buffLen,
                    SIZE_HOST_DESC,
                    Qmss_Location_TAIL
                   );

    /* Poll on the PA response queue to see if response from PA has come */
    for (j = 0; j < 100; j++)
    {
        platform_delaycycles (1000);

        /* Verify if we got empty Tx completion Q*/
        count = Qmss_getQueueEntryCount (gTxCmdReturnQHnd);

        if (count != 0)  {

            if ((QMSS_QPOP (gTxCmdReturnQHnd, QHANDLER_QPOP_FDQ_ATTACHEDBUF, (Cppi_HostDesc **)&pHostDesc)) != NULL)
            {
                ret_val = -1;
                goto return_fail;
            }

            /* Restore the states */
            Cppi_setSoftwareInfo(Cppi_DescType_HOST, (Cppi_Desc *)pHostDesc, (uint8_t *)myswinfo_orig);
            Cppi_setPSData (Cppi_DescType_HOST, (Cppi_Desc *)pHostDesc, NULL, NULL);
            Cppi_setData (Cppi_DescType_HOST, (Cppi_Desc *)pHostDesc, NULL, NULL);
            Cppi_setOriginalBufInfo (Cppi_DescType_HOST, (Cppi_Desc *)pHostDesc, NULL, NULL);
            Cppi_setPacketLen (Cppi_DescType_HOST, (Cppi_Desc *)pHostDesc, NULL);
            pHostDesc->buffLen = 0;

            QMSS_QPUSHDESCSIZE (gTxCmdFreeQHnd, pHostDesc, SIZE_HOST_DESC);
        }

        if (Qmss_getQueueEntryCount (gPaCfgCmdRespQHnd) > 0)
        {

            /* We have a response from PA PDSP for the command we submitted earlier for
             * MAC address addition.
             */
            if ((QMSS_QPOP (gPaCfgCmdRespQHnd, QHANDLER_QPOP_FDQ_NO_ATTACHEDBUF, (Cppi_HostDesc **)&pHostDesc)) != NULL)
            {
                ret_val = -1;
                goto return_fail;
            }

            if (pHostDesc->softwareInfo0 != cmdReplyInfo.replyId)
            {
            	platform_write ("Found an entry in PA response queue with swinfo0 = 0x%08x, expected 0x%08x\n",
                                pHostDesc->softwareInfo0, cmdReplyInfo.replyId);
                pHostDesc->buffLen  =   pHostDesc->origBufferLen;
                QMSS_QPUSH (gTxCmdFreeQHnd, pHostDesc, pHostDesc->buffLen, SIZE_HOST_DESC, Qmss_Location_TAIL);
                ret_val = -1;
                goto return_fail;

            }

            retVal  =   Pa_forwardResult (res_mgr_get_painstance(), (Ptr)pHostDesc->buffPtr, &retHandle, &handleType, &cmdDest);
            if (retVal != pa_OK)
            {
            	platform_write ("PA sub-system rejected Pa_addMac command\n");
                ret_val = -1;
                goto return_fail;
            }

            /* Reset the buffer length and put the descriptor back on the Rx free queue */
            pHostDesc->buffLen = pHostDesc->origBufferLen;
            QMSS_QPUSH (gRxFreeQHnd, pHostDesc, pHostDesc->buffLen, SIZE_HOST_DESC, Qmss_Location_TAIL);
            break;
        }
    }

    if (j == 100)
    {
    	platform_write ("Timeout waiting for reply from PA to Pa_addMac command\n");
     	Osal_nimuFree (pCmdDataBuffer, cmdbuf_len);
        ret_val = -1;
    }

return_fail:

	Osal_nimuFree (pCmdDataBuffer, cmdbuf_len);

    return (ret_val);
}



/**
 *  @b Description
 *  @n
 *      Returns the number of received packet drops, most likely due to NDK buffer starvation.
 *
 *  @retval
 *      None.
 */
uint32_t nimu_getrxdrops(void) {
	return  gRxDropCounter;
}

/**
 *  @b Description
 *  @n
 *      Returns the number of transmit packet drops due to Tx descriptor starvation.
 *
 *  @retval
 *      None.
 */
uint32_t nimu_gettxdrops(void) {
	return  gTxDropCounter;
}
