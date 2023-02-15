/*
 *  TMS320C6678 Ethernet driver
 *
 *  Copyright (c) 2016-2023 by BUPT University
 *  Written by Maxul Lee <lmy2010lmy@gamil.com>
 *
 *  The license and distribution terms for this file may be
 *  found in the file LICENSE in this distribution or at
 *  http://www.rtems.org/license/LICENSE.
 */

#include <ti/csl/csl_bootcfgAux.h>
#include <ti/csl/csl_cacheAux.h>
#include <ti/csl/csl_chip.h>
#include <ti/csl/csl_cpIntcAux.h>
#include <ti/csl/src/intc/csl_intc.h>

#include "Keystone_common.h"
#include <stdio.h>

unsigned int Normally_Enabled_Interrupts = 0xFFF0;

extern void TSC_init();
extern void Timer_Init(void);

static system_init()
{
    // for this test, we use DSP core number as DSP number
    // hence the program should be run on core 0 of DSP0 and core 1 of DSP1
    Uint32 uiDspNum = KeyStone_Get_DSP_Number();

    // enable TSC, memory protection interrupts, EDC for internal RAM
    // clear cache; protect L1 as cache
    KeyStone_common_CPU_init();

    // print device information
    // Enable memory protection interrupts, EDC for MSMC RAM
    KeyStone_common_device_init();

    // enable exception handling
    KeyStone_Exception_cfg(TRUE);

    CACHE_setL1PSize(CACHE_L1_32KCACHE);
    CACHE_setL1DSize(CACHE_L1_32KCACHE);
    CACHE_setL2Size(CACHE_256KCACHE);

    // make SL2 cacheable
    for (i = 12; i < 16; i++)
        gpCGEM_regs->MAR[i] = 1;

    // make other cores local memory cacheable and prefetchable
    for (i = 16; i < 24; i++)
        gpCGEM_regs->MAR[i] = 1 | (1 << CSL_CGEM_MAR0_PFX_SHIFT);

    // make DDR cacheable and prefetchable
    for (i = 128; i < 256; i++)
        gpCGEM_regs->MAR[i] = 1 | (1 << CSL_CGEM_MAR0_PFX_SHIFT);

    // make other space non-cacheable and non-prefetchable
    for (i = 24; i < 128; i++)
        gpCGEM_regs->MAR[i] = 0;

    CSL_BootCfgUnlockKicker();
}

static void GE_Interrupts_Init()
{
    // map message descriptor accumulation high priority channel interrupt to INT7
    gpCGEM_regs->INTMUX1 = (CSL_GEM_QM_INT_HIGH_N << CSL_CGEM_INTMUX1_INTSEL7_SHIFT);

    // enable INT7 for GE NIC
    CPU_interrupt_enable((1 << 7));
}

static void nic_init()
{
    c6678_emac_hw_init();
    c6678_emac_init();
    c6678_emac_start();

    GE_Interrupts_Init();
}

void main()
{
    int i;

    system_init();
    nic_init();

    // After INT being enabled, TSCL will start to count
    TSC_init();

    // receiving packets here
    for (;;) {
        c6678_eth_recvpacket();
    }
}
