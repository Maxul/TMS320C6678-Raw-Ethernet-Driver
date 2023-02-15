#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ethernet.h"

typedef unsigned int uint;

void dbgprintf(char* p, int len)
{
    int i;

    for (i = 0; i < len; ++i)
        printf("%02x ", 0xff & (*(p + i)));
    printf("\n");
}

void printHex(char* p, int len, int sep)
{
    char* cp = p;
    int i = len;

    do {
        i--;
        printf("%02X%c", *cp++, i ? sep : ' ');
    } while (i > 0);
}

//---------------------------------------------
// Fast Crit Sections
//---------------------------------------------
#include <ti/csl/csl_semAux.h>
#define OEMSysCrit_HW_SEM 22
uint OEMSysCritOn()
{
    /* Get the hardware semaphore.
     *
     * Acquire Multi core QMSS synchronization lock
     */
    while ((CSL_semAcquireDirect(OEMSysCrit_HW_SEM)) == 0)
        ;
    return 1;
}

void OEMSysCritOff(uint key)
{
    /* Release the hardware semaphore
     *
     * Release multi-core lock.
     */
    CSL_semReleaseSemaphore(OEMSysCrit_HW_SEM);
}

#include "Keystone_common.h"
unsigned int Enabled_Interrupts = 0xfff0;
void _ISR_Disable(int core)
{
    Enabled_Interrupts = IER;
    IER = 0;
    /* Disable Global host interrupts. */
    gpCIC0_regs->GLOBAL_ENABLE_HINT_REG = 0;
}
void _ISR_Enable(int core)
{
    IER |= Enabled_Interrupts;
    /* Enable Global host interrupts. */
    gpCIC0_regs->GLOBAL_ENABLE_HINT_REG = 1;
}
int _ISR_Get_level()
{
    return 0;
}
