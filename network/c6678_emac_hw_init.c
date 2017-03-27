/* Resource manager for QMSS, PA, CPPI */
#include "ti/platform/resource_mgr.h"

/* Initialize the components required to run this application:
 *  (1) QMSS
 *  (2) CPPI
 *  (3) Packet Accelerator
 */
void c6678_emac_hw_init(void)
{

	QMSS_CFG_T qmss_cfg;
	CPPI_CFG_T cppi_cfg;

	if (platform_get_switch_state(1))
	{
		printf("Sorry, DHCP is set! Please set to static IP mode!!!\n");
		exit(0);
	}

	/* Initialize QMSS */
	if (platform_get_coreid() == 0) {
		qmss_cfg.master_core = 1;
	} else {
		qmss_cfg.master_core = 0;
	}
	qmss_cfg.max_num_desc = MAX_NUM_DESC;
	qmss_cfg.desc_size = MAX_DESC_SIZE;
	qmss_cfg.mem_region = Qmss_MemRegion_MEMORY_REGION0;
	if (res_mgr_init_qmss(&qmss_cfg) != 0) {
		printf("Failed to initialize the QMSS subsystem.\n");
	} else {
		printf("QMSS successfully initialized.\n");
	}

	/* Initialize CPPI */
	if (platform_get_coreid() == 0) {
		cppi_cfg.master_core = 1;
	} else {
		cppi_cfg.master_core = 0;
	}
	cppi_cfg.dma_num = Cppi_CpDma_PASS_CPDMA;
	cppi_cfg.num_tx_queues = NUM_PA_TX_QUEUES;
	cppi_cfg.num_rx_channels = NUM_PA_RX_CHANNELS;
	if (res_mgr_init_cppi(&cppi_cfg) != 0) {
		printf("Failed to initialize CPPI subsystem.\n");
	} else {
		printf("CPPI successfully initialized.\n");
	}

	if (res_mgr_init_pass() != 0) {
		printf("Failed to initialize the Packet Accelerator.\n");
	} else {
		printf("PA successfully initialized.\n");
	}
}
