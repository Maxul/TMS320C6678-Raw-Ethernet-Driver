-heap  0x800000
-stack 0x1000

MEMORY
{
	/* Local L2, 0.5~1MB*/
	VECTORS: 	o = 0x00800000  l = 0x00000200   
	LL2_RW_DATA: 	o = 0x00800200  l = 0x0003FE00   

	/* Shared L2 2~4MB*/
	SL2: 		o = 0x0C000000  l = 0x00200000   
	
	/* External DDR3, upto 2GB per core */
	DDR3_CODE: 	o = 0x80000000  l = 0x01000000   /*set memory protection attribitue as execution only*/
	DDR3_R_DATA: 	o = 0x81000000  l = 0x01000000 	 /*set memory protection attribitue as read only*/
	DDR3_RW_DATA: 	o = 0x82000000  l = 0x06000000   /*set memory protection attribitue as read/write*/
}

SECTIONS
{
	vecs       	>    VECTORS 

	.text           >    SL2
	.cinit          >    SL2
	.const          >    SL2
	.switch         >    SL2

	.stack          >    DDR3_RW_DATA
	GROUP
	{
		.neardata
		.rodata
		.bss
	} 		>    LL2_RW_DATA
	.far            >    DDR3_RW_DATA
	.fardata        >    DDR3_RW_DATA
	.cio            >    DDR3_RW_DATA
	.sysmem         >    DDR3_R_DATA
	/*QMSS_Data:linkingRAM1 		> 	SL2
	QMSS_Data:Descriptor_SL2 	> 	SL2
	PacketData:buffer_SL2 		> 	SL2
	QMSS_Data:Descriptor_LL2 	> 	LL2_RW_DATA
	PacketData:buffer_LL2 		> 	LL2_RW_DATA
	QMSS_Data:Descriptor_DDR 	> 	DDR3_RW_DATA
	PacketData:buffer_DDR 		> 	DDR3_RW_DATA*/
	
	.cppi: load >> DDR3_RW_DATA
    .qmss: load >> DDR3_RW_DATA
	platform_lib: load >> DDR3_RW_DATA
	.nimu_eth_ll2: load >> SL2
    .resmgr_memregion: load >> SL2 align = 0x4
    .resmgr_handles: load >> SL2 align = 0x4
    .resmgr_pa: load >> SL2 align = 0x4

}


