
#include <stdlib.h>
#include "shared.h"
#include "nbbl_helper.h"
#include "ch.h"
#include "hal.h"
#include "stm32f4xx_conf.h"
#include "utils_sys.h"
#include "mc_interface.h"

/**
 * This is the Brickproof Bootloader's app descriptor.
 * Details: https://github.com/PX4/Firmware/tree/nuttx_next/src/drivers/bootloaders/src/uavcan
 */
static const volatile app_descriptor_t _app_descriptor __attribute__((section(".app_descriptor"))) =
{
#if defined(HW60_IS_NV8R)
		.signature = {'A','P','D','e','s','c','0','0'},
#else
		.signature = {'N','o','t','U','s','e','d','0'},
#endif
		.image_crc = 0,
		.image_size = 0,
		.vcs_commit = 0,
		.major_version = 1,
		.minor_version = 1,
		.reserved = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}
};

uint32_t nbbl_helper_invalidate_app_signature(void)
{
     uint32_t base = (uint32_t) &_app_descriptor;;
     uint32_t len = (uint32_t) sizeof(_app_descriptor);
     uint16_t res = FLASH_COMPLETE;

     FLASH_Unlock();
     FLASH_ClearFlag(FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR |
             FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);

     utils_sys_lock_cnt();

     /* zero out the application descriptor to invalidate it for NBBL */
     for (uint32_t i = 0;i < len;i++) {
         res = FLASH_ProgramByte(base + i, 0);
         if (res != FLASH_COMPLETE) {
             break;
         }
     }

     FLASH_Lock();

     utils_sys_unlock_cnt();

     return FLASH_COMPLETE;
}
void nbbl_helper_invalidate_app_signature_and_reboot(void)
{
     LED_RED_ON();
     // invalidate the application descriptor
     nbbl_helper_invalidate_app_signature();

     conf_general_store_backup_data();

	 // Clear pending interrupts
	 SCB->ICSR = SCB_ICSR_PENDSVCLR_Msk;

	 // Disable all interrupts
	 for(int i = 0;i < 8;i++) {
		NVIC->ICER[i] = NVIC->IABR[i];
	 }

     NVIC_SystemReset(); // reboot MCU
     LED_RED_OFF();
     while(1);  // should never get here
}

uint32_t nbbl_helper_canspeed_to_br( uint16_t b )
{
    uint32_t cs2br[]={
/* CAN_BAUD types from datatypes.h */
/*      CAN_BAUD_125K = 0,  */  125000UL,
/*      CAN_BAUD_250K,      */  250000UL,
/*      CAN_BAUD_500K,      */  500000UL,
/*      CAN_BAUD_1M,        */ 1000000UL
    };
    if( b > CAN_BAUD_1M ) b = CAN_BAUD_1M; 
    return cs2br[b];
}
