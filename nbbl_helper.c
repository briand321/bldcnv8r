
#include <stdlib.h>
#include "shared.h"
#include "nbbl_helper.h"
#include "ch.h"
#include "hal.h"
#include "stm32f4xx_conf.h"
#include "utils_sys.h"
#include "mc_interface.h"
#include "utils_math.h"
#include "app.h"

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

void nbbl_helper_save_shared_state(void)
{
    const app_configuration *conf = app_get_configuration();
    bootloader_app_shared_t shared_info;

    if( conf != NULL )
    {
        bootloader_app_shared_invalidate();

        shared_info.bus_speed =
            nbbl_helper_canspeed_to_br( conf->can_baud_rate );
        shared_info.node_id = conf->controller_id;

        bootloader_app_shared_write(&shared_info, App);
        bootloader_app_shared_write(&shared_info, App2);
    }
}

uint16_t nbbl_helper_default_node_id(void)
{
    bootloader_app_shared_t common;

    // Priority and logic for default node ID:
    //
    // If last running FW stored a good node ID to assume, use it.
    // Else-If bootloader stored a dynamically assigned node ID, use it.
    // Else derive a node ID from the STM chip's UUID in range 16 to 80.

    // Did last running FW app leave us a node ID to use?
    if( (0 == bootloader_app_shared_read(&common,App2)) &&
		common.bus_speed && common.node_id )
    {
        return common.node_id;
    }
    // Did bootloader pass us a good dynamic node ID to use?
    else if( (0 == bootloader_app_shared_read(&common, BootLoader)) &&
		common.bus_speed && common.node_id )
    {
        return common.node_id;
    }
    else
    {   // Compute by taking CRC over chip's UUID and range limiting.
        uint16_t id = utils_crc32c(STM32_UUID_8, 12) & 0x3F;
        id = id + 16;   // range 16 - 79
        return id;
    }
}
