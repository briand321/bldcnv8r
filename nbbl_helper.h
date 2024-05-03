
#ifndef  NBBL_HELPER_H_
#define  NBBL_HELPER_H_
#include <stdint.h>

uint32_t init_bootloader_if(void);
uint32_t nbbl_helper_invalidate_app_signature(void);
void nbbl_helper_invalidate_app_signature_and_reboot(void);
uint32_t nbbl_helper_canspeed_to_br( uint16_t b );
uint16_t nbbl_helper_default_node_id(void);
void nbbl_helper_save_shared_state(void);

#endif  /* NBBL_HELPER_H_ */
