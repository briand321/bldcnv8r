/****************************************************************************
 *
 *   Copyright (c) 2015 PX4 Development Team. All rights reserved.
 *       Author: Ben Dyer <ben_dyer@mac.com>
 *               Pavel Kirienko <pavel.kirienko@zubax.com>
 *               David Sidrane <david_s5@nscdg.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>
#include <string.h>

#include <errno.h>
#include "shared.h"
#include "crcbldr.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define BOOTLOADER_COMMON_APP_SIGNATURE         0xB0A04150u
#define BOOTLOADER_COMMON_BOOTLOADER_SIGNATURE  0xB0A0424Cu

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/
/*
  * Shared memory region used to communicate boot parameters between the bootloader and application
  */
static const volatile bootloader_app_shared_t
_shared_boot_info_memory[2] __attribute__((section(".sharedsram")));


/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sm_read
 ****************************************************************************/

inline static void sm_read(uint16_t idx,bootloader_app_shared_t *pshared)
{
    /* Read out of shared reserved SRAM */
    bootloader_app_shared_t *p_sm 
        = (bootloader_app_shared_t*) &_shared_boot_info_memory[idx];
    memcpy( pshared, p_sm, sizeof( bootloader_app_shared_t ) );
}


/****************************************************************************
 * Name: sm_write
 ****************************************************************************/

inline static void sm_write(uint16_t idx, bootloader_app_shared_t *pshared)
{
    /* Write to shared reserved SRAM */
    bootloader_app_shared_t *p_sm 
        = (bootloader_app_shared_t*) &_shared_boot_info_memory[idx];
    memcpy( p_sm, pshared, sizeof( bootloader_app_shared_t ) );
}

/****************************************************************************
 * Name: calulate_signature
 ****************************************************************************/

static uint64_t calulate_signature(bootloader_app_shared_t *pshared)
{
	uint64_t crc;
	crc = crc64_add_word(CRC64_INITIAL, pshared->signature);
	crc = crc64_add_word(crc, pshared->bus_speed);
	crc = crc64_add_word(crc, pshared->node_id);
	crc ^= CRC64_OUTPUT_XOR;
	return crc;
}

/****************************************************************************
 * Name: bootloader_app_shared_init
 ****************************************************************************/
static void bootloader_app_shared_init(bootloader_app_shared_t *pshared, eRole_t role)
{
	memset(pshared, 0, sizeof(bootloader_app_shared_t));

	if (role != Invalid) {
		pshared->signature =
			(role ==
			 App ? BOOTLOADER_COMMON_APP_SIGNATURE :
			 BOOTLOADER_COMMON_BOOTLOADER_SIGNATURE);
	}

}

/****************************************************************************
 * Public Functions
 ****************************************************************************/
/****************************************************************************
 * Name: bootloader_app_shared_read
 *
 * Description:
 *   Based on the role requested, this function will conditionally populate
 *   a bootloader_app_shared_t structure from the physical locations used
 *   to transfer the shared data to/from an application (internal data) .
 *
 *   The functions will only populate the structure and return a status
 *   indicating success, if the internal data has the correct signature as
 *   requested by the Role AND has a valid crc.
 *
 * Input Parameters:
 *   shared - A pointer to a bootloader_app_shared_t return the data in if
 *   the internal data is valid for the requested Role
 *   role   - An eRole_t of App or BootLoader to validate the internal data
 *            against. For a Bootloader this would be the value of App to
 *            read the application passed data.
 *
 * Returned value:
 *   0      - Indicates that the internal data has been copied to callers
 *            bootloader_app_shared_t structure.
 *
 *  -1      - The Role or crc of the internal data was not valid. The copy
 *            did not occur.
 *
 ****************************************************************************/

int bootloader_app_shared_read(bootloader_app_shared_t *shared,
			       eRole_t role)
{
	int rv = -1;
	bootloader_app_shared_t working;

    if( role == App2 )
    {
	    sm_read(1,&working);
    }
    else
    {
	    sm_read(0,&working);
    }

	if ((((role == App) || (role == App2)) 
         ? working.signature == BOOTLOADER_COMMON_APP_SIGNATURE
	     : working.signature == BOOTLOADER_COMMON_BOOTLOADER_SIGNATURE)
	    && (working.crc == calulate_signature(&working))) {
		*shared = working;
		rv = 0;
	}

	return rv;
}

/****************************************************************************
 * Name: bootloader_app_shared_write
 *
 * Description:
 *   Based on the role, this function will commit the data passed
 *   into the physical locations used to transfer the shared data to/from
 *   an application (internal data) .
 *
 *   The functions will populate the signature and crc the data
 *   based on the provided Role.
 *
 * Input Parameters:
 *   shared - A pointer to a bootloader_app_shared_t data to commit to
 *   the internal data for passing to/from an application.
 *   role   - An eRole_t of App or BootLoader to use in the internal data
 *            to be passed to/from an application. For a Bootloader this
 *            would be the value of Bootloader to write to the passed data.
 *            to the application via the internal data.
 *
 * Returned value:
 *   None.
 *
 ****************************************************************************/
void bootloader_app_shared_write(bootloader_app_shared_t *shared,
				 eRole_t role)
{
	bootloader_app_shared_t working = *shared;
	working.signature =
		((role ==
		 App) || (role == App2)) ? BOOTLOADER_COMMON_APP_SIGNATURE :
		 BOOTLOADER_COMMON_BOOTLOADER_SIGNATURE;
	working.crc = calulate_signature(&working);
    if( role == App2 )
    {
	    sm_write(1,&working);
    }
    else
    {
	    sm_write(0,&working);
    }
}

/****************************************************************************
 * Name: bootloader_app_shared_invalidate
 *
 * Description:
 *   Invalidates the data passed the physical locations used to transfer
 *   the shared data to/from an application (internal data) .
 *
 *   The functions will invalidate the signature and crc and shoulf be used
 *   to prevent deja vu.
 *
 * Input Parameters:
 *   None.
 *
 * Returned value:
 *   None.
 *
 ****************************************************************************/

void bootloader_app_shared_invalidate(void)
{
	bootloader_app_shared_t working;
	bootloader_app_shared_init(&working, Invalid);
	sm_write(0,&working);
	sm_write(1,&working);
}
