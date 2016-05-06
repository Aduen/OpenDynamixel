/*
 * dxl2.h
 *
 *  Created on: 2013. 11. 8.
 *      Author: in2storm
 */

#ifndef DXL2_H_
#define DXL2_H_
#ifdef __cplusplus
extern "C" {
#endif
    
#include "dxl_constants.h"
#include "stdint.h"
    
    /** DYNAMIXEL device type  */
#define DXL_RX_BUF_SIZE 0x3FF
#define DXL_PARAMETER_BUF_SIZE 128
    
    
    uint32_t dxl_get_baudrate(int baudnum);
    unsigned short update_crc(unsigned short crc_accum, unsigned char *data_blk_ptr, unsigned short data_blk_size); // for Dxl 2.0
    
#ifdef __cplusplus
}
#endif
#endif /* DXL2_H_ */
