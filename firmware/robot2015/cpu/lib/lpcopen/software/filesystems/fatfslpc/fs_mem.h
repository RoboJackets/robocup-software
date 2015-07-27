
#ifndef __FS_MEM_H_
#define __FS_MEM_H_

#include "board.h"

#define SECTOR_SZ            512 /* Size of a single sector */

#ifdef __cplusplus
extern "C" {
#endif

/** @defgroup LPCOPEN_FSLIBS_CHANFATFS_FSMEM Memory based file system support
 * @ingroup LPCOPEN_FSLIBS_CHANFATFS
 * @{
 */

/**
 * @brief	Gets Ramdisk Memory information
 * @param	buffer	: pointer to hold the starting address of RAM disk
 * @param	size	: Pointer to returned size of the RAM disk
 * @return	None
 * @note	This function must be implemented by the application so that the
 * filesystem will be aware of the location of the RAM disk.
 */
void FATFS_GetBufferInfo(uint8_t **buffer, uint32_t *size);

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* ifndef __FS_MEM_H_ */
