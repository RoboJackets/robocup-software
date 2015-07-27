/*
 * @brief	lwIP Filesystem implementation module header
 *
 * @note
 * Copyright(C) NXP Semiconductors, 2012
 * All rights reserved.
 *
 * @par
 * Software that is described herein is for illustrative purposes only
 * which provides customers with programming information regarding the
 * LPC products.  This software is supplied "AS IS" without any warranties of
 * any kind, and NXP Semiconductors and its licensor disclaim any and
 * all warranties, express or implied, including all implied warranties of
 * merchantability, fitness for a particular purpose and non-infringement of
 * intellectual property rights.  NXP Semiconductors assumes no responsibility
 * or liability for the use of the software, conveys no license or rights under any
 * patent, copyright, mask work right, or any other intellectual property rights in
 * or to any products. NXP Semiconductors reserves the right to make changes
 * in the software without notification. NXP Semiconductors also makes no
 * representation or warranty that such application will be suitable for the
 * specified use without further testing or modification.
 *
 * @par
 * Permission to use, copy, modify, and distribute this software and its
 * documentation is hereby granted, under NXP Semiconductors' and its
 * licensor's relevant copyrights in the software, without fee, provided that it
 * is used in conjunction with NXP Semiconductors microcontrollers.  This
 * copyright, permission, and disclaimer notice must appear in all copies of
 * this code.
 */
#ifndef __LWIP_FS_H_
#define __LWIP_FS_H_

#include "lwip/opt.h"

/** @defgroup EXAMPLE_DUALCORE_LWIP_FS Filesystem Glue logic for reading http files
 * @ingroup EXAMPLE_DUALCORE_LWIP
 * The lwIP Filesystem implementation module implements the file system functions
 * required for FAT file system. The webserver uses this function to fetch the
 * files from the appropriate device.
 * @{
 */

/** Set this to 1 and provide the functions:
 * - "int fs_open_custom(struct fs_file *file, const char *name)"
 *    Called first for every opened file to allow opening files
 *    that are not included in fsdata(_custom).c
 * - "void fs_close_custom(struct fs_file *file)"
 *    Called to free resources allocated by fs_open_custom().
 */
#ifndef LWIP_HTTPD_CUSTOM_FILES
#define LWIP_HTTPD_CUSTOM_FILES       0
#endif

/** Set this to 1 to include an application state argument per file
 * that is opened. This allows to keep a state per connection/file.
 */
#ifndef LWIP_HTTPD_FILE_STATE
#define LWIP_HTTPD_FILE_STATE         0
#endif

/** HTTPD_PRECALCULATED_CHECKSUM==1: include precompiled checksums for
 * predefined (MSS-sized) chunks of the files to prevent having to calculate
 * the checksums at runtime. */
#ifndef HTTPD_PRECALCULATED_CHECKSUM
#define HTTPD_PRECALCULATED_CHECKSUM  0
#endif

#if HTTPD_PRECALCULATED_CHECKSUM
struct fsdata_chksum {
  u32_t offset;
  u16_t chksum;
  u16_t len;
};
#endif /* HTTPD_PRECALCULATED_CHECKSUM */

struct fs_file {
  const char *data;
  int len;
  int index;
  void *pextension;
#if HTTPD_PRECALCULATED_CHECKSUM
  const struct fsdata_chksum *chksum;
  u16_t chksum_count;
#endif /* HTTPD_PRECALCULATED_CHECKSUM */
  u8_t http_header_included;
#if LWIP_HTTPD_CUSTOM_FILES
  u8_t is_custom_file;
#endif /* LWIP_HTTPD_CUSTOM_FILES */
#if LWIP_HTTPD_FILE_STATE
  void *state;
#endif /* LWIP_HTTPD_FILE_STATE */
};

/**
 * @brief	Get HTTP header function
 * @param fName	:   Filename for which the header be generated
 * @param buff	:   buffer to which the generated header be copied
 * @return Number of bytes in header
 * The function will read the HTTP header from the file & returns with
 * No. of bytes read
 */
int GetHTTP_Header(const char *fName, char *buff);

/**
 * @brief	Open default HTTP file function
 * @return Pointer to File structure of default file on success
 *         NULL on failure
 * The function will open the default file. The default file will have the
 * HTTP content pointed by http_index_html variable.
 * This function will be called if the HTTP files are not present in
 * the file system.
 */
struct fs_file *fs_open_default(void);

/**
 * @brief	Open a file from the Filesystem function
 * @param name	:	Name of the file to be opened
 * @return Pointer to File structure on success
 *         NULL on failure
 * The function will open the file present on the file system.
 */
struct fs_file *fs_open(const char *name);

/**
 * @brief	Closes/Frees a previously opened file function
 * @param file	:	Pointer to File structure of opened file
 * @return None
 * The function will close the file & free the resources.
 */
void fs_close(struct fs_file *file);

/**
 * @brief	This reads the requested number of bytes from the file function
 * @param file	:	Pointer to File structure of opened file
 * @param	buffer :	pointer to memroy, where the data be stored
 * @param	count	: Number of bytes to be read from the file
 * @return Number of bytes successfully read
 * The function will read the specified number of bytes from the file.
 */
int fs_read(struct fs_file *file, char *buffer, int count);

/**
 * @brief	Get number of bytes yet to be read in a file function
 * @param file	:	Pointer to File structure of opened file
 * @return Number of byets yet to be read from the file
 * The function will return the number bytes yet to be read from the file.
 */
int fs_bytes_left(struct fs_file *file);

#if LWIP_HTTPD_FILE_STATE
/** This user-defined function is called when a file is opened. */
void *fs_state_init(struct fs_file *file, const char *name);
/** This user-defined function is called when a file is closed. */
void fs_state_free(struct fs_file *file, void *state);
#endif /* #if LWIP_HTTPD_FILE_STATE */

#ifdef LWIP_DEBUG
/**
 * @brief	Assert function for debug
 * @param msg	:	Message to be printed
 * @param	line	:	Line number from which the assert was raised
 * @param	file	:	Filename from which the assert was raised
 * @return None
 * The function will assert.
 */
void assert_printf(char *msg, int line, char *file);

/**
 * @brief	This is a dummy function, required for debug
 * @param eno	:	Error Number
 * @return Empty string
 */
const char *lwip_strerr(err_t eno);
#endif

/**
 * @}
 */

#endif /* __LWIP_FS_H_ */
