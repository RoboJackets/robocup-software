/*----------------------------------------------------------------------------
* Name:    fatutil.h
* Purpose: FAT utilities
* Version: V1.00
**----------------------------------------------------------------------------
*      This software is supplied "AS IS" without any warranties, express,
*      implied or statutory, including but not limited to the implied
*      warranties of fitness for purpose, satisfactory quality and
*      noninfringement. NXP extends you a royalty-free right to reproduce
*      and distribute executable files created using this software for use
*      on NXP Semiconductors LPC microcontroller devices only. Nothing else
*      gives you the right to use this software.
*
* Copyright (c) 2011 NXP Semiconductors. All rights reserved.
*---------------------------------------------------------------------------*/

#ifndef __FATUTIL_H__
#define __FATUTIL_H__

#include "usb.h"
#include "DataRam.h"
#define FAT12 1
#define FAT16 2
#define FAT32 3

#define UDIV_ROUNDUP(a, b) ((a + b - 1) / b)
#define FATTYPE FAT12

// #define DISKSIZE	(VIRTUAL_MEMORY_BYTES)
#define DISKSIZE    (0x100000)
#define BYTESPERSECTOR (512)
#define RESERVEDSECTORCNT (1)
#define ROOTENTRIES (UDIV_ROUNDUP(BYTESPERSECTOR, 32))
// #define DIRECTORYSECTORCNT ((ROOTENTRIES - 1) / 16 + 1)
#define DIRECTORYSECTORCNT (UDIV_ROUNDUP(ROOTENTRIES, 16))
#define NUMFATS (2)
#define SECTORSPERCLUSTER (1)
#define DATASECTORS (1024)
// #define TOTALSECTORS  (NONDATASECTORS + 330)
#define TOTALSECTORS  (UDIV_ROUNDUP(DISKSIZE, BYTESPERSECTOR))
#define FATENTRIESPERSECTOR (UDIV_ROUNDUP(BYTESPERSECTOR * 8, 12 /*FAT12*/))
#define SECTORSPERFAT (UDIV_ROUNDUP(UDIV_ROUNDUP((TOTALSECTORS - RESERVEDSECTORCNT), SECTORSPERCLUSTER), \
									FATENTRIESPERSECTOR))

#define NONDATASECTORS (RESERVEDSECTORCNT + NUMFATS * SECTORSPERFAT + DIRECTORYSECTORCNT)
#define STARTDATAREGION (NONDATASECTORS * BYTESPERSECTOR)
#define BYTESPERCLUSTER (BYTESPERSECTOR * SECTORSPERCLUSTER)
#define FAT_MEDIA   (0xF0)

// Time of date of FIRMWARE.BIN on the USB Stick
#define TIME_HOUR   16
#define TIME_MIN    00
#define TIME_SEC    00
#define DATE_YEAR   2011 - 1980
#define DATE_MONTH  8
#define DATE_DAY    11

extern int32_t RootEntries;
extern int32_t BytesPerSector;
extern int32_t NumFats;
extern int32_t SectorsPerFat;
extern int32_t NonDataSectors;
extern int32_t TotalSectors;
extern int32_t StartDataRegion;

typedef struct {
	uint8_t     BS_jmpBoot[3];	/* Jump instruction to the boot code */
	uint8_t     BS_OEMName[8];	/* Name of system that formatted the volume */
	uint16_t    BPB_BytsPerSec;	/* Bytes per sector (should be 512) */
	uint8_t     BPB_SecPerClus;	/* Sectors per cluster (FAT-12 = 1) */
	uint16_t    BPB_RsvdSecCnt;	/* Reserved sectors (FAT-12 = 1) */
	uint8_t     BPB_NumFATs;	/* FAT tables on the disk (should be 2) */
	uint16_t    BPB_RootEntCnt;	/* Max directory entries in root directory */
	uint16_t    BPB_TotSec16;	/* Total number of sectors on the disk (FAT-12 and FAT-16) */
	uint8_t     BPB_Media;		/* Media type {fixed, removable, etc.} */
	uint16_t    BPB_FATSz16;	/* Sector size of FAT table (FAT-12 = 9) */
	uint16_t    BPB_SecPerTrk;	/* # of sectors per cylindrical track */
	uint16_t    BPB_NumHeads;	/* # of heads per volume (1.4Mb 3.5" = 2) */
	uint32_t    BPB_HiddSec;	/* # of preceding hidden sectors (0) */
	uint32_t    BPB_TotSec32;	/* # of FAT-32 sectors (0 for FAT-12) */
	uint8_t     BS_DrvNum;		/* A drive number for the media (OS specific) */
	uint8_t     BS_Reserved1;	/* Reserved space for Windows NT (set to 0) */
	uint8_t     BS_BootSig;		/* (0x29) Indicates following: */
	uint32_t    BS_VolID;		/* Volume serial # (for tracking this disk) */
	uint8_t     BS_VolLab[11];	/* Volume label (RDL or "NO NAME    ") */
	uint8_t     BS_FilSysType[8];	/* Deceptive FAT type Label */
} ATTR_PACKED BSStruct;

typedef struct {
	uint8_t FATByte[BYTESPERSECTOR * SECTORSPERFAT];
} ATTR_PACKED FAT;

typedef struct {				// (total 16 bits--a unsigned short)
	uint16_t sec : 5;	// low-order 5 bits are the seconds
	uint16_t min : 6;	// next 6 bits are the minutes
	uint16_t hour : 5;	// high-order 5 bits are the hour
} ATTR_PACKED FATTime;

typedef struct {				// (total 16 bits--a unsigned short)
	uint16_t day : 5;	// low-order 5 bits are the day
	uint16_t month : 4;	// next 4 bits are the month
	uint16_t year : 7;	// high-order 7 bits are the year
} ATTR_PACKED FATDate;

typedef struct {
	uint8_t     Name[8];		/* File name (capital letters, padded w/spaces) */
	uint8_t     Extension[3];	/* Extension (same format as name, no '.') */
	uint8_t     Attributes;		/* Holds the attributes code */
	uint8_t     Reserved[10];	/* Reserved for Windows NT (Set to zero!) */
	FATTime     Time;			/* Time of last write */
	FATDate     Date;			/* Date of last write */
	uint16_t    startCluster;	/* Pointer to the first cluster of the file */
	uint32_t    fileSize;		/* File size in bytes */
} ATTR_PACKED DirEntry;

typedef struct _diskimage {
	BSStruct BootSector;
	uint8_t BootCode[448];
	uint16_t BootSectorSignature;
	FAT Fat[NUMFATS];
	DirEntry DirectoryEntries[ROOTENTRIES];
} DISKIMAGE;

int16_t GetFAT12Entry(DISKIMAGE *DiskImagePtr, int FATindex);

void SetFAT12Entry(DISKIMAGE *DiskImagePtr, int FATindex, unsigned short FAT12ClusEntryVal);

uint32_t CalculateCountOfClusters(DISKIMAGE *DiskImagePtr);

void CreateDiskImage(DISKIMAGE *DiskImagePtr);

void SetDiskMetricsFromDiskImage(DISKIMAGE *DiskImagePtr);

void InitializeDiskDiskImage(DISKIMAGE *DiskImagePtr);

void InitializeFAT12(DISKIMAGE *DiskImagePtr);

#endif  /* __FATUTIL_H__ */
