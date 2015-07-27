/*----------------------------------------------------------------------------
*      Name:    fatutil.c
*      Purpose: FAT utilities
*      Version: V1.00
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

#include "fatutil.h"
#include <string.h>

/** @ingroup EXAMPLE_MassStorage
    @{ */

#if 0
DISKIMAGE Fat12_1_FAT = {
	{
		{0xEB, 0x3C, 0x90},							/* Jump instruction to the boot code */
		"MSWIN4.1",							/* Name of system that formatted the volume */
		BYTESPERSECTOR,					/* Bytes per sector (should be 512) */
		SECTORSPERCLUSTER,								/* Sectors per cluster (FAT-12 = 1) */
		RESERVEDSECTORCNT,								/* Reserved sectors (FAT-12 = 1) */
		NUMFATS,						/* FAT tables on the disk (should be 2) */
		ROOTENTRIES,								/* Max directory entries in root directory */
		VIRTUAL_MEMORY_BLOCKS,					/* FAT-12 total number of sectors on the disk */
		FAT_MEDIA,								/* Media type {fixed, removable, etc.} */
		SECTORSPERFAT,					/* Sector size of FAT table (FAT-12 = 9) */
		63,								/* # of sectors per cylindrical track */
		0xFF,								/* # of heads per volume (1.4Mb 3.5" = 2) */
		1,								/* # of preceding hidden sectors (0) */
		0,								/* # of FAT-32 sectors (0 for FAT-12) */
		0x80,								/* A drive number for the media (OS specific) */
		0,								/* Reserved space for Windows NT (set to 0) */
		41,								/* (0x29) Indicates following: */
		654449012,						/* Volume serial # (for tracking this disk) */
		{"Jennic USB "},				/* Volume label (RDL or "NO NAME    ") */
		{"FAT12   "},					/* Deceptive FAT type Label */
	},

	{0},						/* Boot code */
	0xaa55,						/* BootSectorSignature */

	//		{
	//				{0xF8, 0x0F, {0xFF}},					/* 1st FAT */
	// #if NUMFATS > 1
	//				{0xF8, 0x0F, {0xFF}},					/* 2nd FAT */
	// #endif
	// #if NUMFATS > 2
	//				{{0}},					/* 3rd FAT */
	// #endif
	//		},
	//		{
	//				{
	//						{"LPC134x "},	/* File name (capital letters, padded w/spaces) */
	//						{"USB"},		/* Extension (same format as name, no '.') */
	//						0x28,			/* Holds the attributes code */
	//						{0},			/* Reserved for Windows NT (Set to zero!) */
	//						{0,0,0},		/* Time of last write */
	//						{0,0,0},		/* Date of last write */
	//						0,				/* Pointer to the first cluster of the file */
	//						0,				/* File size in bytes */
	//				},
	//				{
	//						{"FIRMWARE"},	/* File name (capital letters, padded w/spaces) */
	//						{"BIN"},		/* Extension (same format as name, no '.') */
	//						0x20,			/* Holds the attributes code */
	//						{0},			/* Reserved for Windows NT (Set to zero!) */
	//						{0,0,0},		/* Time of last write */
	//						{0,0,0},		/* Date of last write */
	//						0,				/* Pointer to the first cluster of the file */
	//						0,				/* File size in bytes */
	//				},
	//		},
};
#endif
// Return an unsigned short containing the 12-bit FAT entry code
// at FATindex unsigned short
int16_t GetFAT12Entry(DISKIMAGE *DiskImagePtr, int FATindex)
{
	unsigned short FATEntryCode;		// The return value
	int FatOffset = ((FATindex * 3) / 2);	// Calculate the offset
	if (FATindex % 2 == 1) {// If the index is odd
		// Pull out a unsigned short from a unsigned char array
		FATEntryCode = *((unsigned short *) &DiskImagePtr->Fat[0].FATByte[FatOffset]);
		FATEntryCode >>= 4;	// Extract the high-order 12 bits
	}
	else {				// If the index is even
		// Pull out a unsigned short from a unsigned char array
		FATEntryCode = *((unsigned short *) &DiskImagePtr->Fat[0].FATByte[FatOffset]);
		FATEntryCode &= 0x0fff;	// Extract the low-order 12 bits
	}
	return FATEntryCode;
}	// End GetFatEntry

void SetFAT12Entry(DISKIMAGE *DiskImagePtr, int FATindex, unsigned short FAT12ClusEntryVal)
{
	int FATOffset = ((FATindex * 3) / 2);	// Calculate the offset
	int FATData = *((unsigned char *) &DiskImagePtr->Fat[0].FATByte[FATOffset]);
	if (FATindex % 2 == 0) {// If the index is even
		FAT12ClusEntryVal &= 0x0FFF;	// mask to 12 bits
		FATData &= 0xF000;	// mask complement
	}
	else {			// Index is odd
		FAT12ClusEntryVal <<= 4;	// move 12-bits high
		FATData &= 0x000F;	// mask complement
	}

	// Update FAT entry value in the FAT table
	*((unsigned char *) &DiskImagePtr->Fat[0].FATByte[FATOffset]) = FATData | FAT12ClusEntryVal;
}	// End SetFatEntry

void InitializeFAT12(DISKIMAGE *DiskImagePtr)
{
	int i;

	SetFAT12Entry(DiskImagePtr, 1, 0xfff);
	for (i = 0;
		 i <
		 ((DiskImagePtr->BootSector.BPB_TotSec16 - StartDataRegion /
		   BytesPerSector) / DiskImagePtr->BootSector.BPB_SecPerClus);
		 i++) {
		if (!i) {
			DiskImagePtr->DirectoryEntries[1].startCluster = 2;	// Startcluster is always 2 as defined by FAT12
		}
		else {
			SetFAT12Entry(DiskImagePtr, i + 1, i + 2);	// 2 represents the  startcluster (which is fixed by definition)
			SetFAT12Entry(DiskImagePtr, i + 2, 0xfff);
		}
		DiskImagePtr->DirectoryEntries[1].fileSize += BYTESPERCLUSTER;
	}
	//	InitializeDiskDiskImage(DiskImagePtr);
}

void InitializeFAT16(DISKIMAGE *DiskImagePtr)
{}

void InitializeFAT32(DISKIMAGE *DiskImagePtr)
{}

uint32_t CalculateCountOfClusters(DISKIMAGE *DiskImagePtr)
{
	int RootDirSectors, DataSec, CountofClusters;

	RootDirSectors =
		((DiskImagePtr->BootSector.BPB_RootEntCnt *
		  32) + (DiskImagePtr->BootSector.BPB_BytsPerSec - 1)) / DiskImagePtr->BootSector.BPB_BytsPerSec;

	DataSec = DiskImagePtr->BootSector.BPB_TotSec16 -
			  (DiskImagePtr->BootSector.BPB_RsvdSecCnt +
			   (DiskImagePtr->BootSector.BPB_NumFATs *
				DiskImagePtr->BootSector.BPB_FATSz16) +
			   RootDirSectors);
	CountofClusters =  DataSec / DiskImagePtr->BootSector.BPB_SecPerClus;

	return CountofClusters;
}

int32_t ReservedSectorCnt = RESERVEDSECTORCNT;
int32_t RootEntries       = ROOTENTRIES;
int32_t BytesPerSector    = BYTESPERSECTOR;
int32_t NumFats           = NUMFATS;
int32_t SectorsPerFat     = SECTORSPERFAT;
int32_t NonDataSectors    = NONDATASECTORS;
int32_t TotalSectors      = TOTALSECTORS;
int32_t StartDataRegion   = STARTDATAREGION;

void InitializeDiskDiskImage(DISKIMAGE *DiskImagePtr)
{
	DiskImagePtr->BootSector.BPB_RsvdSecCnt = ReservedSectorCnt;
	DiskImagePtr->BootSector.BPB_TotSec16   = TotalSectors;
	DiskImagePtr->BootSector.BPB_FATSz16    = SectorsPerFat;
	DiskImagePtr->BootSector.BPB_NumFATs    = NumFats;
	DiskImagePtr->BootSector.BPB_RootEntCnt = RootEntries;
	DiskImagePtr->BootSector.BPB_BytsPerSec = BytesPerSector;
	DiskImagePtr->BootSector.BPB_Media      = 0xf8;
	DiskImagePtr->BootSector.BPB_SecPerClus = SECTORSPERCLUSTER;
}

void SetDiskMetricsFromDiskImage(DISKIMAGE *DiskImagePtr)
{
	ReservedSectorCnt = DiskImagePtr->BootSector.BPB_RsvdSecCnt;
	TotalSectors      = DiskImagePtr->BootSector.BPB_TotSec16;
	SectorsPerFat     = DiskImagePtr->BootSector.BPB_FATSz16;
	NumFats           = DiskImagePtr->BootSector.BPB_NumFATs;
	RootEntries       = DiskImagePtr->BootSector.BPB_RootEntCnt;
	BytesPerSector    = DiskImagePtr->BootSector.BPB_BytsPerSec;
	StartDataRegion   = (ReservedSectorCnt + NumFats * SectorsPerFat + ((RootEntries - 1) / 16 + 1)) * BytesPerSector;
}

void CreateDiskImage(DISKIMAGE *DiskImagePtr)
{
	int i;

	// Initialize all directory entries
	for (i = 0; i < ROOTENTRIES; i++) {
		memset((void *) &DiskImagePtr->DirectoryEntries[i], 0, sizeof(DiskImagePtr->DirectoryEntries[0]));
	}

	// Set the volume label
	strncpy((char *) DiskImagePtr->DirectoryEntries[0].Name, "USBlib  ", 8);
	strncpy((char *) DiskImagePtr->DirectoryEntries[0].Extension, "   ", 3);
	DiskImagePtr->DirectoryEntries[0].Attributes = 0x28;
	DiskImagePtr->DirectoryEntries[0].Time.hour  = TIME_HOUR;
	DiskImagePtr->DirectoryEntries[0].Time.min   = TIME_MIN;
	DiskImagePtr->DirectoryEntries[0].Time.sec   = TIME_SEC;
	DiskImagePtr->DirectoryEntries[0].Date.year  = DATE_YEAR;
	DiskImagePtr->DirectoryEntries[0].Date.month = DATE_MONTH;
	DiskImagePtr->DirectoryEntries[0].Date.day   = DATE_DAY;

	// Initialize the binary data file
	strncpy((char *) DiskImagePtr->DirectoryEntries[1].Name, "FIRMWARE", 8);
	strncpy((char *) DiskImagePtr->DirectoryEntries[1].Extension, "BIN", 3);
	DiskImagePtr->DirectoryEntries[1].Attributes = 0x20;
	// DiskImagePtr->DirectoryEntries[1].Reserved[0] = 0x18;
	DiskImagePtr->DirectoryEntries[1].Time.hour  = TIME_HOUR;
	DiskImagePtr->DirectoryEntries[1].Time.min   = TIME_MIN;
	DiskImagePtr->DirectoryEntries[1].Time.sec   = TIME_SEC;
	DiskImagePtr->DirectoryEntries[1].Date.year  = DATE_YEAR;
	DiskImagePtr->DirectoryEntries[1].Date.month = DATE_MONTH;
	DiskImagePtr->DirectoryEntries[1].Date.day   = DATE_DAY;

	DiskImagePtr->DirectoryEntries[1].fileSize = 0;
#if (FATTYPE == FAT12)
	InitializeFAT12(DiskImagePtr);
#endif
#if (FATTYPE == FAT16)
	InitializeFAT16(DiskImagePtr);
#endif
#if (FATTYPE == FAT32)
	InitializeFAT32(DiskImagePtr);
#endif

}

void InitializeDiskImage(DISKIMAGE *DiskImagePtr,
						 int VolumeSize,
						 int BytesPerSector,
						 int NumFATs,
						 int SectorsPerFAT,
						 int RootEntries)
{
	DiskImagePtr->BootSector.BPB_FATSz16    = SectorsPerFAT;
	DiskImagePtr->BootSector.BPB_NumFATs    = NumFATs;
	DiskImagePtr->BootSector.BPB_RootEntCnt = RootEntries;
	DiskImagePtr->BootSector.BPB_BytsPerSec = BytesPerSector;
	DiskImagePtr->BootSector.BPB_SecPerClus = SECTORSPERCLUSTER;
	DiskImagePtr->BootSector.BPB_Media      = 0xf8;
	DiskImagePtr->BootSector.BPB_RsvdSecCnt = RESERVEDSECTORCNT;

	DiskImagePtr->BootSector.BPB_TotSec16   = VolumeSize / BytesPerSector;

}

/** @} */
