#ifndef DMPKEY_H__
#define DMPKEY_H__

#define KEY_FCFG_1 (0)
#define KEY_FCFG_2 (KEY_FCFG_1 + 1)
#define KEY_FCFG_3 (KEY_FCFG_2 + 1)
#define KEY_FCFG_5 (KEY_FCFG_3 + 1)
#define KEY_FCFG_6 (KEY_FCFG_5 + 1)
#define KEY_FCFG_7 (KEY_FCFG_6 + 1)

#define KEY_CFG_1 (KEY_FCFG_7 + 1)
#define KEY_CFG_2 (KEY_CFG_1 + 1)
#define KEY_CFG_3 (KEY_CFG_2 + 1)
#define KEY_CFG_3B (KEY_CFG_3 + 1)
#define KEY_CFG_3C (KEY_CFG_3B + 1)
#define KEY_CFG_3D (KEY_CFG_3C + 1)
#define KEY_CFG_4 (KEY_CFG_3D + 1)
#define KEY_CFG_6 (KEY_CFG_4 + 1)
#define KEY_CFG_8 (KEY_CFG_6 + 1)
#define KEY_CFG_9 (KEY_CFG_8 + 1)
#define KEY_CFG_10 (KEY_CFG_9 + 1)
#define KEY_CFG_12 (KEY_CFG_10 + 1)
#define KEY_CFG_13 (KEY_CFG_12 + 1)
#define KEY_CFG_15 (KEY_CFG_13 + 1)
#define KEY_CFG_16 (KEY_CFG_15 + 1)
#define KEY_CFG_17 (KEY_CFG_16 + 1)
#define KEY_CFG_18 (KEY_CFG_17 + 1)
#define KEY_CFG_19 (KEY_CFG_18 + 1)

#define KEY_D_0_22 (KEY_CFG_19 + 1)
#define KEY_D_0_24 (KEY_D_0_22 + 1)
#define KEY_D_0_36 (KEY_D_0_24 + 1)
#define KEY_D_0_52 (KEY_D_0_36 + 1)
#define KEY_D_0_224 (KEY_D_0_52 + 1)
#define KEY_D_0_228 (KEY_D_0_224 + 1)
#define KEY_D_0_232 (KEY_D_0_228 + 1)
#define KEY_D_0_236 (KEY_D_0_232 + 1)
#define KEY_D_0_96 (KEY_D_0_236 + 1)
#define KEY_D_0_104 (KEY_D_0_96 + 1)
#define KEY_D_0_108 (KEY_D_0_104 + 1)
#define KEY_D_0_163 (KEY_D_0_108 + 1)

#define KEY_DMP_PREVPTAT (KEY_D_0_163 + 1)
#define KEY_D_1_10 (KEY_DMP_PREVPTAT + 1)
#define KEY_D_1_98 (KEY_D_1_10 + 1)
#define KEY_D_1_106 (KEY_D_1_98 + 1)
#define KEY_D_1_179 (KEY_D_1_106 + 1)
#define KEY_D_1_236 (KEY_D_1_179 + 1)
#define KEY_D_1_244 (KEY_D_1_236 + 1)

#define NUM_KEYS (KEY_D_1_244 + 1)

typedef struct {
    unsigned short key;
    unsigned short addr;
} tKeyLabel;

#define DINA22 0x22
#define DINA42 0x42

#define DINA06 0x06
#define DINA0E 0x0e
#define DINA26 0x26
#define DINA36 0x36
#define DINA46 0x46
#define DINA56 0x56
#define DINA66 0x66
#define DINA76 0x76
#define DINA2E 0x2e
#define DINA4E 0x4e
#define DINA6E 0x6e

#define DINA00 0x00
#define DINA08 0x08
#define DINA18 0x18
#define DINA20 0x20
#define DINA28 0x28
#define DINA30 0x30
#define DINA38 0x38
#define DINA50 0x50
#define DINA78 0x78

#define DINA0C 0x0c
#define DINA2C 0x2c
#define DINA4C 0x4c
#define DINA6C 0x6c

#define DINA2D 0x2d
#define DINA35 0x35
#define DINA3D 0x3d
#define DINA55 0x55
#define DINA7D 0x7d

#define DINA80 0x80
#define DINA90 0x90
#define DINAA0 0xa0
#define DINAC9 0xc9
#define DINACB 0xcb
#define DINACD 0xcd
#define DINAD8 0xd8
#define DINADD 0xdd
#define DINAF8 0xf8
#define DINAFE 0xfe

unsigned short getAddress(unsigned short key);

#endif  // DMPKEY_H__
