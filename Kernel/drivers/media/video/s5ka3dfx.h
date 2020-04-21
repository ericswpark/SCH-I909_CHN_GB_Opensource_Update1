#define S5KA3DFX_COMPLETE
//#undef S5KA3DFX_COMPLETE
/*
 * Driver for S5KA3DFX (VGA camera) from Samsung Electronics
 * 
 * 1/4" 2.0Mp CMOS Image Sensor SoC with an Embedded Image Processor
 *
 * Copyright (C) 2009, Jinsung Yang <jsgood.yang@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */
#ifndef __S5KA3DFX_H__
#define __S5KA3DFX_H__

struct s5ka3dfx_reg {
        unsigned char addr;
        unsigned char val;
};

struct s5ka3dfx_regset_type {
        unsigned char *regset;
        int len;
};

/*
 * Macro
 */
#define REGSET_LENGTH(x)        (sizeof(x)/sizeof(s5ka3dfx_reg))

/*
 * User defined commands
 */
/* S/W defined features for tune */
#define REG_DELAY       0xFF00  /* in ms */
#define REG_CMD         0xFFFF  /* Followed by command */

/* Following order should not be changed */
enum image_size_s5ka3dfx {
        /* This SoC supports upto UXGA (1600*1200) */
#if 0
        QQVGA,  /* 160*120*/
        QCIF,   /* 176*144 */
        QVGA,   /* 320*240 */
        CIF,    /* 352*288 */
        VGA,    /* 640*480 */
#endif
        SVGA,   /* 800*600 */
#if 0
        HD720P, /* 1280*720 */
        SXGA,   /* 1280*1024 */
        UXGA,   /* 1600*1200 */
#endif
};

/*
 * Following values describe controls of camera
 * in user aspect and must be match with index of s5ka3dfx_regset[]
 * These values indicates each controls and should be used
 * to control each control
 */
enum s5ka3dfx_control {
        S5KA3DFX_INIT,
        S5KA3DFX_EV,
        S5KA3DFX_AWB,
        S5KA3DFX_MWB,
        S5KA3DFX_EFFECT,
        S5KA3DFX_CONTRAST,
        S5KA3DFX_SATURATION,
        S5KA3DFX_SHARPNESS,
};

#define S5KA3DFX_REGSET(x)      {       \
        .regset = x,                    \
        .len = sizeof(x)/sizeof(s5ka3dfx_reg),}

#endif
