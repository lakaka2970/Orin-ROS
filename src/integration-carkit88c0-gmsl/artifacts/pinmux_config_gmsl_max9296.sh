#!/bin/bash

# Exit on any error
set -e

# Create silent wrapper for devmem2 to suppress all output
silent_devmem2() {
    command sudo devmem2 "$@" > /dev/null 2>&1
}

# ------------------------------------------------------------------------------
# This script configures the pinmux settings for the CARKIT2C0 GMSL setup
# All the connected pins are configured for their respective functions
# ------------------------------------------------------------------------------

# ------------------------------------------------------------------------------
# MFPs and pins connected to the Deserializer 
# ------------------------------------------------------------------------------

# spi2_sck_pcc0, E61, pin #117
# Usage: MFP0/SCLK
silent_devmem2 0x0c302048 word 0x1408

# can0_stb_paa4, E59, pin #97
# Usage: MFP1/GPIO1
silent_devmem2 0x0c303020 word 0xC004

# spi5_mosi_pac2, G7, pin #96
# Usage: MFP2/BNE
silent_devmem2 0x02448050 word 0x57

# soc_gpio50_pbb2, A62, pin #85
# Usage: MFP3/GPO
silent_devmem2 0x0c303050 word 0xC054

# soc_gpio57_pac4, A7, pin #84
# Usage: MFP4/ODO11/GPIO9
silent_devmem2 0x02448020 word 0x1054

# spi2_mosi_pcc2, F60, pin #103
# Usage: MFP5/MOSI
silent_devmem2 0x0c302028 word 0x408

# spi2_miso_pcc1, D62, pin #106
# Usage: MFP6/MISO
silent_devmem2 0x0c302050 word 0x458

# spi2_cs0_pcc3, D60, pin #104
# Usage: MFP7/RO
silent_devmem2 0x0c302038 word 0x9

# extperiph2_clk_pp1, H53, pin #88
# Usage: MFP8/GPI
silent_devmem2 0x02430000 word 0x5

# spi5_miso_pac1, F9, pin #92
# Usage: MFP9/GPI
silent_devmem2 0x02448048 word 0x3

# spi5_cs0_pac3, L15, pin #76
# Usage: MFP10/GPI
silent_devmem2 0x02448040 word 0x7

# dp_aux_ch3_p_pn7, A53, pin #105
# Usage: MFP11/SCL
silent_devmem2 0x02440070 word 0x1460

# cam_i2c_scl_pp2, F53, pin #75
# Usage: MFP11/SCL
silent_devmem2 0x02430018 word 0x1460

# dp_aux_ch3_n_pn0, C53, pin #107
# Usage: MFP12/SDA
silent_devmem2 0x02440078 word 0x460

# cam_i2c_sda_pp3, E53, pin #77
# Usage: MFP12/SDA
silent_devmem2 0x02430010 word 0x460

# uart4_tx_ph3, L5, pin #95
# Usage: RST
silent_devmem2 0x02434020 word 0x2

# ------------------------------------------------------------------------------
# GPIOs connected to Testpoints
# Set the register values as needed. No meaningful configuration yet.
# ------------------------------------------------------------------------------

# uart4_rts_ph5, L4, pin #86
# Usage: TP L4
silent_devmem2 0x02434010 word 0x2

# gen2_i2c_scl_pcc7, J61, pin #87
# Usage: TP J61
silent_devmem2 0x0c302030 word 0x1460

# gen2_i2c_sda_pdd0, K61, pin #89
# Usage: TP K61
silent_devmem2 0x0c302040 word 0x460

# spi5_sck_pac0, F10, pin #90
# Usage: TP F10
silent_devmem2 0x02448058 word 0x1003

# extperiph1_clk_pp0, J54, pin #91
# Usage: TP J54
silent_devmem2 0x02430008 word 0x400

# uart4_cts_ph6, L49, pin #93
# Usage: TP L49
silent_devmem2 0x02434008 word 0x1

# soc_gpio32_pq5, H55, pin #94
# Usage: TP H55
silent_devmem2 0x02430068 word 0x405

# can0_err_paa7, F59, pin #98
# Usage: TP F59
silent_devmem2 0x0c303038 word 0xC004

# dp_aux_ch3_hpd_pm3, J51, pin #101
# Usage: TP J51
silent_devmem2 0x02440048 word 0x440

# soc_gpio37_pr0, K57, pin #109
# Usage: TP K57
silent_devmem2 0x02430080 word 0x400

# spi1_sck_pz3, J57, pin #111
# Usage: TP J57
silent_devmem2 0x0243d028 word 0x1055

# spi1_mosi_pz5, D55, pin #112
# Usage: TP D55
silent_devmem2 0x0243d040 word 0x55

# spi1_cs0_pz6, E55, pin #113
# Usage: TP E55
silent_devmem2 0x0243d008 word 0x55

# spi1_miso_pz4, A56, pin #114
# Usage: TP A56
silent_devmem2 0x0243d018 word 0x55











