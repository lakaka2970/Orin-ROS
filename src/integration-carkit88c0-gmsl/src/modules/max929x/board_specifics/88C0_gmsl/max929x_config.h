/*
 * Copyright (c) 2020-2026, Infineon Technologies AG.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef MAX929X_CONFIG_H
#define MAX929X_CONFIG_H

#include "max929x_types.h"

static struct max929x_reg max9296_SINGLE_LINK_Dser_Ser_init[] = {
    /*
    # This script is validated on: 
    # MAX96717
    # MAX9296B
    # Please refer to the Errata sheet for each device.
    # ---------------------------------------------------------------------------------
    */
    //
    // CSIConfigurationTool
    //
    // GMSL-A / Serializer: MAX96717 (Pixel Mode) / Mode: 1x4 / Device Address: 0x84 / Multiple-VC Case: Single VC / Pipe Sharing: Separate Pipes
    // PipeZ:
    // Input Stream: VC0 RAW12 PortB (D-PHY) (Doubled)

    // Deserializer: MAX9296B / Mode: 2 (1x4) / Device Address: 0x90
    // PipeX:
    // GMSL-A Input Stream: VC0 RAW12 PortB - Output Stream: VC0 RAW12 PortA (D-PHY)

    {0x90, 0x0313, 0x00},  //  (CSI_OUT_EN): CSI output disabled
    // Link Initialization for Deserializer
    {0x90, 0x0010, 0x31},           //  (AUTO_LINK): Enabled | (Default)  (LINK_CFG): 0x1 |  (RESET_ONESHOT): Activated
    {MAX929X_DELAY, 0x0000, 0x78},  // 120 msec delay
    // Video Transmit Configuration for Serializer(s)
    {0x84, 0x0002, 0x03},  // DEV : REG2 | VID_TX_EN_Z (VID_TX_EN_Z): Disabled

    {0x90, 0x0003, 0x00},
    {0x90, 0x0170, 0x08},
    {0x90, 0x0172, 0x07},
    {0x90, 0x0176, 0x03},
    {0x84, 0x0173, 0x1E},
    {0x84, 0x0174, 0x1E},
    {0x84, 0x0175, 0x1E},
    {0x84, 0x0176, 0x0C},
    {0x84, 0x0172, 0x0C},
    {0x84, 0x0170, 0x0A},
    {MAX929X_DELAY, 0x0000, 0x78},  // 120 msec delay

    //
    // INSTRUCTIONS FOR GMSL-A SERIALIZER MAX96717
    //
    // MIPI D-PHY Configuration
    {0x84, 0x0330, 0x00},  // MIPI_RX : MIPI_RX0 | (Default) RSVD (Port Configuration): 1x4
    {0x84, 0x0383, 0x00},  // MIPI_RX_EXT : EXT11 | Tun_Mode (Tunnel Mode): Disabled
    {0x84, 0x0331, 0x30},  // MIPI_RX : MIPI_RX1 | (Default) ctrl1_num_lanes (Port B - Lane Count): 4
    {0x84, 0x0332, 0xE0},  // MIPI_RX : MIPI_RX2 | (Default) phy1_lane_map (Lane Map - PHY1 D0): Lane 2 | (Default) phy1_lane_map (Lane Map - PHY1 D1): Lane 3
    {0x84, 0x0333, 0x04},  // MIPI_RX : MIPI_RX3 | (Default) phy2_lane_map (Lane Map - PHY2 D0): Lane 0 | (Default) phy2_lane_map (Lane Map - PHY2 D1): Lane 1
    {0x84, 0x0334, 0x00},  // MIPI_RX : MIPI_RX4 | (Default) phy1_pol_map (Polarity - PHY1 Lane 0): Normal | (Default) phy1_pol_map (Polarity - PHY1 Lane 1): Normal
    {0x84, 0x0335, 0x00},  // MIPI_RX : MIPI_RX5 | (Default) phy2_pol_map (Polarity - PHY2 Lane 0): Normal | (Default) phy2_pol_map (Polarity - PHY2 Lane 1): Normal | (Default) phy2_pol_map (Polarity - PHY2 Clock Lane): Normal
    // Controller to Pipe Mapping Configuration
    {0x84, 0x0308, 0x64},  // FRONTTOP : FRONTTOP_0 | (Default) RSVD (CLK_SELZ): Port B | (Default) START_PORTB (START_PORTB): Enabled
    {0x84, 0x0311, 0x40},  // FRONTTOP : FRONTTOP_9 | (Default) START_PORTBZ (START_PORTBZ): Start Video
    {0x84, 0x0318, 0x6C},  // FRONTTOP : FRONTTOP_16 | mem_dt1_selz (mem_dt1_selz): 0x6C
    // Double Mode Configuration
    {0x84, 0x0313, 0x40},  // FRONTTOP : FRONTTOP_11 | bpp12dblz (bpp12dblz): Send 12-bit pixels as 24-bit
    {0x84, 0x031E, 0x38},  // FRONTTOP : FRONTTOP_22 | (Default) soft_bppz (soft_bppz): 0x18 | soft_bppz_en (soft_bppz_en): Software override enabled
    // Pipe Configuration
    {0x84, 0x005B, 0x00},  // CFGV__VIDEO_Z : TX3 | TX_STR_SEL (TX_STR_SEL Pipe Z): 0x0
    //
    // INSTRUCTIONS FOR DESERIALIZER MAX9296B
    //
    // Video Pipes And Routing Configuration
    {0x90, 0x0050, 0x00},  // (Default)  (STR_SELX): 0x0
    // Pipe to Controller Mapping Configuration
    {0x90, 0x040B, 0x07},  //  (MAP_EN_L Pipe X): 0x7
    {0x90, 0x040C, 0x00},  // (Default)  (MAP_EN_H Pipe X): 0x0
    {0x90, 0x040D, 0x2C},  //  (MAP_SRC_0 Pipe X DT): 0x2C | (Default)  (MAP_SRC_0 Pipe X VC): 0x0
    {0x90, 0x040E, 0x2C},  //  (MAP_DST_0 Pipe X DT): 0x2C | (Default)  (MAP_DST_0 Pipe X VC): 0x0
    {0x90, 0x040F, 0x00},  // (Default)  (MAP_SRC_1 Pipe X DT): 0x0 | (Default)  (MAP_SRC_1 Pipe X VC): 0x0
    {0x90, 0x0410, 0x00},  // (Default)  (MAP_DST_1 Pipe X DT): 0x0 | (Default)  (MAP_DST_1 Pipe X VC): 0x0
    {0x90, 0x0411, 0x01},  //  (MAP_SRC_2 Pipe X DT): 0x1 | (Default)  (MAP_SRC_2 Pipe X VC): 0x0
    {0x90, 0x0412, 0x01},  //  (MAP_DST_2 Pipe X DT): 0x1 | (Default)  (MAP_DST_2 Pipe X VC): 0x0
    {0x90, 0x042D, 0x15},  //  (MAP_DPHY_DST_0 Pipe X): 0x1 |  (MAP_DPHY_DST_1 Pipe X): 0x1 |  (MAP_DPHY_DST_2 Pipe X): 0x1
    // Double Mode Configuration
    {0x90, 0x0473, 0x01},  //  (ALT_MEM_MAP12 CTRL1): Alternate memory map enabled
    // MIPI D-PHY Configuration
    {0x90, 0x0330, 0x04},  // (Default)  (Port Configuration): 2 (1x4)
    {0x90, 0x044A, 0xD0},  // (Default)  (Port A - Lane Count): 4
    {0x90, 0x0333, 0x4E},  // (Default)  (Lane Map - PHY0 D0): Lane 2 | (Default)  (Lane Map - PHY0 D1): Lane 3 | (Default)  (Lane Map - PHY1 D0): Lane 0 | (Default)  (Lane Map - PHY1 D1): Lane 1
    {0x90, 0x0335, 0x00},  // (Default)  (Polarity - PHY0 Lane 0): Normal | (Default)  (Polarity - PHY0 Lane 1): Normal | (Default)  (Polarity - PHY1 Lane 0): Normal | (Default)  (Polarity - PHY1 Lane 1): Normal | (Default)  (Polarity - PHY1 Clock Lane): Normal
    {0x90, 0x1D00, 0xF4},  //  (config_soft_rst_n - PHY1): 0x0
    // This is to set predefined (coarse) CSI output frequency
    // CSI Phy 1 is 1200 Mbps/lane.
    {0x90, 0x0320, 0x2C},
    {0x90, 0x1D00, 0xF5},  //  (config_soft_rst_n - PHY1): 0x1
    {0x90, 0x0332, 0x30},  //  (phy_Stdby_2): Put PHY2 in standby mode |  (phy_Stdby_3): Put PHY3 in standby mode
    {0x90, 0x0313, 0x02},  //  (CSI_OUT_EN): CSI output enabled
    // Video Transmit Configuration for Serializer(s)
    {0x84, 0x0002, 0x43},  // DEV : REG2 | VID_TX_EN_Z (VID_TX_EN_Z): Enabled


    {0x90, 0x0005, 0x00},
    {0x84, 0x0005, 0x00},
    // A DMUX2 DES1<-SER4
    // rx & tx ID = 0x1
    {0x90, 0x02B3, 0x04},
    {0x90, 0x02B4, 0xA1},
    {0x90, 0x02B5, 0x41},
    {0x84, 0x02CA, 0x9B},
    {0x84, 0x02CB, 0xA1},
    {0x84, 0x02CC, 0x41},
    // A RFT DES3<-SER6
    // rx & tx ID = 0x2
    {0x90, 0x02B9, 0x04},
    {0x90, 0x02BA, 0xA2},
    {0x90, 0x02BB, 0x42},
    {0x84, 0x02D0, 0x9B},
    {0x84, 0x02D1, 0xA2},
    {0x84, 0x02D2, 0x42},
    // A OK DES4<-SER5
    // rx & tx ID = 0x3
    {0x90, 0x02BC, 0x04},
    {0x90, 0x02BD, 0xA3},
    {0x90, 0x02BE, 0x43},
    {0x84, 0x02CD, 0x9B},
    {0x84, 0x02CE, 0xA3},
    {0x84, 0x02CF, 0x43},
    // A RESETN DES8->SER2
    // rx & tx ID = 0x4
    {0x90, 0x02C8, 0x9B},
    {0x90, 0x02C9, 0xA4},
    {0x90, 0x02CA, 0x44},
    {0x84, 0x02C4, 0x84},
    {0x84, 0x02C5, 0xA4},
    {0x84, 0x02C6, 0x44},
    // A DMUX1 DES9->SER3
    // rx & tx ID = 0x5
    {0x90, 0x02CB, 0x9B},
    {0x90, 0x02CC, 0xA5},
    {0x90, 0x02CD, 0x45},
    {0x84, 0x02C7, 0x84},
    {0x84, 0x02C8, 0xA5},
    {0x84, 0x02C9, 0x45},

    {0x90, 0x0170, 0x09},
    {0x84, 0x0170, 0x0B},

    // Enable GMSL negative output
    {0x84, 0x14ce, 0x19},

    // Reset MIPI RX and enable continuous clock mode
    // {0x84, 0x0330, 0x08},

    // Reset MIPI RX and enable non-continuous clock mode
    {0x84,0x0330,0x48},

    {MAX929X_DELAY, 0x0000, 0x78},  // 120 msec delay

    // Unset mipi_rx_reset and enable continuous clock mode
    // {0x84, 0x0330, 0x00},

    // Unset mipi_rx_reset and enable non-continuous clock mode
    {0x84,0x0330,0x40},
};

static struct max929x_reg max9296_SPLITTER_MODE_Dser_Ser_init[] = {
    /*
    # This script is validated on:
    # MAX96717
    # MAX9296A
    # Please refer to the Errata sheet for each device.
    # ---------------------------------------------------------------------------------
    */
    //
    // CSIConfigurationTool
    //
    // GMSL-A / Serializer: MAX96717 (Pixel Mode) / Mode: 1x4 / Device Address: 0x84 / Multiple-VC Case: Single VC / Pipe Sharing: Separate Pipes
    // PipeZ:
    // Input Stream: VC0 RAW12 PortB (D-PHY) (Doubled)

    // GMSL-B / Serializer: MAX96717 (Pixel Mode) / Mode: 1x4 / Device Address: 0x84 / Multiple-VC Case: Single VC / Pipe Sharing: Separate Pipes
    // PipeZ:
    // Input Stream: VC0 RAW12 PortB (D-PHY) (Doubled)

    // Deserializer: MAX9296A / Mode: 2 (1x4) / Device Address: 0x90
    // PipeX:
    // GMSL-A Input Stream: VC0 RAW12 PortB - Output Stream: VC0 RAW12 PortA (D-PHY)
    // PipeY:
    // GMSL-B Input Stream: VC0 RAW12 PortB - Output Stream: VC0 RAW12 PortB (D-PHY)

    {0x90, 0x0313, 0x00},  // (CSI_OUT_EN): CSI output disabled
    // Single Link Initialization Before Serializer Device Address Change
    {0x90, 0x0010, 0x22},           // (AUTO_LINK): Disabled |  (LINK_CFG): 0x2 |  (RESET_ONESHOT): Activated
    {MAX929X_DELAY, 0x0000, 0x78},  // 120 msec delay
    // GMSL-B Serializer Address Change from 0x84 to 0xC4
    {0x84, 0x0000, 0xC4},           // DEV : REG0 | DEV_ADDR
    {MAX929X_DELAY, 0x0000, 0x78},  // 120 msec delay

    // Set unique TX_SRC_ID
    {0xC4, 0x007B, 0x31},
    {0xC4, 0x0083, 0x31},
    {0xC4, 0x008B, 0x31},
    {0xC4, 0x0093, 0x31},
    {0xC4, 0x00A3, 0x31},
    {0xC4, 0x00AB, 0x31},
    {MAX929X_DELAY, 0x0000, 0x78},  // 120 msec delay

    // Link Initialization for Deserializer
    {0x90, 0x0010, 0x23},           // (Default)  (AUTO_LINK): Disabled |  (LINK_CFG): 0x3 | (Default)  (RESET_ONESHOT): Activated
    {MAX929X_DELAY, 0x0000, 0x78},  // 120 msec delay
    // Video Transmit Configuration for Serializer(s)
    {0x84, 0x0002, 0x03},  // DEV : REG2 | VID_TX_EN_Z (VID_TX_EN_Z): Disabled
    {0xC4, 0x0002, 0x03},  // DEV : REG2 | VID_TX_EN_Z (VID_TX_EN_Z): Disabled

    // SPI settings
    {0x90, 0x0003, 0x00},
    {0x90, 0x0170, 0x08},
    {0x90, 0x0172, 0x07},
    {0x90, 0x0176, 0x03},

    {0x84, 0x0173, 0x1E},
    {0x84, 0x0174, 0x1E},
    {0x84, 0x0175, 0x1E},
    {0x84, 0x0176, 0x04},  // enable SS1
    {0x84, 0x0172, 0x0C},
    {0x84, 0x0170, 0xA},

    {0xC4, 0x0173, 0x1E},
    {0xC4, 0x0174, 0x1E},
    {0xC4, 0x0175, 0x1E},
    {0xC4, 0x0176, 0x04},  // enable SS1
    {0xC4, 0x0172, 0x0C},
    {0xC4, 0x0170, 0xA},

    //
    // INSTRUCTIONS FOR GMSL-A SERIALIZER MAX96717
    //
    // MIPI D-PHY Configuration
    {0x84, 0x0330, 0x00},  // MIPI_RX : MIPI_RX0 | (Default) RSVD (Port Configuration): 1x4
    {0x84, 0x0383, 0x00},  // MIPI_RX_EXT : EXT11 | Tun_Mode (Tunnel Mode): Disabled
    {0x84, 0x0331, 0x30},  // MIPI_RX : MIPI_RX1 | (Default) ctrl1_num_lanes (Port B - Lane Count): 4
    {0x84, 0x0332, 0xE0},  // MIPI_RX : MIPI_RX2 | (Default) phy1_lane_map (Lane Map - PHY1 D0): Lane 2 | (Default) phy1_lane_map (Lane Map - PHY1 D1): Lane 3
    {0x84, 0x0333, 0x04},  // MIPI_RX : MIPI_RX3 | (Default) phy2_lane_map (Lane Map - PHY2 D0): Lane 0 | (Default) phy2_lane_map (Lane Map - PHY2 D1): Lane 1
    {0x84, 0x0334, 0x00},  // MIPI_RX : MIPI_RX4 | (Default) phy1_pol_map (Polarity - PHY1 Lane 0): Normal | (Default) phy1_pol_map (Polarity - PHY1 Lane 1): Normal
    {0x84, 0x0335, 0x00},  // MIPI_RX : MIPI_RX5 | (Default) phy2_pol_map (Polarity - PHY2 Lane 0): Normal | (Default) phy2_pol_map (Polarity - PHY2 Lane 1): Normal | (Default) phy2_pol_map (Polarity - PHY2 Clock Lane): Normal
    // Controller to Pipe Mapping Configuration
    {0x84, 0x0308, 0x64},  // FRONTTOP : FRONTTOP_0 | (Default) RSVD (CLK_SELZ): Port B | (Default) START_PORTB (START_PORTB): Enabled
    {0x84, 0x0311, 0x40},  // FRONTTOP : FRONTTOP_9 | (Default) START_PORTBZ (START_PORTBZ): Start Video
    {0x84, 0x0318, 0x6C},  // FRONTTOP : FRONTTOP_16 | mem_dt1_selz (mem_dt1_selz): 0x6C
    // Double Mode Configuration
    {0x84, 0x0313, 0x40},  // FRONTTOP : FRONTTOP_11 | bpp12dblz (bpp12dblz): Send 12-bit pixels as 24-bit
    {0x84, 0x031E, 0x38},  // FRONTTOP : FRONTTOP_22 | (Default) soft_bppz (soft_bppz): 0x18 | soft_bppz_en (soft_bppz_en): Software override enabled
    // Pipe Configuration
    {0x84, 0x005B, 0x00},  // CFGV__VIDEO_Z : TX3 | TX_STR_SEL (TX_STR_SEL Pipe Z): 0x0
    //
    // INSTRUCTIONS FOR GMSL-B SERIALIZER MAX96717
    //
    // MIPI D-PHY Configuration
    {0xC4, 0x0330, 0x00},  // MIPI_RX : MIPI_RX0 | (Default) RSVD (Port Configuration): 1x4
    {0xC4, 0x0383, 0x00},  // MIPI_RX_EXT : EXT11 | Tun_Mode (Tunnel Mode): Disabled
    {0xC4, 0x0331, 0x30},  // MIPI_RX : MIPI_RX1 | (Default) ctrl1_num_lanes (Port B - Lane Count): 4
    {0xC4, 0x0332, 0xE0},  // MIPI_RX : MIPI_RX2 | (Default) phy1_lane_map (Lane Map - PHY1 D0): Lane 2 | (Default) phy1_lane_map (Lane Map - PHY1 D1): Lane 3
    {0xC4, 0x0333, 0x04},  // MIPI_RX : MIPI_RX3 | (Default) phy2_lane_map (Lane Map - PHY2 D0): Lane 0 | (Default) phy2_lane_map (Lane Map - PHY2 D1): Lane 1
    {0xC4, 0x0334, 0x00},  // MIPI_RX : MIPI_RX4 | (Default) phy1_pol_map (Polarity - PHY1 Lane 0): Normal | (Default) phy1_pol_map (Polarity - PHY1 Lane 1): Normal
    {0xC4, 0x0335, 0x00},  // MIPI_RX : MIPI_RX5 | (Default) phy2_pol_map (Polarity - PHY2 Lane 0): Normal | (Default) phy2_pol_map (Polarity - PHY2 Lane 1): Normal | (Default) phy2_pol_map (Polarity - PHY2 Clock Lane): Normal
    // Controller to Pipe Mapping Configuration
    {0xC4, 0x0308, 0x64},  // FRONTTOP : FRONTTOP_0 | (Default) RSVD (CLK_SELZ): Port B | (Default) START_PORTB (START_PORTB): Enabled
    {0xC4, 0x0311, 0x40},  // FRONTTOP : FRONTTOP_9 | (Default) START_PORTBZ (START_PORTBZ): Start Video
    {0xC4, 0x0318, 0x6C},  // FRONTTOP : FRONTTOP_16 | mem_dt1_selz (mem_dt1_selz): 0x6C
    // Double Mode Configuration
    {0xC4, 0x0313, 0x40},  // FRONTTOP : FRONTTOP_11 | bpp12dblz (bpp12dblz): Send 12-bit pixels as 24-bit
    {0xC4, 0x031E, 0x38},  // FRONTTOP : FRONTTOP_22 | (Default) soft_bppz (soft_bppz): 0x18 | soft_bppz_en (soft_bppz_en): Software override enabled
    // Pipe Configuration
    {0xC4, 0x005B, 0x01},  // CFGV__VIDEO_Z : TX3 | TX_STR_SEL (TX_STR_SEL Pipe Z): 0x1
    //
    // INSTRUCTIONS FOR DESERIALIZER MAX9296A
    //
    // Video Pipes And Routing Configuration
    {0x90, 0x0050, 0x00},  // (Default)  (STR_SELX): 0x0
    {0x90, 0x0051, 0x01},  // // (Default)  (STR_SELY): 0x1
    // Pipe to Controller Mapping Configuration
    {0x90, 0x040B, 0x07},  // (MAP_EN_L Pipe X): 0x7
    {0x90, 0x040C, 0x00},  // (Default)  (MAP_EN_H Pipe X): 0x0
    {0x90, 0x040D, 0x2C},  // (MAP_SRC_0 Pipe X DT): 0x2C | (Default)  (MAP_SRC_0 Pipe X VC): 0x0
    {0x90, 0x040E, 0x2C},  // (MAP_DST_0 Pipe X DT): 0x2C | (Default)  (MAP_DST_0 Pipe X VC): 0x0
    {0x90, 0x040F, 0x00},  // (Default)  (MAP_SRC_1 Pipe X DT): 0x0 | (Default)  (MAP_SRC_1 Pipe X VC): 0x0
    {0x90, 0x0410, 0x00},  // (Default)  (MAP_DST_1 Pipe X DT): 0x0 | (Default)  (MAP_DST_1 Pipe X VC): 0x0
    {0x90, 0x0411, 0x01},  // (MAP_SRC_2 Pipe X DT): 0x1 | (Default)  (MAP_SRC_2 Pipe X VC): 0x0
    {0x90, 0x0412, 0x01},  // (MAP_DST_2 Pipe X DT): 0x1 | (Default)  (MAP_DST_2 Pipe X VC): 0x0
    {0x90, 0x042D, 0x15},  // (MAP_DPHY_DST_0 Pipe X): 0x1 |  (MAP_DPHY_DST_1 Pipe X): 0x1 |  (MAP_DPHY_DST_2 Pipe X): 0x1
    {0x90, 0x044B, 0x07},  // (MAP_EN_L Pipe Y): 0x7
    {0x90, 0x044C, 0x00},  // (Default)  (MAP_EN_H Pipe Y): 0x0
    {0x90, 0x044D, 0x2C},  // (MAP_SRC_0 Pipe Y DT): 0x2C | (Default)  (MAP_SRC_0 Pipe Y VC): 0x0
    {0x90, 0x044E, 0x2C},  // (MAP_DST_0 Pipe Y DT): 0x2C | (Default)  (MAP_DST_0 Pipe Y VC): 0x0
    {0x90, 0x044F, 0x00},  // (Default)  (MAP_SRC_1 Pipe Y DT): 0x0 | (Default)  (MAP_SRC_1 Pipe Y VC): 0x0
    {0x90, 0x0450, 0x00},  // (Default)  (MAP_DST_1 Pipe Y DT): 0x0 | (Default)  (MAP_DST_1 Pipe Y VC): 0x0
    {0x90, 0x0451, 0x01},  // (MAP_SRC_2 Pipe Y DT): 0x1 | (Default)  (MAP_SRC_2 Pipe Y VC): 0x0
    {0x90, 0x0452, 0x01},  // (MAP_DST_2 Pipe Y DT): 0x1 | (Default)  (MAP_DST_2 Pipe Y VC): 0x0
    {0x90, 0x046D, 0x2A},  // (MAP_DPHY_DST_0 Pipe Y): 0x2 |  (MAP_DPHY_DST_1 Pipe Y): 0x2 |  (MAP_DPHY_DST_2 Pipe Y): 0x2
    // Double Mode Configuration
    {0x90, 0x0473, 0x01},  // (ALT_MEM_MAP12 CTRL1): Alternate memory map enabled
    {0x90, 0x04B3, 0x01},  // (ALT_MEM_MAP12 CTRL2): Alternate memory map enabled
    // MIPI D-PHY Configuration
    {0x90, 0x0330, 0x04},  // (Default)  (Port Configuration): 2 (1x4)
    {0x90, 0x044A, 0xD0},  // (Default)  (Port A - Lane Count): 4
    {0x90, 0x0333, 0x4E},  // (Default)  (Lane Map - PHY0 D0): Lane 2 | (Default)  (Lane Map - PHY0 D1): Lane 3 | (Default)  (Lane Map - PHY1 D0): Lane 0 | (Default)  (Lane Map - PHY1 D1): Lane 1
    {0x90, 0x0335, 0x00},  // (Default)  (Polarity - PHY0 Lane 0): Normal | (Default)  (Polarity - PHY0 Lane 1): Normal | (Default)  (Polarity - PHY1 Lane 0): Normal | (Default)  (Polarity - PHY1 Lane 1): Normal | (Default)  (Polarity - PHY1 Clock Lane): Normal
    {0x90, 0x1D00, 0xF4},  // (config_soft_rst_n - PHY1): 0x0
    // This is to set predefined (coarse) CSI output frequency
    // CSI Phy 1 is 1200 Mbps/lane.
    {0x90, 0x0320, 0x2C},
    {0x90, 0x1D00, 0xF5},  // (config_soft_rst_n - PHY1): 0x1
    {0x90, 0x048A, 0xD0},  // (Default)  (Port B - Lane Count): 4
    {0x90, 0x0334, 0xE4},  // (Default)  (Lane Map - PHY2 D0): Lane 0 | (Default)  (Lane Map - PHY2 D1): Lane 1 | (Default)  (Lane Map - PHY3 D0): Lane 2 | (Default)  (Lane Map - PHY3 D1): Lane 3
    {0x90, 0x0336, 0x00},  // (Default)  (Polarity - PHY2 Lane 0): Normal | (Default)  (Polarity - PHY2 Lane 1): Normal | (Default)  (Polarity - PHY3 Lane 0): Normal | (Default)  (Polarity - PHY3 Lane 1): Normal | (Default)  (Polarity - PHY2 Clock Lane): Normal
    {0x90, 0x1E00, 0xF4},  // (config_soft_rst_n - PHY2): 0x0
    // This is to set predefined (coarse) CSI output frequency
    // CSI Phy 2 is 1200 Mbps/lane.
    {0x90, 0x0323, 0x34},
    {0x90, 0x1E00, 0xF5},  // (config_soft_rst_n - PHY2): 0x1
    {0x90, 0x0313, 0x02},  // (CSI_OUT_EN): CSI output enabled
    // Video Transmit Configuration for Serializer(s)
    {0x84, 0x0002, 0x43},  // DEV : REG2 | VID_TX_EN_Z (VID_TX_EN_Z): Enabled
    {0xC4, 0x0002, 0x43},  // DEV : REG2 | VID_TX_EN_Z (VID_TX_EN_Z): Enabled

    {0x90, 0x0005, 0x00},  // Deserializer REG5
    {0x84, 0x0005, 0x00},  // Serializer A REG5
    {0xC4, 0x0005, 0x00},  // Serializer B REG5

    // GPIO settings serializer Link A (0x84)
    // A DMUX1 DES1->SER3
    // rx & tx ID = 0x1
    {0x90, 0x02B3, 0x9B},
    {0x90, 0x02B4, 0xA1},
    {0x90, 0x02B5, 0x41},
    {0x84, 0x02C7, 0x84},
    {0x84, 0x02C8, 0xA1},
    {0x84, 0x02C9, 0x41},
    // A RFT DES3<-SER6
    // rx & tx ID = 0x2
    {0x90, 0x02B9, 0x04},
    {0x90, 0x02BA, 0xA2},
    {0x90, 0x02BB, 0x42},
    {0x84, 0x02D0, 0x9B},
    {0x84, 0x02D1, 0xA2},
    {0x84, 0x02D2, 0x42},
    // A RESETN DES8->SER2
    // rx & tx ID = 0x4
    {0x90, 0x02C8, 0x9B},
    {0x90, 0x02C9, 0xA4},
    {0x90, 0x02CA, 0x44},
    {0x84, 0x02C4, 0x84},
    {0x84, 0x02C5, 0xA4},
    {0x84, 0x02C6, 0x44},

    // GPIO settings serializer Link B (0xC4)
    // B RESETN DES10->SER2
    // rx & tx ID = 0x3
    {0x90, 0x02CE, 0x9B},
    {0x90, 0x02CF, 0xA3},
    {0x90, 0x02D0, 0x43},
    {0xC4, 0x02C4, 0x84},
    {0xC4, 0x02C5, 0xA3},
    {0xC4, 0x02C6, 0x43},
    // B DMUX1 DES9->SER3
    // rx & tx ID = 0x5
    {0x90, 0x02CB, 0x9B},
    {0x90, 0x02CC, 0xA5},
    {0x90, 0x02CD, 0x45},
    {0xC4, 0x02C7, 0x84},
    {0xC4, 0x02C8, 0xA5},
    {0xC4, 0x02C9, 0x45},
    // B RFT DES4<-SER6
    // rx & tx ID = 0x6
    {0x90, 0x02BC, 0x04},
    {0x90, 0x02BD, 0xA6},
    {0x90, 0x02BE, 0x46},
    {0xC4, 0x02D0, 0x9B},
    {0xC4, 0x02D1, 0xA6},
    {0xC4, 0x02D2, 0x46},

    // Enable SPI channel on all devices
    {0x90, 0x0170, 0x09},  // Deserializer, enable SPI channel, SPI slave, accept all packets
    {0x84, 0x0170, 0x03},  // Serializer A, enable SPI channel, SPI main, accept packets with ID = 0b00
    {0xC4, 0x0170, 0x53},  // Serializer B, enable SPI channel, SPI main, accept packets with ID = 0b01

    // Enable GMSL negative output
    {0x84, 0x14ce, 0x19},
    {0xC4, 0x14ce, 0x19},

    // Reset MIPI RX and enable continuous clock mode
    {0x84, 0x0330, 0x08},
    {0xC4, 0x0330, 0x08},

    // Reset MIPI RX and enable non-continuous clock mode
    //{0x84,0x0330,0x48},
    //{0xC4,0x0330,0x48},

    {MAX929X_DELAY, 0x0000, 0x78},  // 120 msec delay

    // Unset mipi_rx_reset and enable continuous clock mode
    {0x84, 0x0330, 0x00},
    {0xC4, 0x0330, 0x00},

    // Unset mipi_rx_reset and enable non-continuous clock mode
    // {0x84,0x0330,0x40},
    // {0xC4,0x0330,0x40},
};

#endif
