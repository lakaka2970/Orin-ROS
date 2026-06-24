/*
 * (c) (2022-2023), Infineon Technologies AG, or an affiliate of Infineon Technologies AG. All rights reserved.
 *
 * Use of this file is subject to the terms of use agreed between (i) you or
 * the company in which ordinary course of business you are acting and (ii)
 * Infineon Technologies AG or its licensees.
 */

#ifndef IFXRFE_CTRXOPCODES_H
#define IFXRFE_CTRXOPCODES_H 1

// Opcodes used in SPI frames
#define IFXRFE_OPCODE_NOP                          0x1
#define IFXRFE_OPCODE_TRIGGER_RESET                0x8
#define IFXRFE_OPCODE_EXECUTE_HANDLE               0x9
#define IFXRFE_OPCODE_EXECUTE_DIRECTLY_FW_CMD      0xD
#define IFXRFE_OPCODE_CONFIGURE_HANDLE             0xE
#define IFXRFE_OPCODE_CONFIGURE_AND_EXECUTE_HANDLE 0xF

#endif  //IFXRFE_CTRXOPCODES_H
