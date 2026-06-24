/*
 * (c) (2022-2025), Infineon Technologies AG, or an affiliate of Infineon Technologies AG. All rights reserved.
 *
 * Use of this file is subject to the terms of use agreed between (i) you or
 * the company in which ordinary course of business you are acting and (ii)
 * Infineon Technologies AG or its licensees.
 */

#ifndef SEQUENCER_DATA_H
#define SEQUENCER_DATA_H

#include <stdint.h>

/**
 * @brief Length of the sequencer program in 32-bit words
 */
#define SEQUENCER_PROGRAM_LEN 43

/**
 * @brief Sequencer data
 * 
 * This array contains 1335 32-bit words that define the radar timing
 * and control sequences for the primary CTRX device.
 */
extern const uint32_t seqData_standalone[SEQUENCER_PROGRAM_LEN];

#endif /* SEQUENCER_DATA_H */
