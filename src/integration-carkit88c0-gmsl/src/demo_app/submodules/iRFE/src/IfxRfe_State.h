/*
 * (c) (2022-2023), Infineon Technologies AG, or an affiliate of Infineon Technologies AG. All rights reserved.
 *
 * Use of this file is subject to the terms of use agreed between (i) you or
 * the company in which ordinary course of business you are acting and (ii)
 * Infineon Technologies AG or its licensees.
 */
#ifndef IFXRFE_STATE_H
#define IFXRFE_STATE_H 1

#include <stdbool.h>

#define RETURN_ON_NOT_INITIALIZED()      \
    if (!IsInitialized())                \
    {                                    \
        return IFXRFE_E_NOT_INITIALIZED; \
    }


///\brief Sets the flag whether the IfxRfe is initialized
///\param initialized True if initialized, false otherwise
void SetInitialized(bool initialized);

///\brief Gets the flag that determines, whether the IfxRfe is initialized
///\return initialized flag
bool IsInitialized();

#endif  //IFXRFE_STATE_H