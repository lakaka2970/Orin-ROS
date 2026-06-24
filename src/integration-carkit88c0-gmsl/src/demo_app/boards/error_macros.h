#ifndef ERROR_MACROS_H
#define ERROR_MACROS_H 1

#include "IfxRfe_ErrorDefinitions.h"
#include "PlatformErrors.h"

#define EXIT_ON_IFXRFE_ERROR(expression, cleanup)                                                                                                     \
    {                                                                                                                                        \
        error_t const ret_code = expression;                                                                                                 \
        if (ret_code != IFXRFE_E_SUCCESS)                                                                                                    \
        {                                                                                                                                    \
            cleanup;                                                                                                              \
            printf("Error in function %s() in file %s:%d -> IfxRfe returned %d (0x%X)\n", __func__, __FILE__, __LINE__, ret_code, ret_code); \
            return ret_code;                                                                                                                 \
        }                                                                                                                                    \
    }

#define EXIT_ON_PLATFORM_ERROR(expression, cleanup)                                                                                                     \
    {                                                                                                                                        \
        error_t const ret_code = expression;                                                                                                 \
        if (ret_code != PLATFORM_JETSON_E_SUCCESS)                                                                                                    \
        {                                                                                                                                    \
            cleanup;                                                                                                              \
            printf("Error in function %s() in file %s:%d -> Platform returned %d (0x%X)\n", __func__, __FILE__, __LINE__, ret_code, ret_code); \
            return ret_code;                                                                                                                 \
        }                                                                                                                                    \
    }

#endif //ERROR_MACROS_H
