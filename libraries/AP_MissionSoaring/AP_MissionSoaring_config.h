#include <AP_HAL/AP_HAL_Boards.h>

#ifndef HAL_MISSIONSOARING_ENABLED
    #if !defined(HAL_MINIMIZE_FEATURES) && defined(HAVE_OS_POSIX_IO)
        #define HAL_MISSIONSOARING_ENABLED 1
    #else
        #define HAL_MISSIONSOARING_ENABLED 0
    #endif
#endif