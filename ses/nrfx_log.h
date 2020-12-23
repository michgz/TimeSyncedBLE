#ifndef __H_NRFX_LOG_CUSTOM
#define __H_NRFX_LOG_CUSTOM

// Must come before the SDK "nrfx_log.h" in the search path. Will override some of the definitions
// in there

#if 0

    #include "./nrf_log.h"

    #define NRFX_LOG_ERROR_STRING_GET NRF_LOG_ERROR_STRING_GET

    // These are defined to be empty. NRFX doesn't really need logging, it's very low-level
    #define NRFX_LOG_INFO(...)   
    #define NRFX_LOG_DEBUG(...)   
    #define NRFX_LOG_WARNING(...)   

    #define NRFX_LOG_HEXDUMP_DEBUG   NRF_LOG_HEXDUMP_DEBUG

#else

    #include "../nrfx/nrfx_log.h"

#endif

#endif //  __H_NRFX_LOG_CUSTOM

