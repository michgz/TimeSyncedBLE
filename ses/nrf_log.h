#ifndef __H_NRF_LOG_CUSTOM
#define __H_NRF_LOG_CUSTOM

// Must come before the SDK "nrf_log.h" in the search path. Will override some of the definitions
// in there

#if 0

    #include <__cross_studio_io.h>

    #if NRF_MODULE_ENABLED(NRF_LOG)
    #include "nrf_strerror.h"
    #define NRF_LOG_ERROR_STRING_GET(code) nrf_strerror_get(code)
    #else
    #define NRF_LOG_ERROR_STRING_GET(code) ""
    #endif

    #define NRF_LOG_FLUSH()
    #define NRF_LOG_FINAL_FLUSH()
    #define NRF_LOG_INIT(...)     0
    #define NRF_LOG_DEFAULT_BACKENDS_INIT()
    #define NRF_LOG_PROCESS()     0
    #define NRF_LOG_MODULE_REGISTER()

    //#if NRF_LOG_LEVEL > 0
    #if 1

        #define NRF_LOG_INFO(MSG, ...)       debug_printf(MSG "\r\n",  ##__VA_ARGS__ )
        #define NRF_LOG_DEBUG(MSG, ...)      debug_printf(MSG "\r\n",  ##__VA_ARGS__ );
        #define NRF_LOG_ERROR(MSG, ...)      debug_printf(MSG "\r\n",  ##__VA_ARGS__ )
        #define NRF_LOG_WARNING(MSG, ...)    debug_printf(MSG "\r\n",  ##__VA_ARGS__ )

    #else

        #define NRF_LOG_INFO( ...)
        #define NRF_LOG_DEBUG( ...)
        #define NRF_LOG_ERROR( ...)
        #define NRF_LOG_WARNING( ...)

    #endif

    // Currently no solution for hexdumping
    #define NRF_LOG_HEXDUMP_INFO(...)
    #define NRF_LOG_HEXDUMP_DEBUG(...)
    #define NRF_LOG_HEXDUMP_ERROR(...)

    // No support for these macros
    #define NRF_LOG_INST_ERROR(p_inst,...)
    #define NRF_LOG_INST_WARNING(p_inst,...)
    #define NRF_LOG_INST_INFO(p_inst,...)
    #define NRF_LOG_INST_DEBUG(p_inst,...)

#else

    #include "../log/nrf_log.h"

#endif

#endif //  __H_NRF_LOG_CUSTOM

