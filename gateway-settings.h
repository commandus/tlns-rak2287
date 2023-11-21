#ifndef GATEWAY_SETTINGS
#define GATEWAY_SETTINGS 1

/**
 * Lora gateway settings structure access method
 */
#include <string>
#include "gw/gateway-lora.h"

class LorawanGatewaySettings {
public:
    sx1261_config_t sx1261;
    sx130x_config_t sx130x;
    gateway_t gateway;
    struct lgw_conf_debug_s debug;
    std::string serverAddr;
    std::string gpsTtyPath;
    const char* name;
};

#endif