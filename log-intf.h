#ifndef LOG_INTF_H_
#define LOG_INTF_H_	1

#include "lorawan-types.h"

class LogIntf {
public:
    virtual void onInfo(
        void *env,
        int level,
        int moduleCode,
        int errorCode,
        const std::string &message
    ) = 0;
    virtual void onStarted(void *env, uint64_t gatewayId, const std::string &regionName) = 0;
    virtual void onFinished(void *env, const std::string &message) = 0;
};

#endif
