#include <iomanip>
#include <iostream>
#include <cstring>
#include <csignal>
#include <fcntl.h>

#ifdef _MSC_VER
#else
#include <execinfo.h>

#endif

#include "argtable3/argtable3.h"

#include "packet-forwarder/libloragw-helper.h"
#include "lorawan/lorawan-error.h"
#include "lorawan/lorawan-msg.h"
#include "lorawan/lorawan-date.h"
#include "lorawan/lorawan-string.h"

// generated gateway regional settings source code
#include "config/gateway_usb_conf.h"
#include "lorawan-gateway-listener.h"
#include "scheduler.h"
#include "schedule-item-concentrator.h"

static LorawanGatewaySettings* findLorawanGatewaySettingsByRegionName(
    const char *name
)
{
    size_t len = strlen(name);
    for (int i = 0; i < 2; i++) {
        for (LorawanGatewaySettings &it: lorawanGatewaySettings) {
            if (strncmp(it.name, name, len) == 0)
                return &it;
        }
        // uppercase
        for (int c = 0; c < len; c++) {
            if (::isalpha(name[c]))
                ((unsigned char *)name)[c] = name[c] ^ 32;
        }
    }

    return &lorawanGatewaySettings[0];
}

static void printRegionNames(std::ostream &strm)
{
    for (const auto& it: lorawanGatewaySettings) {
        strm << it.name << " ";
    }
}

// listen() flags parameter values
// 1- Do not send messages
#define FLAG_GATEWAY_LISTENER_NO_SEND   1
// 2- Do not send beacons
#define FLAG_GATEWAY_LISTENER_NO_BEACON 2

class PosixLibLoragwOpenClose : public LibLoragwOpenClose {
private:
    std::string devicePath;
public:
    explicit PosixLibLoragwOpenClose(
        const std::string &aDevicePath
    ) : devicePath(aDevicePath)
    {

    };

    int openDevice(const char *fileName, int mode) override
    {
        return open(devicePath.c_str(), mode);
    };

    int closeDevice(int fd) override
    {
        return close(fd);
    };
};

const std::string programName = "lorawan-gateway";
#ifdef _MSC_VER
#undef ENABLE_TERM_COLOR
#else
#define ENABLE_TERM_COLOR	1
#endif

class LocalGatewayConfiguration {
public:
    std::string devicePath;
    std::string identityFileName;
    LorawanGatewaySettings *gwSettings;
    bool enableSend;
    bool enableBeacon;
    int verbosity;
    LocalGatewayConfiguration()
        : gwSettings(nullptr), enableSend(false), enableBeacon(false), verbosity(0)
    {

    }
};

static LocalGatewayConfiguration localConfig;

static LoraGatewayListener *listener;

static void stop()
{
    if (listener)
        listener->stop();
}

static LibLoragwHelper libLoragwHelper;

static void done()
{
    if (libLoragwHelper.onOpenClose) {
        delete libLoragwHelper.onOpenClose;
        libLoragwHelper.onOpenClose = nullptr;
    }
}

class StdErrLog: public LogIntf {
public:
    void onStarted(
        void *env,
        uint64_t gatewayId,
        const std::string &regionName
    ) override
    {
        std::cout << "gateway: " << std::hex << gatewayId << ", region: " << regionName << std::endl;
    }

    void onFinished(
        void *env,
        const std::string &message
    ) override
    {
        std::cout << "done " << message << std::endl;
    }

    void onInfo(
        void *env,
        int level,
        int moduleCode,
        int errorCode,
        const std::string &message
    ) override {
        if (env) {
            if (localConfig.verbosity < level)
                return;
        }
        struct timeval t{};
        gettimeofday(&t, nullptr);
        std::cerr << timeval2string(t);
#ifdef ENABLE_TERM_COLOR
        if (isatty(2))  // if stderr is piped to the file, do not put ANSI color to the file
            std::cerr << "\033[" << logLevelColor(level)  << "m";
#endif
        std::cerr << " " << std::setw(LOG_LEVEL_FIELD_WIDTH) << std::left << logLevelString(level);
#ifdef ENABLE_TERM_COLOR
        if (isatty(2))
            std::cerr << "\033[0m";
#endif
        std::cerr << message << std::endl;
        if (level == LOG_ALERT) {
            stop();
        }
    }
};

/**
 * Parse command line
 * Return 0- success
 *        1- show help and exit, or command syntax error
 *        2- output file does not exists or can not open to write
 **/
int parseCmd(
    LocalGatewayConfiguration *config,
    int argc,
    char *argv[])
{
    // device path
    struct arg_str *a_device_path = arg_str1(nullptr, nullptr, "<device-name>", "USB gateway device e.g. /dev/ttyACM0");
    struct arg_str *a_region_name = arg_str1("c", "region", "<region-name>", R"(Region name, e.g. "EU433" or "US")");
    struct arg_str *a_identity_file_name = arg_str0("i", "id", "<id-file-name>", "Device identities JSON file name");
    struct arg_lit *a_enable_send = arg_lit0("s", "allow-send", "Allow send");
    struct arg_lit *a_enable_beacon = arg_lit0("b", "allow-beacon", "Allow send beacon");
    struct arg_lit *a_verbosity = arg_litn("v", "verbose", 0, 7, "Verbosity level -v alert, -vv critical error, -vvv error, -vvvv warning, "
        "-vvvvv significant info, -vvvvvv  info, -vvvvvvv  debug");
    struct arg_lit *a_help = arg_lit0("?", "help", "Show this help");
    struct arg_end *a_end = arg_end(20);

    void *argtable[] = {
        a_device_path, a_region_name, a_identity_file_name,
        a_enable_send, a_enable_beacon,
        a_verbosity, a_help, a_end
    };

    // verify the argtable[] entries were allocated successfully
    if (arg_nullcheck(argtable) != 0) {
        arg_freetable(argtable, sizeof(argtable) / sizeof(argtable[0]));
        return ERR_CODE_PARAM_INVALID;
    }
    // Parse the command line as defined by argtable[]
    int nErrors = arg_parse(argc, argv, argtable);

    if (a_device_path->count)
        config->devicePath = std::string(*a_device_path->sval);
    else
        config->devicePath = "";
    if (a_identity_file_name->count)
        config->identityFileName = *a_identity_file_name->sval;
    else
        config->identityFileName = "";

    if (a_region_name->count)
        config->gwSettings = findLorawanGatewaySettingsByRegionName(*a_region_name->sval);
    else
        config->gwSettings = &lorawanGatewaySettings[0];

    config->enableSend = (a_enable_send->count > 0);
    config->enableBeacon = (a_enable_beacon->count > 0);

    config->verbosity = a_verbosity->count;

    // special case: '--help' takes precedence over error reporting
    if ((a_help->count) || nErrors) {
        if (nErrors)
            arg_print_errors(stderr, a_end, programName.c_str());
        std::cerr << "Usage: " << programName << std::endl;
        arg_print_syntax(stderr, argtable, "\n");
        std::cerr << MSG_PROG_NAME_GATEWAY_USB << std::endl;
        arg_print_glossary(stderr, argtable, "  %-25s %s\n");
        std::cerr << "  region names: ";
        printRegionNames(std::cerr);
        std::cerr << std::endl;
        arg_freetable(argtable, sizeof(argtable) / sizeof(argtable[0]));
        return ERR_CODE_PARAM_INVALID;
    }

    arg_freetable(argtable, sizeof(argtable) / sizeof(argtable[0]));
    return CODE_OK;
}

#ifdef _MSC_VER
#undef ENABLE_TERM_COLOR
#else
#define ENABLE_TERM_COLOR	1
#endif

#define TRACE_BUFFER_SIZE   256

static void printTrace() {
#ifdef _MSC_VER
#else
    void *t[TRACE_BUFFER_SIZE];
    auto size = backtrace(t, TRACE_BUFFER_SIZE);
    backtrace_symbols_fd(t, size, STDERR_FILENO);
#endif
}

static StdErrLog errLog;

static void init();
static void run();

void signalHandler(int signal)
{
    // lastSysSignal = signal;
    switch (signal)
    {
        case SIGINT:
            std::cerr << MSG_INTERRUPTED << std::endl;
            stop();
            done();
            break;
        case SIGSEGV:
            std::cerr << ERR_SEGMENTATION_FAULT << std::endl;
            printTrace();
            exit(ERR_CODE_SEGMENTATION_FAULT);
        case SIGABRT:
            std::cerr << ERR_ABRT << std::endl;
            printTrace();
            exit(ERR_CODE_ABRT);
#ifndef _MSC_VER
        case SIGHUP:
            std::cerr << ERR_HANGUP_DETECTED << std::endl;
            break;
        case SIGUSR2:	// 12
            std::cerr << MSG_SIG_FLUSH_FILES << std::endl;
            // flushFiles();
            break;
#endif
        case 42:	// restart
            std::cerr << MSG_RESTART_REQUEST << std::endl;
            stop();
            done();
            init();
            run();
            break;
        default:
            break;
    }
}

void setSignalHandler()
{
#ifndef _MSC_VER
    struct sigaction action{};
    // memset(&action, 0, sizeof(struct sigaction));
    action.sa_handler = &signalHandler;
    sigaction(SIGINT, &action, nullptr);
    sigaction(SIGHUP, &action, nullptr);
    sigaction(SIGSEGV, &action, nullptr);
    sigaction(SIGABRT, &action, nullptr);
    sigaction(SIGUSR2, &action, nullptr);
    sigaction(42, &action, nullptr);
#endif
}

static void run()
{
    setSignalHandler();
    listener->log(LOG_DEBUG, CODE_OK, MSG_LISTENER_RUN);
    libLoragwHelper.bind(&errLog, new PosixLibLoragwOpenClose(localConfig.devicePath));

    int flags = 0;
    if (!localConfig.enableSend)
        flags |= FLAG_GATEWAY_LISTENER_NO_SEND;
    if (!localConfig.enableBeacon)
        flags |= FLAG_GATEWAY_LISTENER_NO_BEACON;

    listener->setConfig(localConfig.gwSettings);
    if (localConfig.verbosity) {
        print_header_lgw_pkt_rx_s(std::cout);
        std::cout << std::endl;
    }
    int r = listener->run();
    if (r) {
        std::stringstream ss;
        ss << ERR_MESSAGE << r << ": " << strerror_lorawan_ns(r) << std::endl;
        listener->log(LOG_INFO, r, ss.str());
    }
}

static void init()
{
    if (localConfig.identityFileName.empty()) {
        // std::cerr << ERR_WARNING << ERR_CODE_INIT_IDENTITY << ": " << ERR_INIT_IDENTITY << std::endl;
        if (localConfig.verbosity > 0)
            std::cerr << MSG_NO_IDENTITIES << std::endl;
    }
    libLoragwHelper.bind(&errLog, nullptr);

    listener = new LoraGatewayListener([]() {
        return new ScheduleItemConcentrator;
    }, 32);
    // signal is not required in USB listener
    // listener->setSysSignalPtr(&lastSysSignal);
    listener->setOnLog(&errLog, localConfig.verbosity);
    listener->setOnUpstream(
        [](const LoraGatewayListener *listener, struct lgw_pkt_rx_s *packet) {
            if (localConfig.verbosity) {
                print_lgw_pkt_rx_s(std::cout, packet);
                std::cout << std::endl;
            } else {
                std::cout << hexString((char *) packet->payload, packet->size) << std::endl;
            }
        }
    );
}

int main(
	int argc,
	char *argv[])
{
    if (parseCmd(&localConfig, argc, argv) != 0)
        exit(ERR_CODE_COMMAND_LINE);
    init();
    run();
    done();
    return 0;
}
