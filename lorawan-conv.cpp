#include "lorawan-conv.h"

bool isDEVADDREmpty(const DEVADDR &addr)
{
    return *((uint32_t *) &addr) == 0;
}

bool isDEVEUIEmpty(const DEVEUI &eui)
{
    return *((uint64_t *) &eui) == 0;
}

uint32_t DEVADDR2int(
	const DEVADDR &value
)
{
    uint32_t retval;
    *((uint32_t*) &retval) = NTOH4(*((uint32_t*) &value));
    return retval;
}

void int2DEVADDR(
	DEVADDR &retval,
	uint32_t value
)
{
	// *((uint32_t*) &retval) = NTOH4(value);
	*((uint32_t*) &retval) = value;
}

uint32_t NETID2int(
	const NETID &value
) {
	return value.c[0] + (value.c[1] << 8) + (value.c[2] << 16);
}

void int2NETID(
	NETID &retval,
	uint32_t value
)
{
	retval.c[0] = value & 0xff;
	retval.c[1] = (value >> 8) & 0xff;
	retval.c[2] = (value >> 16) & 0xff;
}

uint32_t JOINNONCE2int(
	const JOINNONCE &value
) {
	return value.c[0] + (value.c[1] << 8) + (value.c[2] << 16);
}

int FREQUENCY2int(
	const FREQUENCY &frequency
) {
	return frequency[0] + (frequency[1] << 8) + (frequency[2] << 16);
}

void int2JOINNONCE(
    JOINNONCE &retVal,
    int value
)
{
    uint32_t r = NTOH4(value);
    retVal.c[0] = r & 0xff;
    retVal.c[1] = (r >> 8) & 0xff;
    retVal.c[2] = (r >> 16) & 0xff;
}

void int2APPNONCE(
    APPNONCE& retVal,
    int value
)
{
    uint32_t r = NTOH4(value);
    retVal.c[0] = r & 0xff;
    retVal.c[1] = (r >> 8) & 0xff;
    retVal.c[2] = (r >> 16) & 0xff;
}
