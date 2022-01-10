
#include "version.h"

#ifdef DEBUG_VERSION
	const char version[] = "D:2.1.0";
#else
	const char version[] = "R:2.1.0";
#endif

const char compiler_date[] = __DATE__;
const char compiler_time[] = __TIME__;
