#pragma once

#include_next <features.h>

#undef __GLIBC_USE_ISOC2X
#define __GLIBC_USE_ISOC2X 0
#undef __GLIBC_USE_C2X_STRTOL
#define __GLIBC_USE_C2X_STRTOL 0

#undef __GLIBC_USE_ISOC23
#define __GLIBC_USE_ISOC23 0
#undef __GLIBC_USE_C23_STRTOL
#define __GLIBC_USE_C23_STRTOL 0
