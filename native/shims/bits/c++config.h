// Wrapper to override libstdc++'s init-priority selection for <iostream>.
// We include the real header, then flip the macro so that
//   #if !(_GLIBCXX_USE_INIT_PRIORITY_ATTRIBUTE && __has_attribute(__init_priority__))
// takes the first branch (static ios_base::Init), avoiding the
// export of the versioned undefined std::ios_base_library_init().

#pragma once

#include_next <bits/c++config.h>

#undef _GLIBCXX_USE_INIT_PRIORITY_ATTRIBUTE
#define _GLIBCXX_USE_INIT_PRIORITY_ATTRIBUTE 0

#undef _GLIBCXX_EXTERN_TEMPLATE
#define _GLIBCXX_EXTERN_TEMPLATE 0
