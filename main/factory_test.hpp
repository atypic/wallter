#pragma once

#include "sdkconfig.h"

namespace wallter::factory_test {

#if CONFIG_WALLTER_FACTORY_TEST
[[noreturn]] void run();
#endif

} // namespace wallter::factory_test
