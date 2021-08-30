#pragma once

#include "_AXP192.hpp"
#include <mutex>

extern std::mutex gui_mutex;

extern _AXP192 pmu;

void m5core2_init();
