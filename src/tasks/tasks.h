#pragma once
// Task starters. Layout per research doc 04 s7.2 (see config.h ladder).

namespace task_control { void start(); }  // core 1, 500 Hz, prio 20
namespace task_avoid   { void start(); }  // core 1, 50 Hz, prio 15
namespace task_swarm   { void start(); }  // core 0, event + 10 Hz beacon
namespace task_telem   { void start(); }  // core 0, 5 Hz (+ health checks)
