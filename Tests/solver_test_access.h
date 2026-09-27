#ifndef SOLVER_TEST_ACCESS_H
#define SOLVER_TEST_ACCESS_H

// Test-only white-box access to Solver / Emulator / SharedMem internals
// (computed moments, trajectory buffers, meshes, shared-memory segments)
// without adding accessors to production code.
//
// Every header these classes depend on is included first, with normal access
// rules, so that the `private`/`protected` redefinition below only affects
// the project's own class definitions. It must be included before any other
// project header that pulls in solver.h / emulator.h / sharedmem.h.

#include "gtest/gtest.h"

#include <QSettings>
#include <QSharedMemory>
#include <QString>
#include <qsettings.h>

#include <algorithm>
#include <array>
#include <cassert>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstring>
#include <exception>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <mutex>
#include <queue>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "my_math.h"
#include "robo.h"
#include "meshcollider.h"

#define private public
#define protected public
#include "sharedmem.h"
#include "emulator.h"
#include "solver.h"
#undef protected
#undef private

#endif // SOLVER_TEST_ACCESS_H
