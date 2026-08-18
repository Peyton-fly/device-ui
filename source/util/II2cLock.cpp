#include "util/II2cLock.h"
#include "util/ILog.h"

namespace
{
II2cLock *hostLock = nullptr;
}

void II2cLock::install(II2cLock *lock)
{
    // Installing null must not clear an already-installed lock.
    if (!lock)
        return;

    if (hostLock && hostLock != lock)
        ILOG_WARN("II2cLock: replacing the installed bus lock");

    hostLock = lock;
}

II2cLock *II2cLock::installed(void)
{
    return hostLock;
}
