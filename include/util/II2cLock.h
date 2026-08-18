#pragma once

/**
 * @brief Host-provided lock that serializes the UI's I2C traffic with the
 * firmware's Wire users on a shared bus (touch controller, backlight driver).
 *
 * Guards are no-ops when no implementation is installed.
 *
 * Contract:
 *  - Implementations must be reentrant; call sites nest on one task.
 *  - Host lock order is spiLock -> II2cLock, never reversed.
 *  - Steady-state guards must not contain logging, delays, or
 *    Wire.end()/Wire.begin().
 *  - Boot-time one-shot init guards are exempt from the rule above; the
 *    exemption must not be extended to steady-state paths.
 */
class II2cLock
{
  public:
    virtual ~II2cLock() = default;
    virtual void lock(void) = 0;
    virtual void unlock(void) = 0;

    /**
     * Static holder: the LVGL touch callback has no `this` to reach a member.
     */
    static void install(II2cLock *lock);
    static II2cLock *installed(void);

    /// RAII: hold the bus for the enclosing scope. Safe when nothing is installed.
    class Guard
    {
      public:
        Guard(void) : held(II2cLock::installed())
        {
            if (held)
                held->lock();
        }
        ~Guard()
        {
            if (held)
                held->unlock();
        }
        Guard(const Guard &) = delete;
        Guard &operator=(const Guard &) = delete;

      private:
        II2cLock *held;
    };
};
