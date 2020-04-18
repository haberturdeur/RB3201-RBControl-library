#pragma once

#include <esp_timer.h>

#include <memory>
#include <mutex>
#include <vector>

namespace rb {

class Manager;
class Timers;

class Timers {
    friend class Manager;

public:
    static constexpr uint16_t INVALID_ID = 0;

    ~Timers();

    /**
     * \brief Schedule callback to fire after period (in millisecond).
     *
     * Return true from the callback to schedule periodically, false to not (singleshot timer).
     *
     * \param period_ms is period in which will be the schedule callback fired
     * \param callback is a function which will be schedule with the set period.
     * \return timer ID that you can use to cancel the timer.
     */
    uint16_t schedule(uint32_t period_ms, std::function<bool()> callback);

    bool reset(uint16_t id, uint32_t period_ms);
    bool cancel(uint16_t id);

private:
    struct callback_arg_t {
        Timers* self;
        uint16_t id;
    };

    struct timer_t {
        std::function<bool()> callback;
        esp_timer_handle_t handle;
        std::unique_ptr<callback_arg_t> args;
    };

    static void timerCallback(void* handleVoid);

    Timers(Manager& man);

    void cancelByIdxLocked(size_t idx);
    uint16_t getFreeIdLocked();

    std::vector<timer_t> m_timers;
    std::recursive_mutex m_mutex;
    uint16_t m_id_counter;
};

};
