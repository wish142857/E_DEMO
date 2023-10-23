#ifndef TIMER_HPP
#define TIMER_HPP

#include <ctime>
#include <chrono>

/*************
 * @def Timer
 * @note 1. TIMER_START(tag);  2. TIMER_END(tag); 3. TIMER_DURATION_?(tag);
 *************/
#define TIMER_START(tag) auto tag##_start = std::chrono::steady_clock::now(), tag##_end = tag##_start
#define TIMER_END(tag) tag##_end = std::chrono::steady_clock::now()
#define TIMER_DURATION_S(tag) std::chrono::duration_cast<std::chrono::seconds>(tag##_end - tag##_start).count())
#define TIMER_DURATION_MS(tag) std::chrono::duration_cast<std::chrono::milliseconds>(tag##_end - tag##_start).count())
#define TIMER_DURATION_US(tag) std::chrono::duration_cast<std::chrono::microseconds>(tag##_end - tag##_start).count())
#define TIMER_DURATION_NS(tag) std::chrono::duration_cast<std::chrono::nanoseconds>(tag##_end - tag##_start).count())

namespace e_demo {
    /***************
     * @class Timer
     ***************/
    class Timer {
    public:
        Timer() { tic(); };
        ~Timer() {};

        // @brief  [tic] start timing
        void tic() { 
            tocNum = 0;
            ticTimePoint = std::chrono::steady_clock::now(); 
        };

        // @brief  [toc] get duration
        // @return {int} millisecond
        int toc() { 
            tocNum++;
            tocTimePoint = std::chrono::steady_clock::now();
            return std::chrono::duration_cast<std::chrono::milliseconds>(tocTimePoint - ticTimePoint).count();
        };

        // @brief  [fps] get FPS
        // @return {int} Frames Per Second
        int fps() {
            return tocNum ? tocNum * 1000 / (std::chrono::duration_cast<std::chrono::milliseconds>(tocTimePoint - ticTimePoint).count()) : 0;
        }

    private:
        long tocNum;
        std::chrono::time_point<std::chrono::steady_clock> ticTimePoint, tocTimePoint;
    };
}

#endif
