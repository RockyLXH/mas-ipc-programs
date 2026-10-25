#pragma once

#include <iostream>
#include <chrono>
#include <iomanip>


namespace utility
{
    

namespace cleaner
{
/*
 * Using Cleaner as static or global declare for clean up things when the program closed or terminated
 * usage: Cleaner{cleanup_func} as static or global declare
 */
struct Cleaner
{
private:
    using Fn = void (*)(void);
    Fn fn_;

public:
    Cleaner(Fn fn) : fn_(fn) {}
    ~Cleaner()
    {
        std::cout << "Cleaning up\n";
        (*fn_)();
    }
};
}

namespace timer
{
/*
 * Using Timer in function for measuring the time elapsed
 * usage: Timer timer; at the beginning of function
 */

struct Timer
{
private:
    std::chrono::steady_clock::time_point start_;

public:
    Timer()
    {
        start_ = std::chrono::steady_clock::now();
    }

    ~Timer()
    {
        auto dur = std::chrono::steady_clock::now() - start_;
        std::cout << "function: " << __PRETTY_FUNCTION__ << " took [" << dur.count() << " us]" << "\n";
    }
};
}

namespace timestamp
{
/*
 * Using it in std::cout stream for adding the time stamp
 * usage: declare it globally ex. 'Timestamp ts', and then 'std::cout << ts << 'your code' << std::endl;'
*/
struct Timestamp {};

// 重载 << 运算符
std::ostream& operator<<(std::ostream& os, const Timestamp&) {
    auto now = std::chrono::system_clock::now();
    auto us = std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch()) % 1000000;

    auto time_t_now = std::chrono::system_clock::to_time_t(now);

    os << "[" << std::put_time(std::localtime(&time_t_now), "%Y-%m-%d %H:%M:%S") << '.' << std::setfill('0') << std::setw(6) << us.count() << "] ";
    return os;
}
}

} // namespace utility