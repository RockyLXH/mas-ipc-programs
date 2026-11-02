#pragma once

#include <vector>
#include <iostream>
#include <functional>
#include <exception>

class ExitGuard
{
public:
    using Callback = std::function<void()>;

    static ExitGuard &instance()
    {
        static ExitGuard guard;
        return guard;
    }

    void push(Callback cb)
    {
        callbacks.push_back(std::move(cb));
    }

private:
    std::vector<Callback> callbacks;

    ExitGuard() = default;
    ~ExitGuard()
    {
        for (auto it = callbacks.rbegin(); it != callbacks.rend(); ++it)
        {
            try
            {
                (*it)();
            }
            catch (...)
            {
            }
        }
    }
};

void at_exit(std::function<void()> cb)
{
    ExitGuard::instance().push(std::move(cb));
}