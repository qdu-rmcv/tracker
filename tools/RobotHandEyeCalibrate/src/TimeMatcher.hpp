#ifndef INCLUDE_TIME_MATCHER_HPP
#define INCLUDE_TIME_MATCHER_HPP

#include <climits>
#include <cstdint> // 引入 uint64_t
#include <cmath>

template<typename T, typename U>
class TimeMatcher
{
public:
    enum class Result{ FrontTout,BackTout,MatchOK};
    // 使用 uint64_t
    TimeMatcher<T, U>(double TDelay_ms, double UDelay_ms) : TDelay_ns(static_cast<uint64_t>(TDelay_ms * 1e6)),
                                                           UDelay_ns(static_cast<uint64_t>(UDelay_ms * 1e6)) {}

    void setFrontwave(double up_ms,double down_ms) 
    { 
        this->Twave_up_ns = static_cast<uint64_t>(up_ms * 1e6);
        this->Twave_down_ns = static_cast<uint64_t>(down_ms * 1e6);
    }

    void setBackwave(double up_ms,double down_ms)
    { 
        this->Uwave_up_ns = static_cast<uint64_t>(up_ms * 1e6);
        this->Uwave_down_ns = static_cast<uint64_t>(down_ms * 1e6);
    }

    Result Match(const T& a, const U& b)
    {
        if(Diff_us>=0)
        {
            uint64_t delay = static_cast<uint64_t>( (a.time - b.time).count() );
            if(delay > (Diff_us + Twave_up_ns + Uwave_down_ns)) return Result::FrontTout;
            if(delay < ( Diff_us - Twave_down_ns - Uwave_up_ns)) return Result::BackTout;
            return Result::MatchOK;
        }

        Diff_us = -Diff_us;
        uint64_t delay = static_cast<uint64_t>( (b.time - a.time).count() );
        if(delay > (Diff_us + Uwave_up_ns + Twave_down_ns)) return Result::BackTout;
        if(delay < ( Diff_us - Uwave_down_ns - Twave_up_ns)) return Result::FrontTout;
        return Result::MatchOK;                  
    }

private:

    uint64_t Twave_up_ns = 0, Twave_down_ns = 0,
             Uwave_up_ns = 0, Uwave_down_ns = 0;

    uint64_t TDelay_ns, UDelay_ns, Diff_us;
};

#endif