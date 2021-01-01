#ifndef LIFTOFF_C11_BINARY_LATCH_H
#define LIFTOFF_C11_BINARY_LATCH_H

#include <condition_variable>
#include <mutex>

/**
 * @brief Represents a single-use latch which can have
 * multiple waiting threads which are released by the
 * release() method. This is implemented for C++11.
 */
class c11_binary_latch {
private:
    std::mutex mutex;
    std::condition_variable cond;
    bool released{false};
public:
    /**
     * Blocks until a thread calls release(), unless the
     * release() method has already been called.
     */
    void wait();

    /**
     * Releases all threads blocked on the call to wait().
     */
    void release();
};

#endif // LIFTOFF_C11_BINARY_LATCH_H
