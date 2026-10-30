#ifndef TEST_ATOMIC_H
#define TEST_ATOMIC_H
// Tests invoke interrupt handlers explicitly and execute sequentially.
#define ATOMIC_RESTORESTATE 0
#define ATOMIC_BLOCK(mode) for (bool atomicOnce = true; atomicOnce; atomicOnce = false)
#endif
