#pragma once

// Most people call this "FORCE_INLINE", but those people aren't very fun
#define SUPER_INLINE inline __attribute__((always_inline))

#define LIKELY(x) __builtin_expect(static_cast<bool>(x), 1)
#define UNLIKELY(x) __builtin_expect(static_cast<bool>(x), 0)
