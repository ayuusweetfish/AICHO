// gcc -x c breath_rate.h -DBREATH_RATE_TEST

#ifndef BREATH_RATE_TEST
#pragma once
#endif

#include <stdbool.h>
#include <stdint.h>

typedef struct {
  uint32_t rate;
  uint32_t cur;
} breath_rate_estimator;

// `value`:
//  true: exhale
//  false: inhale
// Returns:
//  +2: new inhale
//  +1: held inhale
//  -1: held exhale
//  -2: new exhale
static inline int breath_rate_feed(breath_rate_estimator *e, bool value)
{
  e->cur = (e->cur + 1) % e->rate;
  if (e->cur == 0) return +2;
  if (e->cur < e->rate / 2) return +1;
  if (e->cur == e->rate / 2) return -2;
  return -1;
}

#ifdef BREATH_RATE_TEST
#include <stdio.h>
int main()
{
  breath_rate_estimator e = {.rate = 20, .cur = 19};
  static const bool a[] = {
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0,
  };
  for (int i = 0; i < (sizeof a) / (sizeof a[0]); i++) {
    int r = breath_rate_feed(&e, a[i]);
    printf("[%3d] in %d returns %2d\n", i, a[i], r);
  }

  return 0;
}
#endif
