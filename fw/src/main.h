#pragma once

#ifndef BOARD_REV
  // Rev. 1a - 1
  // Rev. 2  - 2
  #define BOARD_REV 2
#endif

#if BOARD_REV == 1
  #define act_1 22
  #define act_2 23
#elif BOARD_REV == 2
  #define act_1 25
  #define act_2 26
#endif

int my_printf(const char *restrict fmt, ...);
