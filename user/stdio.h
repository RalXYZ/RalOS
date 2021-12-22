#pragma once

#include "stddef.h"

#ifdef DEBUG_LOG
#define Log(format, ...) \
    printf("[%s:%d %s] " format "\n", __FILE__, __LINE__, __func__, ## __VA_ARGS__);
#else
#define Log(format, ...);
#endif

#define GREEN "\033[0;32m"
#define YELLOW "\033[0;33m"
#define BLUE  "\033[0;34m"
#define RED   "\033[0;31m"
#define PURPLE "\033[0;35m"
#define NC "\033[0m"

#define U_MODE_STRING "[" PURPLE "U-MODE" NC "] "

int printf(const char *, ...);
