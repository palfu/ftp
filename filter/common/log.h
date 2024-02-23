#pragma once

#include <string>
#include <stdio.h>

#define LOG_COLOR_NONE_ "\033[0m\n"
#define LOG_COLOR_RED_ "\033[0;32;31m"
#define LOG_COLOR_LIGHT_RED_ "\033[1;31m"
#define LOG_COLOR_GREEN_ "\033[0;32;32m"
#define LOG_COLOR_LIGHT_GREEN_ "\033[1;32m"
#define LOG_COLOR_BLUE_ "\033[0;32;34m"
#define LOG_COLOR_LIGHT_BLUE_ "\033[1;34m"
#define LOG_COLOR_DARY_GRAY_ "\033[1;30m"
#define LOG_COLOR_CYAN_ "\033[0;36m"
#define LOG_COLOR_LIGHT_CYAN_ "\033[1;36;43m"
#define LOG_COLOR_PURPLE_ "\033[0;35m"
#define LOG_COLOR_LIGHT_PURPLE_ "\033[1;35m"
#define LOG_COLOR_BROWN_ "\033[0;33m"
#define LOG_COLOR_YELLOW_ "\033[1;33m"
#define LOG_COLOR_LIGHT_GRAY_ "\033[0;37m"

#define FLOG_INFO(format, ...)                      \
    printf(LOG_COLOR_GREEN_ format, ##__VA_ARGS__); \
    printf(LOG_COLOR_NONE_)
