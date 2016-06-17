#pragma once

#define ERR    1
#define INIT   2

int logging_log(int level, const char *fmt, ...);

void logging_start();

void logging_stop();

#ifdef ENABLE_LOGGING
#define LOG logging_log
#define LOG_START logging_start
#define LOG_STOP logging_stop
#else
#define LOG(...)
#define LOG_START()
#define LOG_STOP()
#endif