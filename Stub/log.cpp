#include "stdafx.h"
#include <stdio.h>
#include <stdarg.h>
#include "log.h"
#include "utils.h"

#ifdef _WIN32
#define LOG_FILE "C:\\temp\\iso15765.log"
#endif //_WIN32

#ifdef __linux__
#define LOG_FILE "/tmp/iso15765.log"
#endif //__linux__

static FILE *logging_file = NULL;

int logging_log(int level, const char *fmt, ...) {
    UNUSED(level);
    if (logging_file == NULL) {
        return 0;
    }
    va_list myargs;
    va_start(myargs, fmt);
    int ret = vfprintf(logging_file, fmt, myargs);
    fprintf(logging_file, "\n");
    fflush(logging_file);
    va_end(myargs);
    return ret;
}

void logging_start() {
    if(logging_file == NULL) {
      logging_file = fopen(LOG_FILE, "a+");
    }
}


void logging_stop() {
    if (logging_file != NULL) {
        fclose(logging_file);
    }
}
