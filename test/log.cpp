#include "stdafx.h"
#include <stdio.h>
#include <stdarg.h>
#include "log.h"

static FILE * logging_file = NULL;

int logging_log(int level, const char *fmt, ...) {
	(void)(level);
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
	logging_file = fopen("C:\\temp\\test.log", "a+");
}


void logging_stop() {
	if(logging_file != NULL) {
		fclose(logging_file);
	}
}