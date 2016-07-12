//#include <string.h>

int strcmp(const char* a, const char* b) {
    while (1) {
        if (*a == 0 && *b == 0) {
            return 0;
        }

        if (*a < *b) {
            return -1;
        }

        if (*a > *b) {
            return 1;
        }

        ++a;
        ++b;
    }
}
