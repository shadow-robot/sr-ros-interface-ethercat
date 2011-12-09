#include <time.h>
#include <stdio.h>

int main(int argc, char *argv[])
{
    struct timespec nano_time;
    clock_getres(CLOCK_REALTIME, &nano_time);
    printf("clock resolution = %ld secs and %ld nsecs\n",
           nano_time.tv_sec, nano_time.tv_nsec);
}
