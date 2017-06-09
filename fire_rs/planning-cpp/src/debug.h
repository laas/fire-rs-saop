#ifndef PLANNING_CPP_DEBUG_H
#define PLANNING_CPP_DEBUG_H


#include <stdio.h>
#include <signal.h>
#include <stdio.h>
#include <signal.h>
#include <execinfo.h>
#include <wait.h>
#include <zconf.h>
#include <cstdlib>

#define ASSERT(test)                  \
if(!(test)) {                         \
  fprintf(stderr, "Failed assert\n"); \
  print_trace();                      \
}                                     \
assert(test);


void print_trace() {
    char pid_buf[30];
    sprintf(pid_buf, "%d", getpid());
    char name_buf[512];
    name_buf[readlink("/proc/self/exe", name_buf, 511)]=0;
    int child_pid = fork();
    if (!child_pid) {
        dup2(2,1); // redirect output to stderr
        fprintf(stdout,"stack trace for %s pid=%s\n",name_buf,pid_buf);
        execlp("gdb", "gdb", "--batch", "-n", "-ex", "thread", "-ex", "bt", name_buf, pid_buf, NULL);
        printf("Cannot print a stack without gdb\n");
        abort(); /* If gdb failed to start */
    } else {
        waitpid(child_pid,NULL,0);
    }
}



#endif //PLANNING_CPP_DEBUG_H
