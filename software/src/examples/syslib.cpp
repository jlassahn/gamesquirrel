
#include <errno.h>
#undef errno
extern int errno;

extern "C" {

void *__dso_handle = 0;

static const char *env[1] = { 0 };
const char **environ = env;

extern int main(int argc, const char *argv[]);
extern void __libc_init_array(void);

void _start(void)
{
    __libc_init_array();
    main(0, nullptr);
}

// called by __libc_init_array
void _init(void)
{
}

void _exit(int n)
{
    while (1)
    {}
}

int _close(int fd)
{
    return -1;
}

int _execve(char *name, const char **argv, const char **env)
{
    errno = ENOMEM;
    return -1;
}

int _fork(void)
{
    return -1;
}

int _fstat(int file, struct stat *st)
{
    return -1;
}

int _getpid(void)
{
    return 1;
}

int _isatty(int fd)
{
    return 1;
}

int _kill(int pid, int sig)
{
    return -1;
}

int _link(const char *oldname, const char *newname)
{
    return -1;
}

int _lseek(int file, int ptr, int dir)
{
    return 0;
}

int _open(const char *name, int flags, int mode)
{
    return -1;
}

int _read(int fd, char *ptr, int len)
{
    return 0;
}

void * _sbrk(int incr)
{
    return 0;
}

int _stat(const char *name, struct stat *st)
{
    return 0;
}

int _times(struct tms *buf)
{
    return -1;
}

int _unlink(const char *name)
{
    return -1;
}

int _wait(int *status)
{
    return -1;
}

int _write(int fd, const char *ptr, int len)
{
    return 0;
}



}

