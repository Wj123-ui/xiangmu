/**
 ******************************************************************************
 * @file      syscalls.c
 * @brief     Minimal syscalls for nano.specs newlib.
 *            Full libc is discarded via linker script (/DISCARD/),
 *            so only stubs needed by nano are retained.
 ******************************************************************************
 */

#include <sys/stat.h>
#include <errno.h>

/* --- Required by nano.specs --- */

int _write(int file, char *ptr, int len)
{
  (void)file; (void)ptr;
  return len;
}

int _read(int file, char *ptr, int len)
{
  (void)file; (void)ptr;
  return 0;
}

int _close(int file)   { (void)file; return -1; }
int _fstat(int file, struct stat *st) { (void)file; st->st_mode = S_IFCHR; return 0; }
int _isatty(int file)  { (void)file; return 1; }
int _lseek(int file, int ptr, int dir) { (void)file; (void)ptr; (void)dir; return 0; }

/* --- Defensive stubs (libc.a discarded, but kept for safety) --- */

void _exit(int status) { (void)status; while(1) {} }
int  _getpid(void)     { return 1; }
int  _kill(int pid, int sig) { (void)pid; (void)sig; errno = EINVAL; return -1; }
