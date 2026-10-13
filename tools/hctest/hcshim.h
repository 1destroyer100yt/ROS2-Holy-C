/* hcshim.h - stand-ins for the TempleOS built-ins our HolyC code uses,
 * so the code can be compiled and unit-tested on a normal Linux box.
 * This is test scaffolding only. It never ships to TempleOS.
 */
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <math.h>
#include <time.h>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <termios.h>
#include <sys/stat.h>
#include <pthread.h>
#include <sched.h>
#include <sys/mman.h>
#include <sys/io.h>
#include <sys/mount.h>
#include <sys/reboot.h>

typedef void U0;
typedef int8_t I8;
typedef uint8_t U8;
typedef int16_t I16;
typedef uint16_t U16;
typedef int32_t I32;
typedef uint32_t U32;
typedef int64_t I64;
typedef uint64_t U64;
typedef double F64;
typedef int8_t Bool;

#define TRUE 1
#define FALSE 0
#define ON 1
#define OFF 0
#define pi 3.14159265358979323846

static U8 *MAlloc(I64 n) { return malloc(n ? n : 1); }
static U8 *CAlloc(I64 n) { return calloc(1, n ? n : 1); }
static U0 Free(void *p) { free(p); }
/* n == 0 with a NULL pointer is fine in HolyC, undefined in C: skip it */
static U8 *MemCpy(void *d, const void *s, I64 n) { return n ? memmove(d, s, n) : d; }
static U8 *MemSet(void *d, I64 v, I64 n) { return n ? memset(d, (int)v, n) : d; }
static I64 MemCmp(const void *a, const void *b, I64 n) { return n ? memcmp(a, b, n) : 0; }
static I64 StrLen(const U8 *s) { return s ? (I64)strlen((const char *)s) : 0; }
static I64 StrCmp(const U8 *a, const U8 *b) { return strcmp((const char *)a, (const char *)b); }
static I64 StrNCmp(const U8 *a, const U8 *b, I64 n) { return strncmp((const char *)a, (const char *)b, n); }
static U8 *StrCpy(U8 *d, const U8 *s) { return (U8 *)strcpy((char *)d, (const char *)s); }
static U8 *StrNew(const U8 *s)
{
  if (!s) s = (const U8 *)"";
  return (U8 *)strdup((const char *)s);
}
static I64 ToI64(F64 d) { return (I64)d; }
/* TempleOS RandU64: here xorshift64*, seeded from the clock and pid */
static U64 RandU64(void)
{
  static U64 s;
  if (!s) {
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    s = ((U64)ts.tv_nsec << 20) ^ (U64)ts.tv_sec ^ ((U64)getpid() << 40) ^ 0x9E3779B97F4A7C15ULL;
  }
  s ^= s >> 12; s ^= s << 25; s ^= s >> 27;
  return s * 2685821657736338717ULL;
}
static F64 ToF64(I64 i) { return (F64)i; }
static F64 Abs(F64 d) { return fabs(d); }
static I64 AbsI64(I64 i) { return i < 0 ? -i : i; }
static F64 Sqrt(F64 d) { return sqrt(d); }
static F64 Floor(F64 d) { return floor(d); }
static F64 Ceil(F64 d) { return ceil(d); }
static F64 Pow(F64 a, F64 b) { return pow(a, b); }
static F64 Sin(F64 d) { return sin(d); }
static F64 Cos(F64 d) { return cos(d); }
static F64 Arg(F64 x, F64 y) { return atan2(y, x); }
static U0 Yield(void) { sched_yield(); }
/* x86 port I/O (the e1000 test runs as root in a Linux guest with iopl(3)) */
static U32 InU32(I64 port) { U32 v; __asm__ volatile("inl %w1,%0" : "=a"(v) : "Nd"((unsigned short)port)); return v; }
static U8 InU8(I64 port) { U8 v; __asm__ volatile("inb %w1,%0" : "=a"(v) : "Nd"((unsigned short)port)); return v; }
static U0 OutU8(I64 port, I64 v) { __asm__ volatile("outb %0,%w1" : : "a"((U8)v), "Nd"((unsigned short)port)); }
static U0 OutU32(I64 port, I64 v) { __asm__ volatile("outl %0,%w1" : : "a"((U32)v), "Nd"((unsigned short)port)); }
/* TempleOS tasks on other cores -> detached pthreads */
typedef struct CTask CTask;
#define mp_cnt ((I64)sysconf(_SC_NPROCESSORS_ONLN))
struct HcSpawnArg { void (*fp)(U8 *); U8 *data; };
static void *HcSpawnRun(void *p)
{
  struct HcSpawnArg a = *(struct HcSpawnArg *)p;
  free(p);
  a.fp(a.data);
  return NULL;
}
static CTask *Spawn(void (*fp)(U8 *), U8 *data, const U8 *name, I64 cpu)
{
  pthread_t t;
  struct HcSpawnArg *a = malloc(sizeof *a);
  a->fp = fp; a->data = data;
  if (pthread_create(&t, NULL, HcSpawnRun, a)) { free(a); return NULL; }
  pthread_detach(t);
  return (CTask *)a;  /* only compared with NULL */
}
/* TempleOS atomic bit ops: return the old bit */
static Bool LBts(I64 *p, I64 bit) { return (__atomic_fetch_or(p, 1LL << bit, __ATOMIC_ACQUIRE) >> bit) & 1; }
static Bool LBtr(I64 *p, I64 bit) { return (__atomic_fetch_and(p, ~(1LL << bit), __ATOMIC_RELEASE) >> bit) & 1; }
static U0 Sleep(I64 ms)
{
  struct timespec ts = {ms / 1000, (ms % 1000) * 1000000};
  nanosleep(&ts, NULL);
}
static I64 HostNowNs(void)
{
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return (I64)ts.tv_sec * 1000000000LL + ts.tv_nsec;
}

/* HolyC-style formatting: %d/%X/%c take I64, %f takes F64, %s takes U8*.
 * (DolDoc $...$ commands are handled when printing, see HcPutDolDoc) */
static U8 *HcVFormat(const U8 *fmt, va_list ap)
{
  size_t cap = 256, len = 0;
  char *out = malloc(cap), spec[64], tmp[512];
  const char *f = (const char *)fmt;
  while (*f) {
    int n = 0;
    tmp[0] = 0;
    if (*f == '%' && f[1] == '%') {
      strcpy(tmp, "%"); f += 2; n = 1;
    } else if (*f == '%') {
      const char *s = f++;
      while (*f && strchr("-+ #0123456789.*", *f)) f++;
      size_t sl = f - s;
      if (sl > 40) sl = 40;
      memcpy(spec, s, sl);
      switch (*f) {
        case 'd': case 'i': case 'u':
          strcpy(spec + sl, "lld"); n = snprintf(tmp, sizeof tmp, spec, (long long)va_arg(ap, I64)); break;
        case 'X': case 'x':
          strcpy(spec + sl, "llX"); n = snprintf(tmp, sizeof tmp, spec, (long long)va_arg(ap, I64)); break;
        case 'c':
          tmp[0] = (char)va_arg(ap, I64); tmp[1] = 0; n = 1; break;
        case 'f': case 'e': case 'g': case 'n':
          spec[sl] = *f == 'n' ? 'g' : *f; spec[sl + 1] = 0;
          n = snprintf(tmp, sizeof tmp, spec, va_arg(ap, F64)); break;
        case 's': {
          const char *sv = va_arg(ap, const char *);
          if (!sv) sv = "(null)";
          if (sl == 1) {  /* plain %s: any length (tmp would cut it off) */
            size_t k = strlen(sv);
            if (len + k + 1 > cap) { cap = (len + k + 1) * 2; out = realloc(out, cap); }
            memcpy(out + len, sv, k);
            len += k;
            f++;
            continue;
          }
          strcpy(spec + sl, "s");
          n = snprintf(tmp, sizeof tmp, spec, sv); break;
        }
        default:
          n = 0;
      }
      if (*f) f++;
    } else {
      tmp[0] = *f++; tmp[1] = 0; n = 1;
    }
    n = (int)strlen(tmp);
    if (len + n + 1 > cap) { cap = (len + n + 1) * 2; out = realloc(out, cap); }
    memcpy(out + len, tmp, n);
    len += n;
  }
  out[len] = 0;
  return (U8 *)out;
}
static U8 *MStrPrint(const U8 *fmt, ...)
{
  va_list ap; va_start(ap, fmt);
  U8 *s = HcVFormat(fmt, ap);
  va_end(ap);
  return s;
}
/* What the TempleOS document does with printed text: $$ is a $,
 * other $...$ are DolDoc commands (colors etc.), dropped here. */
static U0 HcPutDolDoc(const U8 *s)
{
  const char *p = (const char *)s;
  while (*p) {
    if (*p == '$') {
      if (p[1] == '$') { fputc('$', stdout); p += 2; continue; }
      const char *e = strchr(p + 1, '$');
      p = e ? e + 1 : p + strlen(p);
      continue;
    }
    fputc(*p++, stdout);
  }
}
static U0 Print(const U8 *fmt, ...)
{
  va_list ap; va_start(ap, fmt);
  U8 *s = HcVFormat(fmt, ap);
  va_end(ap);
  HcPutDolDoc(s);
  free(s);
}
