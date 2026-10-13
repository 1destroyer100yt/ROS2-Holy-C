/* Exhaustive check of src/Core/F32.HC against the C compiler's own
 * float conversions. Build with the HolyC library converted to C:
 *   python3 tools/hctest/hc2c.py c src/Load.HC -o /tmp/lib.c
 *   gcc -O2 -include /tmp/lib.c tools/hctest/f32_check.c -lm
 * Pass "quick" to check every 97th pattern instead of all 2^32. */
#include <math.h>
static U32 fbits(float f) { U32 u; memcpy(&u, &f, 4); return u; }
static float bitsf(U32 u) { float f; memcpy(&f, &u, 4); return f; }
static U64 dbits(double d) { U64 u; memcpy(&u, &d, 8); return u; }
static double bitsd(U64 u) { double d; memcpy(&d, &u, 8); return d; }

int main(int argc, char **argv)
{
  U64 step = (argc > 1 && !strcmp(argv[1], "quick")) ? 97 : 1, bad = 0, n = 0;
  for (U64 v = 0; v <= 0xFFFFFFFFull; v += step) {
    float f = bitsf((U32)v);
    double want = (double)f, got = RosF32BitsToF64((U32)v);
    n++;
    if (isnan(f)) {
      if (!isnan(got) || (RosF64ToF32Bits(got) & 0x7FC00000u) != 0x7FC00000u) bad++;
      continue;
    }
    if (dbits(want) != dbits(got)) { if (bad++ < 5) printf("F32->F64 %08llx\n", (unsigned long long)v); }
    if (RosF64ToF32Bits(want) != (U32)v) { if (bad++ < 5) printf("roundtrip %08llx\n", (unsigned long long)v); }
  }
  /* random doubles, incl. ties and out-of-range values */
  U64 x = 88172645463325252ull;
  for (int i = 0; i < 20000000; i++) {
    x ^= x << 13; x ^= x >> 7; x ^= x << 17;
    double d = bitsd(x);
    if (i & 1) d = (double)bitsf((U32)x) + ldexp((double)bitsf((U32)x), -24); /* exact ties */
    if (isnan(d)) continue;
    n++;
    if (RosF64ToF32Bits(d) != fbits((float)d)) {
      if (bad++ < 10) printf("F64->F32 %016llx got %08x want %08x\n", (unsigned long long)dbits(d),
                             RosF64ToF32Bits(d), fbits((float)d));
    }
  }
  printf("f32_check: %llu values, %llu mismatches\n", (unsigned long long)n, (unsigned long long)bad);
  return bad != 0;
}
