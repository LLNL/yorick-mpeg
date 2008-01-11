/* if the code is really written in portable ANSI C,
 * this file is unnecessary
 */

/* include  inttypes.h if not present, should get rid of this */
#undef EMULATE_INTTYPES
#undef EMULATE_FAST_INT

/* used in common.h to define unaligned32, almost certainly should eliminate */
#define ARCH_X86 1

/* used in mem.c, almost certainly should eliminate */
#undef HAVE_OSX
#define HAVE_MALLOC_H 1
#define HAVE_MEMALIGN 1
