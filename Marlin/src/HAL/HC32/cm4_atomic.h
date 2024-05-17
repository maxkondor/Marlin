#ifdef __arm__ 

#ifndef _CORTEX_M4_ATOMIC_H_
#define _CORTEX_M4_ATOMIC_H_

static __inline__ uint32_t __iSeiRetVal(void) __attribute__((always_inline));
static __inline__ uint32_t __iSeiRetVal(void) \
{ __asm__ volatile ("CPSIE i\n\t""dmb\n\t""dsb\n\t""isb\n\t"); \
  __asm__ volatile ("" ::: "memory"); return 1; }   

static __inline__ uint32_t __iCliRetVal(void) __attribute__((always_inline));
static __inline__ uint32_t __iCliRetVal(void) \
{ __asm__ volatile ("CPSID i\n\t""dmb\n\t""dsb\n\t""isb\n\t"); \
  __asm__ volatile ("" ::: "memory"); return 1; }   

#endif
#endif
