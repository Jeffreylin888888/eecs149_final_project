#include <stddef.h>
/* Freestanding compiler helper; volatile prevents lowering into a recursive call. */
void *memset(void *dst,int value,size_t n) {
    volatile unsigned char *p=dst;
    for(size_t i=0;i<n;i++) p[i]=(unsigned char)value;
    return dst;
}
