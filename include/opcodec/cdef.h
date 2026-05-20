/* opcodec/cdef.h — AV1-style CDEF directional enhancement filter
 *
 * Copyright (c) 2026 Ophion Development Team. GPL v2.
 */
#ifndef OPCODEC_CDEF_H
#define OPCODEC_CDEF_H

#include <stdint.h>

/* Apply CDEF to an 8-bit luma frame in place.
 * strength: 0=off; 1-7 stronger. Filters each 8x8 block along its dominant
 * gradient direction (8 candidate angles) using damped neighbor differences. */
void opvis_cdef_apply(uint8_t *frame, int width, int height, uint8_t strength);

#endif /* OPCODEC_CDEF_H */
