/* What does CLAHE do when there is only one tile along an axis?
 *
 * The interpolation at the end computes `ty_i = (int)(y / tile_height)` and then
 * pulls it back with
 *
 *     if (ty_i >= tile_count_y - 1) { ty_i = tile_count_y - 2; ty_alpha = 1.0f; }
 *
 * which is the ordinary way to keep `ty_i + 1` in range. With `tile_count_y == 1`
 * it writes -1 instead, and the next line reads `luts[ty_i][tx_i][value]` and
 * `luts[ty_i + 1][...]` - so it walks off the front of the table it allocated.
 * The same holds along x.
 *
 * `tile_count_y` is 1 whenever `height <= tile_size`, and the function clamps
 * `tile_size` down to `height` itself, so this is not a caller error: passing a
 * tile size larger than the image, or equal to it, is enough.
 *
 * This runs it and reports what happened. */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "../../include/breeze/image/histogram.h"

static void try_clahe(const char* label, int width, int height, int tile_size) {
    unsigned char* src = (unsigned char*)malloc((size_t)width * height);
    unsigned char* dst = (unsigned char*)malloc((size_t)width * height);
    int i;

    for (i = 0; i < width * height; i++) src[i] = (unsigned char)((i * 37) % 256);
    memset(dst, 0xEE, (size_t)width * height);

    printf("%s: %dx%d, tile_size %d ... ", label, width, height, tile_size);
    fflush(stdout);
    BreezeHistogramEqualizationCLAHE(src, dst, width, height, tile_size, 0.0f, 0);
    printf("returned. dst =");
    for (i = 0; i < width * height && i < 8; i++) printf(" %d", dst[i]);
    printf("\n");
    fflush(stdout);

    free(src);
    free(dst);
}

int main(void) {
    /* Two tiles along x, one along y: only the y axis is out of range. */
    try_clahe("one row of tiles   ", 6, 4, 6);

    /* One tile in both directions. */
    try_clahe("single tile        ", 4, 4, 4);

    /* The ordinary case, for contrast. */
    try_clahe("two by two tiles   ", 6, 6, 3);

    printf("all three returned\n");
    return 0;
}
