/* Find strided images that two plausible stride defects cannot fake.
 *
 * There are two ways a strided Otsu can go wrong that "the padding is not read"
 * does not cover:
 *
 *   1. counting the padding in `total_pixels` (`stride * height` instead of
 *      `width * height`), and
 *   2. reading the pixels packed (stride 0) and then writing them strided, which
 *      is what `BreezeApplyOtsuThreshold` does if it does not forward its stride
 *      to the `BreezeOtsuThreshold` call inside it.
 *
 * Both stayed green against every corpus case, because the corpus images were
 * either packed (the two agree) or perfectly separable (two far-apart clusters,
 * where the argmax of wB*wF*(mB-mF)^2 does not move when the total changes, and
 * where a shifted read still has the same split point).
 *
 * This searches for one image that separates each. The copies below are the C
 * function verbatim except for the single line under test. */
#include <stdio.h>
#include <string.h>

#include "../../include/breeze/image/otsu_threshold.h"

static unsigned char buggy_total(const unsigned char* src, int width, int height, int stride_bytes) {
    int histogram[256] = {0};
    int x, y, i;
    int stride = stride_bytes > 0 ? stride_bytes : width;
    int total_pixels = stride * height;   /* <- the defect under test */
    float sum = 0, sumB = 0, wB = 0, wF = 0, mB, mF;
    float max_variance = 0;
    unsigned char threshold = 0;

    for (y = 0; y < height; y++)
        for (x = 0; x < width; x++) {
            int idx = y * stride + x;
            histogram[src[idx]]++;
            sum += src[idx];
        }

    for (i = 0; i < 256; i++) {
        wB += histogram[i];
        if (wB == 0) continue;
        wF = total_pixels - wB;
        if (wF == 0) break;
        sumB += i * histogram[i];
        mB = sumB / wB;
        mF = (sum - sumB) / wF;
        float variance = wB * wF * (mB - mF) * (mB - mF);
        if (variance > max_variance) { max_variance = variance; threshold = i; }
    }
    return threshold;
}

/* The same image, 4 wide and 2 high at stride 6, in both layouts. */
static void build(unsigned char* strided, const unsigned char* packed) {
    memset(strided, 0xAA, 12);
    memcpy(strided, packed, 4);
    memcpy(strided + 6, packed + 4, 4);
}

int main(void) {
    static const unsigned char levels[] = {10, 60, 120, 200, 250};
    unsigned int seed = 12345;
    unsigned char packed[8];
    unsigned char strided[12];
    int attempt;
    int found_total = 0, found_apply = 0;

    for (attempt = 0; attempt < 2000000; attempt++) {
        int i;
        unsigned char correct, by_total, by_packed;

        for (i = 0; i < 8; i++) {
            seed = seed * 1103515245u + 12345u;
            packed[i] = levels[(seed >> 16) % 5];
        }
        build(strided, packed);

        correct   = BreezeOtsuThreshold(strided, 4, 2, 6);
        by_total  = buggy_total(strided, 4, 2, 6);
        by_packed = BreezeOtsuThreshold(strided, 4, 2, 0);

        if (!found_total && correct != by_total) {
            found_total = 1;
            printf("total_pixels defect after %d attempts\n", attempt);
            printf("  rows: %d %d %d %d / %d %d %d %d\n",
                   packed[0], packed[1], packed[2], packed[3],
                   packed[4], packed[5], packed[6], packed[7]);
            printf("  correct total_pixels=8  -> %d\n", correct);
            printf("  buggy   total_pixels=12 -> %d\n", by_total);
        }
        if (!found_apply && correct != by_packed) {
            found_apply = 1;
            printf("packed-read defect after %d attempts\n", attempt);
            printf("  rows: %d %d %d %d / %d %d %d %d\n",
                   packed[0], packed[1], packed[2], packed[3],
                   packed[4], packed[5], packed[6], packed[7]);
            printf("  correct stride 6 -> %d\n", correct);
            printf("  packed   stride 0 -> %d\n", by_packed);
        }
        if (found_total && found_apply) {
            printf("image (4 wide, stride 6): %d %d %d %d %d %d / %d %d %d %d %d %d\n",
                   strided[0], strided[1], strided[2], strided[3], strided[4], strided[5],
                   strided[6], strided[7], strided[8], strided[9], strided[10], strided[11]);
            return 0;
        }
    }
    printf("not found (total=%d apply=%d) in %d attempts\n", found_total, found_apply, attempt);
    return 1;
}
