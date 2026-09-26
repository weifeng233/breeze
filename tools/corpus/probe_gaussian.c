/* Two things about gaussian_blur.h that a byte corpus cannot see on its own.
 *
 * 1. Is the `< 3` floor in the auto kernel-size rule observable at all?
 *
 *    The rule is `size = (int)(sigma * 6 + 0.5)`, bumped to odd and floored at 3.
 *    The floor can only fire for `sigma < 0.25`, and the corpus case that was
 *    meant to pin it (`gaussian_blur_auto_sigma_0p2`) came out byte-identical to
 *    the identity: at sigma 0.2 the side taps are exp(-12.5) = 3.7e-6, which
 *    rounds to nothing in an 8-bit result. If that is true for *every* sigma that
 *    reaches the floor and for *every* pixel value, then no corpus case can pin
 *    the floor, and that is worth stating as a measured fact instead of leaving a
 *    case that looks like it checks something.
 *
 *    A constant neighbourhood maximises the difference a side tap can make, so
 *    checking all 256 values on a constant image bounds every real image.
 *
 * 2. What does the even-size 1-D filter actually do?
 *
 *    `BreezeGaussianBlur1D_Horizontal` loops `i` from `-size/2` to `+size/2`,
 *    reading `kernel[i + size/2]`. For an even size that is `size/2` taps on each
 *    side of a kernel that only has `size` entries - one read past the end. The
 *    C's own `BreezeGaussianBlur` never passes an even size (it bumps), so the
 *    overrun is only reachable through the public 1-D functions. This shows it by
 *    putting a sentinel where the extra read lands: change the sentinel, change
 *    the output. */
#include <stdio.h>
#include <string.h>

#include "../../include/breeze/image/gaussian_blur.h"

/* 1. the floor */
static int floor_is_observable(void) {
    static const float sigmas[] = {0.01f, 0.05f, 0.1f, 0.15f, 0.2f, 0.24f, 0.3f, 1.0f};
    int s, v;
    int differences = 0;

    for (s = 0; s < (int)(sizeof sigmas / sizeof sigmas[0]); s++) {
        int auto_size = (int)(sigmas[s] * 6.0f + 0.5f);
        if (auto_size % 2 == 0) auto_size++;
        if (auto_size < 3) auto_size = 3;

        for (v = 0; v < 256; v++) {
            unsigned char src[9];
            unsigned char auto_out[9];
            unsigned char identity_out[9];
            int i;
            memset(src, v, sizeof src);

            BreezeGaussianBlur(src, auto_out, 3, 3, sigmas[s], 0, 0);
            BreezeGaussianBlur(src, identity_out, 3, 3, sigmas[s], 1, 0);

            for (i = 0; i < 9; i++) {
                if (auto_out[i] != identity_out[i]) {
                    if (differences < 8) {
                        printf("  sigma %.2f pixel %3d: auto(%d) = %d, one-tap = %d\n",
                               (double)sigmas[s], v, auto_size, auto_out[i], identity_out[i]);
                    }
                    differences++;
                }
            }
        }
        printf("sigma %.2f -> kernel size %d, %s\n", (double)sigmas[s], auto_size,
               auto_size == 3 && sigmas[s] < 0.25f ? "floor branch" : "");
    }
    return differences;
}

/* 2. the even-size overrun */
static void even_size_overrun(void) {
    /* Five floats: four taps for the kernel, and a sentinel where the fifth read
     * lands because the loop runs i = -2..+2 with a kernel_size of 4. */
    float kernel_a[5] = {0.0f, 0.5f, 0.5f, 0.0f, 0.0f};
    float kernel_b[5] = {0.0f, 0.5f, 0.5f, 0.0f, 1.0f};
    const unsigned char src[4] = {10, 20, 30, 40};
    unsigned char out_a[4];
    unsigned char out_b[4];

    BreezeGaussianBlur1D_Horizontal(src, out_a, 4, 1, kernel_a, 4, 0);
    BreezeGaussianBlur1D_Horizontal(src, out_b, 4, 1, kernel_b, 4, 0);

    printf("kernel_size 4, sentinel 0.0 -> %d %d %d %d\n", out_a[0], out_a[1], out_a[2], out_a[3]);
    printf("kernel_size 4, sentinel 1.0 -> %d %d %d %d\n", out_b[0], out_b[1], out_b[2], out_b[3]);
    printf("the two runs differ: %s\n",
           memcmp(out_a, out_b, sizeof out_a) ? "yes, so the 5th float was read" : "no");
}

int main(void) {
    int differences;

    printf("--- auto kernel size floor (sigma < 0.25) ---\n");
    differences = floor_is_observable();
    printf("bytes differing between the auto size and the one-tap identity: %d\n\n", differences);

    printf("--- even kernel size in the 1-D filter ---\n");
    even_size_overrun();
    return 0;
}
