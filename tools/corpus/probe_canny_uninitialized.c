/* Can the Canny pipeline's output depend on uninitialized memory?
 *
 * It looked like it might: `canny_edges_5x5` in the corpus changed when six
 * unrelated cases were added to the generator, which cannot happen to a function
 * of its inputs. The reading was this:
 *
 *   * `BreezeCannyGradient` writes only the interior - `x` and `y` run from 1 to
 *     width-2 and height-2 - so the border of `magnitude` is whatever `malloc`
 *     handed back.
 *   * `BreezeCannyNonMaxSuppression` reads `magnitude` for interior pixels, but
 *     with a diagonal direction (1 or 3) it looks at `(y-1, x+1)`, `(y+1, x-1)`
 *     and friends. For an interior pixel one step in from the border, one of those
 *     neighbours *is* a border pixel.
 *
 * So the suppression stage compares a real magnitude against heap garbage, and
 * whether a pixel survives depends on it. This program checks both halves: first
 * that suppression's answer changes when the border holds different values, then
 * that the whole pipeline answers differently with the heap in different states.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "../../include/breeze/image/canny_edge.h"

static const unsigned char square[25] = {0, 0, 0,   0,   0,
                                         0, 0, 0,   0,   0,
                                         0, 0, 255, 255, 0,
                                         0, 0, 255, 255, 0,
                                         0, 0, 0,   0,   0};

/* 1. suppression, with the border of `magnitude` set to two different values */
static void suppression_depends_on_the_border(void) {
    float mag_a[25];
    unsigned char dir_a[25];
    float nms_a[25];
    float mag_b[25];
    unsigned char dir_b[25];
    float nms_b[25];
    int i, differences = 0;

    for (i = 0; i < 25; i++) { mag_a[i] = 0.0f; dir_a[i] = 0; }
    BreezeCannyGradient(square, mag_a, dir_a, 5, 5, 0);

    /* Same gradients, but the border holds a value above every real magnitude -
     * which is what garbage looks like when it is not small. 100 is not enough:
     * the interior magnitudes here are 360, 806 and 1081, so a border of 100
     * still loses every comparison and nothing moves. */
    memcpy(mag_b, mag_a, sizeof mag_b);
    memcpy(dir_b, dir_a, sizeof dir_b);
    for (i = 0; i < 25; i++) {
        int on_border = (i < 5) || (i >= 20) || (i % 5 == 0) || (i % 5 == 4);
        if (on_border) mag_b[i] = 1.0e16f;
    }

    BreezeCannyNonMaxSuppression(mag_a, dir_a, nms_a, 5, 5);
    BreezeCannyNonMaxSuppression(mag_b, dir_b, nms_b, 5, 5);

    for (i = 0; i < 25; i++) {
        if (nms_a[i] != nms_b[i]) {
            printf("  index %2d (x=%d, y=%d): border 0 -> %.6g, border 1e16 -> %.6g\n",
                   i, i % 5, i / 5, (double)nms_a[i], (double)nms_b[i]);
            differences++;
        }
    }
    printf("pixels whose suppression answer depends on the border: %d\n\n", differences);
}

/* 2. the whole pipeline, with the heap perturbed between runs */
static int pipeline_once(unsigned char* dst, size_t perturb) {
    /* Move the heap around before the pipeline allocates its four buffers. */
    void* noise = malloc(perturb);
    if (noise) {
        memset(noise, 0x5A, perturb);
        free(noise);
    }
    memset(dst, 0, 25);
    BreezeCannyEdgeDetection(square, dst, 5, 5, 1.0f, 20.0f, 60.0f, 0);
    return 0;
}

static void pipeline_depends_on_the_heap(void) {
    unsigned char a[25];
    unsigned char b[25];
    unsigned char c[25];
    int i, differing = 0;

    pipeline_once(a, 0);
    pipeline_once(b, 4096);
    pipeline_once(c, 65536);

    printf("run A (no heap noise): ");
    for (i = 0; i < 25; i++) printf("%d ", a[i]);
    printf("\nrun B (4096 bytes):    ");
    for (i = 0; i < 25; i++) printf("%d ", b[i]);
    printf("\nrun C (65536 bytes):   ");
    for (i = 0; i < 25; i++) printf("%d ", c[i]);
    printf("\n");

    for (i = 0; i < 25; i++) {
        if (a[i] != b[i] || a[i] != c[i]) differing++;
    }
    printf("pixels where the three runs disagree: %d\n", differing);
}

int main(void) {
    printf("--- suppression vs the border of `magnitude` ---\n");
    suppression_depends_on_the_border();

    printf("--- the pipeline vs the state of the heap ---\n");
    pipeline_depends_on_the_heap();
    return 0;
}
