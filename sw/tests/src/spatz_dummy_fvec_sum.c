// Copyright 2025 ETH Zurich and University of Bologna.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0
//
// Giulio Ferraro <gferraro@student.ethz.ch>

#include "snrt.h"
// Test that we can generate a load & store address stream.
// We copy the array x to the array z.

#define SIZE 64

int main() {
    if (snrt_global_core_idx() == 0) {
        unsigned int avl = 64;

        alignas(64) const double a[SIZE] = {
            0.5,  1.5,  2.5,  3.5,  4.5,  5.5,  6.5,  7.5,  8.5,  9.5,  10.5,
            11.5, 12.5, 13.5, 14.5, 15.5, 16.5, 17.5, 18.5, 19.5, 20.5, 21.5,
            22.5, 23.5, 24.5, 25.5, 26.5, 27.5, 28.5, 29.5, 30.5, 31.5, 32.5,
            33.5, 34.5, 35.5, 36.5, 37.5, 38.5, 39.5, 40.5, 41.5, 42.5, 43.5,
            44.5, 45.5, 46.5, 47.5, 48.5, 49.5, 50.5, 51.5, 52.5, 53.5, 54.5,
            55.5, 56.5, 57.5, 58.5, 59.5, 60.5, 61.5, 62.5, 63.5};

        alignas(64) const double b[SIZE] = {
            -1.25,  0.75,   2.75,   4.75,   6.75,   8.75,   10.75,  12.75,
            14.75,  16.75,  18.75,  20.75,  22.75,  24.75,  26.75,  28.75,
            30.75,  32.75,  34.75,  36.75,  38.75,  40.75,  42.75,  44.75,
            46.75,  48.75,  50.75,  52.75,  54.75,  56.75,  58.75,  60.75,
            62.75,  64.75,  66.75,  68.75,  70.75,  72.75,  74.75,  76.75,
            78.75,  80.75,  82.75,  84.75,  86.75,  88.75,  90.75,  92.75,
            94.75,  96.75,  98.75,  100.75, 102.75, 104.75, 106.75, 108.75,
            110.75, 112.75, 114.75, 116.75, 118.75, 120.75, 122.75, 124.75};

        alignas(64) const double sum[SIZE] = {
            -0.75,  2.25,   5.25,   8.25,   11.25,  14.25,  17.25,  20.25,
            23.25,  26.25,  29.25,  32.25,  35.25,  38.25,  41.25,  44.25,
            47.25,  50.25,  53.25,  56.25,  59.25,  62.25,  65.25,  68.25,
            71.25,  74.25,  77.25,  80.25,  83.25,  86.25,  89.25,  92.25,
            95.25,  98.25,  101.25, 104.25, 107.25, 110.25, 113.25, 116.25,
            119.25, 122.25, 125.25, 128.25, 131.25, 134.25, 137.25, 140.25,
            143.25, 146.25, 149.25, 152.25, 155.25, 158.25, 161.25, 164.25,
            167.25, 170.25, 173.25, 176.25, 179.25, 182.25, 185.25, 188.25};

        unsigned int vl;

        double res[SIZE];

        asm volatile("vsetvli %0, %1, e64, m8, ta, ma" : "=r"(vl) : "r"(avl));
        asm volatile("vle64.v v0, (%0)" ::"r"(a));
        asm volatile("vle64.v v8, (%0)" ::"r"(b));
        asm volatile("vfadd.vv v0, v0, v8");
        asm volatile("vse64.v v0, (%0)" ::"r"(res));

        volatile int i;
        for (i = 0; i < SIZE; i++)
            ;

        int errors = 0;
        for (i = 0; i < 64; i++) errors += res[i] != sum[i];

        printf("Errors: %d\n", errors);

        for (i = 0; i < 100; i++)
            ;  //Wait to flush print buffer
    }

    snrt_cluster_hw_barrier();

    return 0;
}
