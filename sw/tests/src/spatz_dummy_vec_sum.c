// Copyright 2025 ETH Zurich and University of Bologna.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0
//
// Giulio Ferraro <gferraro@student.ethz.ch>

#include "snrt.h"

// Test that we can generate a load & store address stream.
// We copy the array x to the array z.

int main() {
    if (snrt_global_core_idx() == 0) {
        unsigned int avl = 64;

        int x[avl], y[avl], z[avl];
        int i;
        for (i = 0; i < avl; i++) {
            x[i] = i;
            y[i] = i;
        }

        unsigned int vl;

        asm volatile("vsetvli %0, %1, e64, m8, ta, ma" : "=r"(vl) : "r"(avl));
        printf("Vector length: %d\n", vl);
        asm volatile("vle64.v v0, (%0)" ::"r"(x));
        asm volatile("vle64.v v8, (%0)" ::"r"(y));
        asm volatile("vadd.vv v0, v0, v8");
        asm volatile("vse64.v v0, (%0)" ::"r"(y));
        printf("Stored vectors\n");

        for (i = 0; i < 64; i++) {
            if (y[i] != 2 * i) {
                printf("Error at index %d: %f != %f\n", i, y[i], 2 * i);
            }
        }
    }

    snrt_cluster_hw_barrier();

    return 0;
}
