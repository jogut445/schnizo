// Copyright 2025 ETH Zurich and University of Bologna.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0
//
// Giulio Ferraro <gferraro@student.ethz.ch>

#include "snrt.h"

// Test that we can generate a load & store address stream.
// We copy the array x to the array z.

// 64-bit AXPY: y = a * x + y
void axpy_v(const int a, const int *x, const int *y, unsigned int avl) {
    unsigned int vl;

    // Stripmine and accumulate a partial vector
    do {
        // Set the vl
        asm volatile("vsetvli %0, %1, e32, m8, ta, ma" : "=r"(vl) : "r"(avl));

        // Load vectors
        asm volatile("vle32.v v0, (%0)" ::"r"(x));
        asm volatile("vle32.v v8, (%0)" ::"r"(y));

        asm volatile("vmacc.vx v8, %0, v0" ::"r"(a));

        // Store results
        asm volatile("vse32.v v8, (%0)" ::"r"(y));

        // Multiply-accumulate

        // Bump pointers
        x += vl;
        y += vl;
        avl -= vl;
    } while (avl > 0);
}

int main() {
    if (snrt_global_core_idx() == 0) {
        unsigned int avl = 200;

        int x[avl], y[avl], z[avl];
        int i;
        for (i = 0; i < avl; i++) {
            x[i] = i;
            y[i] = i;
        }

        axpy_v(2, x, y, avl);

        for (i = 0; i < avl; i++) {
            if (y[i] != 3 * i) {
                printf("Error at index %d: %d != %d\n", i, y[i], 3 * i);
            }
        }
    }

    snrt_cluster_hw_barrier();

    return 0;
}
