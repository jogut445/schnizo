// Copyright 2025 ETH Zurich and University of Bologna.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0
//
// Giulio Ferraro <gferraro@student.ethz.ch>

#include "snrt.h"

// Test that we can generate a load & store address stream.
// We copy the array x to the array z.

int main() {
    // if (!snrt_is_compute_core()) {
    //   return 0;
    // }

    if (snrt_global_core_idx() == 0) {
        unsigned int avl = 100;
        unsigned int vl;
        unsigned int result = 0;

        asm volatile("vsetvli %0, %1, e64, m8, ta, ma" : "=r"(vl) : "r"(avl));

        printf("Vector length: %d\n", vl);
    }

    snrt_cluster_hw_barrier();

    return 0;
}
