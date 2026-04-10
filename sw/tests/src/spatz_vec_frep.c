// Copyright 2025 ETH Zurich and University of Bologna.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0
//
// Giulio Ferraro <gferraro@student.ethz.ch>

#include "snrt.h"

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
        unsigned int iter = 10;
        unsigned int avl = 128 * iter;

        int x[avl], y[avl], z[avl], vl;
        // int i;
        // for(i=0; i<avl; i++) {
        //   x[i] = i;
        //   y[i] = i;
        // }

        asm volatile("vsetvli  %[rvl],  %[rdvl], e32, m8, ta, ma       \n"
                     : [ rvl ] "+r"(vl)
                     : [ rdvl ] "r"(128));

        // Use temporaries so we can mark them read-write in inline asm
        int *xa = x;
        int *ya = y;
        const int inc = 128 * 4;  // bytes per vector (VL=128, e32)

        // asm volatile(
        //   "frep.o   %[n_frep], 4, 0, 0      \n"
        //   "vle32.v  v0,    (%[xa])          \n"
        //   "vse32.v  v0,    (%[ya])          \n"
        //   "add      %[xa], %[xa], %[inc]    \n"
        //   "add      %[ya], %[ya], %[inc]    \n"
        //   : [xa] "+r"(xa), [ya] "+r"(ya)
        //   : [n_frep] "r"(iter), [inc] "r"(inc)
        //   : "memory"
        // );

        asm volatile(

            "frep.o   %[n_frep], 6, 0, 0            \n"
            "vle32.v  v0,    (%[xa])                 \n"
            "vle32.v  v8,    (%[ya])                 \n"
            "vmacc.vx v8,    %[a],    v0             \n"
            "vse32.v  v8,    (%[ya])                 \n"
            "add     %[xa], %[xa],   %[inc]   \n"
            "add     %[ya], %[ya],   %[inc]   \n"
            : [ rvl ] "+r"(vl), [ xa ] "+r"(xa), [ ya ] "+r"(ya)
            : [ n_frep ] "r"(iter), [ inc ] "r"(inc), [ a ] "r"(2)
            : "memory");

        // for(i=0; i<avl; i++) {
        //   if (y[i] != 3*i) {
        //     printf("Error at index %d: %d != %d\n", i, y[i], i);
        //   }
        // }
    }

    snrt_cluster_hw_barrier();

    return 0;
}
