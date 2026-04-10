// Copyright 2025 ETH Zurich and University of Bologna.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0
//
// Giulio Ferraro <gferraro@student.ethz.ch>

#include "snrt.h"

#define PROBLEM_SIZE 64 * 100

int main() {
    if (snrt_global_core_idx() == 0) {
        double x[PROBLEM_SIZE], y[PROBLEM_SIZE], z[PROBLEM_SIZE];
        int start, end;

        volatile unsigned int vl;

        unsigned volatile int max_vl;
        asm volatile("vsetvli  %[rvl],  %[rdvl], e64, m1, ta, ma       \n"
                     : [rvl] "+r"(max_vl)
                     : [rdvl] "r"(-1));

        double* x_addr = x;
        double* y_addr = y;
        double* z_addr = z;
        double a = 10.0;
        int increment = sizeof(double) * max_vl;

        unsigned int n_vec_whole_iter = PROBLEM_SIZE / max_vl;
        unsigned int n_remaining_elems = PROBLEM_SIZE % max_vl;

        start = snrt_mcycle();

        asm volatile(
            // Code
            "frep.o  %[n_frep], 7, 0, 0\n"
            "vle64.v  v0,    (%[xa])          \n"
            "vle64.v  v8,    (%[ya])          \n"
            "add     %[xa], %[xa],   %[inc]   \n"  // move adds before fmadd to hide it beneath the fld
            "add     %[ya], %[ya],   %[inc]   \n"  // latency. This reduces the LCP overhead.
            "vfmacc.vf v8,    %[a],    v0     \n"
            "vse64.v  v8,    (%[za])          \n"
            "add     %[za], %[za],   %[inc]   \n"
            // Outputs
            : [xa] "+r"(x_addr), [ya] "+r"(y_addr), [za] "+r"(z_addr)
            // Inputs
            :
            [n_frep] "r"(n_vec_whole_iter - 1), [a] "f"(a), [inc] "r"(increment)
            // Clobbers
            : "memory");

        if (n_remaining_elems != 0) {
            asm volatile("vsetvli  %[rvl],  %[rdvl], e64, m1, ta, ma       \n"
                         : [rvl] "+r"(max_vl)
                         : [rdvl] "r"(n_remaining_elems));

            asm volatile(
                // Code
                "vle64.v  v0,    (%[xa])          \n"
                "vle64.v  v8,    (%[ya])          \n"
                "vfmacc.vf v8,    %[a],    v0     \n"
                "vse64.v  v8,    (%[za])          \n"
                // Outputs
                : [xa] "+r"(x_addr), [ya] "+r"(y_addr), [za] "+r"(z_addr)
                // Inputs
                : [a] "f"(a)
                // Clobbers
                : "memory");
        }

        asm volatile("fence");

        end = snrt_mcycle();
    }

    snrt_cluster_hw_barrier();

    return 0;
}
