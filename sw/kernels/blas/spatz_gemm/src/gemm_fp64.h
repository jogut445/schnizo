// Copyright 2025 ETH Zurich and University of Bologna.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0
//
// Adapted version for Schnizo
//
// Author: Pascal Etterli <petterli@student.ethz.ch>
//         Tim Fischer <fischeti@iis.ee.ethz.ch>
//         Luca Bertaccini <lbertaccini@iis.ee.ethz.ch>
//         Luca Colagrande <colluca@iis.ee.ethz.ch>
//         Viviane Potocnik <vivianep@iis.ee.ethz.ch>

// Floating-point multiplications by zero cannot be optimized as in some
// edge cases they do not yield zero:
// - 0f * NaN = NaN
// - 0f * INFINITY == NaN
// Thus in order to optimize it, we need to test for zero. You can use this
// function for free when `multiplier` is a constant.

// #ifdef USE_LMUL_1
// #define LMUL "1"
// #endif
// #ifdef USE_LMUL_2
// #define LMUL "2"
// #endif
// #ifdef USE_LMUL_4
// #define LMUL "4"
// #endif
// #ifdef USE_LMUL_8
// #define LMUL "8"
// #endif

#define LMUL "4"

#define SETVLEN(rd, rs) asm volatile("vsetvli %[vl], %[rvl], e64, m" LMUL ", ta, ma" \
                 : [vl] "=r"(rd) \
                 : [rvl] "r"(rs));



static inline double multiply_opt(double multiplicand, double multiplier) {
    if (multiplier)
        return multiplicand * multiplier;
    else
        return 0;
}

static inline void gemm_fp64_naive(uint32_t setup_ssr, uint32_t partition_banks,
                                   uint32_t transa, uint32_t transb, uint32_t M,
                                   uint32_t N, uint32_t K, void* A_p,
                                   uint32_t lda, void* B_p, uint32_t ldb,
                                   uint32_t beta, void* C_p, uint32_t ldc) {
    double* A = (double*)A_p;
    double* B = (double*)B_p;
    double* C = (double*)C_p;

    uint32_t elems_per_line =
        (snrt_cluster_compute_core_num() * SNRT_TCDM_BANK_WIDTH) /
        sizeof(double);

    if (!transa && !transb) {
        snrt_mcycle();
        for (uint32_t m = 0; m < M; m++) {
            for (uint32_t n = 0; n < N; n++) {
                uint32_t c_idx;
                uint32_t n_inner, n_outer, n_outer_stride;
                if (partition_banks) {
                    n_inner = n % elems_per_line;
                    n_outer = n / elems_per_line;
                    n_outer_stride = SNRT_TCDM_HYPERBANK_WIDTH / sizeof(double);
                    c_idx = m * ldc + n_outer * n_outer_stride + n_inner;
                } else {
                    c_idx = m * ldc + n;
                }
                double c0 = multiply_opt(C[c_idx], beta);
                for (uint32_t k = 0; k < K; k++) {
                    uint32_t a_idx, b_idx;
                    if (partition_banks) {
                        uint32_t k_inner = k % elems_per_line;
                        uint32_t k_outer = k / elems_per_line;
                        uint32_t k_outer_stride =
                            SNRT_TCDM_HYPERBANK_WIDTH / sizeof(double);
                        a_idx = m * lda + k_outer * k_outer_stride + k_inner;
                        b_idx = k * ldb + n_outer * n_outer_stride + n_inner;
                    } else {
                        a_idx = m * lda + k;
                        b_idx = k * ldb + n;
                    }
                    c0 += A[a_idx] * B[b_idx];
                }
                C[c_idx] = c0;
            }
        }
        snrt_mcycle();
    } else if (transa && !transb) {
        snrt_mcycle();
        for (uint32_t m = 0; m < M; m++) {
            for (uint32_t n = 0; n < N; n++) {
                double c0 = multiply_opt(C[m * ldc + n], beta);
                for (uint32_t k = 0; k < K; k++) {
                    c0 += A[k * M * lda + m * lda] * B[k * ldb + n];
                }
                C[m * ldc + n] = c0;
            }
        }
        snrt_mcycle();
    } else if (!transa && transb) {
        snrt_mcycle();
        for (uint32_t m = 0; m < M; m++) {
            for (uint32_t n = 0; n < N; n++) {
                double c0 = multiply_opt(C[m * ldc + n], beta);
                for (uint32_t k = 0; k < K; k++) {
                    c0 += A[m * lda + k] * B[n * ldb + k];
                }
                C[m * ldc + n] = c0;
            }
        }
        snrt_mcycle();
    } else {
        snrt_mcycle();
        for (uint32_t m = 0; m < M; m++) {
            for (uint32_t n = 0; n < N; n++) {
                double c0 = multiply_opt(C[m * ldc + n], beta);
                for (uint32_t k = 0; k < K; k++) {
                    c0 += A[k * M * lda + m * lda] * B[k + n * ldb];
                }
                C[m * ldc + n] = c0;
            }
        }
        snrt_mcycle();
    }
}

static inline void gemm_fp64_opt(uint32_t setup_ssr, uint32_t partition_banks,
                                 uint32_t transa, uint32_t transb, uint32_t M,
                                 uint32_t N, uint32_t K, void* A_p,
                                 uint32_t lda, void* B_p, uint32_t ldb,
                                 uint32_t beta, void* C_p, uint32_t ldc) {
    double* A = (double*)A_p;
    double* B = (double*)B_p;
    double* C = (double*)C_p;

    // Unrolling factor of most inner loop.
    // Should be at least as high as the FMA delay
    // for maximum utilization
    const uint32_t unroll = 4; // limitted by the number of slots

    // Schnizo frep is non nested so we have to compute the address offsets manually.
    double* ptr_a;
    double* ptr_b;
    uint32_t inc_a, inc_b;

    snrt_mcycle();

    for (uint32_t m = 0; m < M; m++) {
        uint32_t n = 0;
        // Guard that we only compute columns which can be fully unrolled in the N dimension
        for (uint32_t n0 = 0; n0 < N / unroll; n0++) {
            double c[unroll];

            // Start addresses
            // Assumes that A and B is regular and not transposed

            // Set the A pointer to the beginning of the row
            ptr_a = &A[m*lda];
            inc_a = sizeof(double);
            // Set the B pointer to the next columns
            ptr_b = &B[n];
            inc_b = sizeof(double) * ldb;

            // Load intermediate result - beta is always zero
            c[0] = 0.0;
            c[1] = 0.0;
            c[2] = 0.0;
            c[3] = 0.0;

            asm volatile(
            "frep.o %[n_frep], 11, 0, 0 \n"
            "fld     ft0,  0(%[ptr_a])\n"
            "fld     ft1,  0(%[ptr_b])\n"
            "fld     ft3,  8(%[ptr_b])\n"
            "fld     ft5, 16(%[ptr_b])\n"
            "fld     ft7, 24(%[ptr_b])\n"
            "add     %[ptr_a], %[ptr_a], %[inc_a]\n"
            "add     %[ptr_b], %[ptr_b], %[inc_b]\n"
            "fmadd.d %[c0], ft0, ft1, %[c0] \n"
            "fmadd.d %[c1], ft0, ft3, %[c1] \n"
            "fmadd.d %[c2], ft0, ft5, %[c2] \n"
            "fmadd.d %[c3], ft0, ft7, %[c3] \n"
            : [c0] "+f"(c[0]), [c1] "+f"(c[1]),
              [c2] "+f"(c[2]), [c3] "+f"(c[3]),
              [ptr_a] "+r"(ptr_a), [ptr_b] "+r"(ptr_b)
            : [inc_a] "r"(inc_a), [inc_b] "r"(inc_b),
              [n_frep] "r"(K - 1) // frep iterates n+1 times
            : "ft0", "ft1", "ft3", "ft5", "ft7", "memory");

            // Store results back
            C[m * ldc + n + 0] = c[0];
            C[m * ldc + n + 1] = c[1];
            C[m * ldc + n + 2] = c[2];
            C[m * ldc + n + 3] = c[3];
            n += unroll;
        }
    }

    snrt_mcycle();
}

// Requires that A and B is regular and not transposed!
static inline void gemm_fp64_opt_4FPUs(uint32_t setup_ssr, uint32_t partition_banks,
                                 uint32_t transa, uint32_t transb, uint32_t M,
                                 uint32_t N, uint32_t K, void* A_p,
                                 uint32_t lda, void* B_p, uint32_t ldb,
                                 uint32_t beta, void* C_p, uint32_t ldc) {
    double* A = (double*)A_p;
    double* B = (double*)B_p;
    double* C = (double*)C_p;

    // Unrolling factor of most inner loop.
    // Should be at least as high as the FMA delay
    // for maximum utilization
    const uint32_t unroll = 4; // limitted by the number of slots

    // Schnizo frep is non nested so we have to compute the address offsets manually.
    double* ptr_a0;
    double* ptr_a1;
    double* ptr_a2;
    double* ptr_a3;
    double* ptr_b;
    uint32_t inc_a, inc_b;

    snrt_mcycle();

    for (uint32_t m = 0; m < M; m+=4) {
        uint32_t n = 0;
        // Guard that we only compute columns which can be fully unrolled in the N dimension
        for (uint32_t n0 = 0; n0 < N / unroll; n0++) {
            double c[16];

            // Start addresses
            // Assumes that A and B is regular and not transposed

            // Set the A pointers
            ptr_a0 = &A[m*lda];
            ptr_a1 = &A[(m+1)*lda];
            ptr_a2 = &A[(m+2)*lda];
            ptr_a3 = &A[(m+3)*lda];
            inc_a = sizeof(double);
            // Set the B pointer to the next columns
            ptr_b = &B[n];
            inc_b = sizeof(double) * ldb;

            // Load intermediate result - beta is always zero
            for (int i = 0; i < 16; i++) c[i] = 0.0;

            asm volatile(
            "frep.o %[n_frep], 29, 0, 0 \n"
            "fld     ft1,  0(%[ptr_b])\n"
            "fld     ft2,  8(%[ptr_b])\n"
            "fld     ft3, 16(%[ptr_b])\n"
            "fld     ft4, 24(%[ptr_b])\n"
            
            "fld     ft5,  0(%[ptr_a0])\n"
            "fmadd.d %[c00], ft5, ft1, %[c00] \n"
            "fmadd.d %[c01], ft5, ft2, %[c01] \n"
            "fmadd.d %[c02], ft5, ft3, %[c02] \n"
            "fmadd.d %[c03], ft5, ft4, %[c03] \n"
            
            "fld     ft6,  0(%[ptr_a1])\n"
            "fmadd.d %[c10], ft6, ft1, %[c10] \n"
            "fmadd.d %[c11], ft6, ft2, %[c11] \n"
            "fmadd.d %[c12], ft6, ft3, %[c12] \n"
            "fmadd.d %[c13], ft6, ft4, %[c13] \n"
            
            "fld     ft7,  0(%[ptr_a2])\n"
            "fmadd.d %[c20], ft7, ft1, %[c20] \n"
            "fmadd.d %[c21], ft7, ft2, %[c21] \n"
            "fmadd.d %[c22], ft7, ft3, %[c22] \n"
            "fmadd.d %[c23], ft7, ft4, %[c23] \n"
            
            "fld     ft8,  0(%[ptr_a3])\n"
            "fmadd.d %[c30], ft8, ft1, %[c30] \n"
            "fmadd.d %[c31], ft8, ft2, %[c31] \n"
            "fmadd.d %[c32], ft8, ft3, %[c32] \n"
            "fmadd.d %[c33], ft8, ft4, %[c33] \n"

            "add     %[ptr_a0], %[ptr_a0], %[inc_a]\n"
            "add     %[ptr_a1], %[ptr_a1], %[inc_a]\n"
            "add     %[ptr_a2], %[ptr_a2], %[inc_a]\n"
            "add     %[ptr_a3], %[ptr_a3], %[inc_a]\n"
            "add     %[ptr_b], %[ptr_b], %[inc_b]\n"
            : [c00] "+f"(c[0]), [c01] "+f"(c[1]), [c02] "+f"(c[2]), [c03] "+f"(c[3]),
              [c10] "+f"(c[4]), [c11] "+f"(c[5]), [c12] "+f"(c[6]), [c13] "+f"(c[7]),
              [c20] "+f"(c[8]), [c21] "+f"(c[9]), [c22] "+f"(c[10]), [c23] "+f"(c[11]),
              [c30] "+f"(c[12]), [c31] "+f"(c[13]), [c32] "+f"(c[14]), [c33] "+f"(c[15]),
              [ptr_a0] "+r"(ptr_a0), [ptr_a1] "+r"(ptr_a1), [ptr_a2] "+r"(ptr_a2), [ptr_a3] "+r"(ptr_a3),
              [ptr_b] "+r"(ptr_b)
            : [inc_a] "r"(inc_a), [inc_b] "r"(inc_b),
              [n_frep] "r"(K - 1)
            : "ft1", "ft2", "ft3", "ft4", "ft5", "ft6", "ft7", "ft8", "memory");

            // Store results back
            C[m * ldc + n + 0] = c[0];
            C[m * ldc + n + 1] = c[1];
            C[m * ldc + n + 2] = c[2];
            C[m * ldc + n + 3] = c[3];
            C[(m+1) * ldc + n + 0] = c[4];
            C[(m+1) * ldc + n + 1] = c[5];
            C[(m+1) * ldc + n + 2] = c[6];
            C[(m+1) * ldc + n + 3] = c[7];
            C[(m+2) * ldc + n + 0] = c[8];
            C[(m+2) * ldc + n + 1] = c[9];
            C[(m+2) * ldc + n + 2] = c[10];
            C[(m+2) * ldc + n + 3] = c[11];
            C[(m+3) * ldc + n + 0] = c[12];
            C[(m+3) * ldc + n + 1] = c[13];
            C[(m+3) * ldc + n + 2] = c[14];
            C[(m+3) * ldc + n + 3] = c[15];
            n += unroll;
        }
    }

    snrt_mcycle();
}

// The pointer are called according to which loop they are updated in
static inline void gemm_fp64_vec_dummy(uint32_t setup_ssr, uint32_t partition_banks,
                                       uint32_t transa, uint32_t transb,
                                       uint32_t M, uint32_t N, uint32_t K,
                                       void *A_p, uint32_t lda, void *B_p, uint32_t ldb,
                                       uint32_t beta, void *C_p, uint32_t ldc) {
    // Only supports !transa && !transb and non bank-partitioned layout.
    if (transa || transb || partition_banks)
        return; // Fallback not implemented here.

    snrt_mcycle();

    int computed_col = 0;
    while (computed_col < N) {

        int vl;
        SETVLEN(vl, N-computed_col)

        double * C_l1 = (double *)C_p + computed_col;
        double * B_l1 = (double *)B_p + computed_col;

        double *C_l2_1 = C_l1;
        double *C_l2_2 = C_l1 + ldc;
        double *A_l2_1 = (double *)A_p;
        double *A_l2_2 = (double *)A_p + lda;

        for(int i = 0; i+1 < M; i+=2) {

            int j = 0;

            double * B_l3_1 = B_l1;

            double *A_l3_1 = A_l2_1;
            double *A_l3_2 = A_l2_2;

            double t0 = *A_l3_1;
            double t1 = *A_l3_2;

            if (beta) {
                asm volatile("vle64.v v0, (%0);" ::"r"(C_l2_1));
                asm volatile("vle64.v v8, (%0);" ::"r"(C_l2_2));
            } else {
                asm volatile("vle64.v v16, (%0);" ::"r"(B_l3_1));
                B_l3_1 += ldb;
                asm volatile("vfmul.vf v0, v16, %0" ::"f"(t0));
                A_l3_1++; t0 = *A_l3_1;
                asm volatile("vfmul.vf v8, v16, %0" ::"f"(t1));
                A_l3_2++; t1 = *A_l3_2;
                j++;
            }
            

            asm volatile("vse64.v v0, (%0);" ::"r"(C_l2_1));
            asm volatile("vse64.v v8, (%0);" ::"r"(C_l2_2));

            C_l2_1 += ldc * 2;
            C_l2_2 = C_l2_1 + ldc;
            A_l2_1 += lda * 2;
            A_l2_2 = A_l2_1 + lda;
        }

        if (M % 2) {

            double* B_l3_1 = B_l1;
            double* A_l3_1 = A_l2_1;

            double t0 = *A_l3_1;

            int j;

            if (beta) asm volatile("vle64.v v0, (%0);" ::"r"(C_l2_1));
            else {
                asm volatile("vle64.v v16, (%0);" ::"r"(B_l3_1));
                B_l3_1 += ldb;
                asm volatile("vfmul.vf v0, v16, %0" ::"f"(t0));
                A_l3_1++; t0 = *A_l3_1;
                j = 1;
            } 
                
            #pragma clang loop unroll(disable)
            for (; j < K; j++) {
                asm volatile("vle64.v v16, (%0);" ::"r"(B_l3_1));
                B_l3_1 += ldb;
                asm volatile("vfmacc.vf v0, %0, v16" ::"f"(t0));
                A_l3_1++; t0 = *A_l3_1;
            }

            asm volatile("vse64.v v0, (%0);" ::"r"(C_l2_1) : "memory");
        }

        computed_col += vl;
    }

    asm volatile("fence");
    snrt_mcycle();
}


static inline void gemm_fp64_vec_dummy_scalar(uint32_t setup_ssr, uint32_t partition_banks,
                                       uint32_t transa, uint32_t transb,
                                       uint32_t M, uint32_t N, uint32_t K,
                                       void *A_p, uint32_t lda, void *B_p, uint32_t ldb,
                                       uint32_t beta, void *C_p, uint32_t ldc) {
    // Only supports !transa && !transb and non bank-partitioned layout.
    if (transa || transb || partition_banks)
        return; // Fallback not implemented here.

    snrt_mcycle();

    uint64_t inc_b = (uint64_t)ldb * sizeof(double);
    uint64_t inc_a = sizeof(double);

    int computed_col = 0;
    while (computed_col < N) {

        int vl;
        SETVLEN(vl, N-computed_col)

        double * C_l1 = (double *)C_p + computed_col;
        double * B_l1 = (double *)B_p + computed_col;

        double *C_l2_1 = C_l1;
        double *C_l2_2 = C_l1 + ldc;
        double *A_l2_1 = (double *)A_p;
        double *A_l2_2 = (double *)A_p + lda;

        for(int i = 0; i+1 < M; i+=2) {

            int j = 0;

            double * B_l3_1 = B_l1;

            double *A_l3_1 = A_l2_1;
            double *A_l3_2 = A_l2_2;

            double t0 = *A_l3_1;
            double t1 = *A_l3_2;

            /* Assembly label to locate this section in objdump */
            asm volatile(
                "dummy_scalar_loop%=: \n"
                : /* no outputs */ : /* no inputs */ : "memory");


            asm volatile(
                "frep.o  %[n_frep], 5, 0, 0 \n"
                // Body:
                // "vle64.v v16, (%[ptr_b]) \n"
                "add     %[ptr_b], %[ptr_b], %[inc_b] \n"
                // "vfmacc.vf v0, %[ft0], v16 \n"
                "add     %[ptr_a0], %[ptr_a0], %[inc_a] \n"
                "fld     %[ft0], 0(%[ptr_a0]) \n"
                // "vfmacc.vf v8, %[ft1], v16 \n"
                "add     %[ptr_a1], %[ptr_a1], %[inc_a] \n"
                "fld     %[ft1], 0(%[ptr_a1]) \n"
                : [ptr_b] "+r"(B_l3_1), [ptr_a0] "+r"(A_l3_1),
                  [ptr_a1] "+r"(A_l3_2), [ft0] "+f"(t0), [ft1] "+f"(t1)
                :
                [inc_b] "r"(inc_b), [inc_a] "r"(inc_a), [n_frep] "r"(beta ? K - 1 : K - 2)
                :);

            C_l2_1 += ldc * 2;
            C_l2_2 = C_l2_1 + ldc;
            A_l2_1 += lda * 2;
            A_l2_2 = A_l2_1 + lda;
        }

        computed_col += vl;
    }

    asm volatile("fence");
    snrt_mcycle();
}




// Requires that A and B is regular and not transposed!
// Should work for all MNK possible dimensions, not tested for very specific
// edge cases
static inline void gemm_fp64_vec_frep(
    uint32_t setup_ssr, uint32_t partition_banks, uint32_t transa,
    uint32_t transb, uint32_t M, uint32_t N, uint32_t K, void* A_p,
    uint32_t lda, void* B_p, uint32_t ldb, uint32_t beta, void* C_p,
    uint32_t ldc) {

    if (transa || transb || partition_banks)
        return;  // Fallback not implemented here.

    snrt_mcycle();

    uint64_t inc_b = (uint64_t)ldb * sizeof(double);
    uint64_t inc_a = sizeof(double);

    int computed_col = 0;
    while (computed_col < N) {
        int vl;
        SETVLEN(vl, N-computed_col)

        double* C_l1 = (double*)C_p + computed_col;
        double* B_l1 = (double*)B_p + computed_col;
        
        double* C_l2_1 = C_l1;
        double* C_l2_2 = C_l1 + ldc;
        double* A_l2_1 = (double*)A_p;
        double* A_l2_2 = (double*)A_p + lda;

        for (int i = 0; i+1 < M; i += 2) {

            double* B_l3_1 = B_l1;
            double* A_l3_1 = A_l2_1;
            double* A_l3_2 = A_l2_2;

            // Load the first two scalar values
            double t0 = *A_l3_1;
            double t1 = *A_l3_2;

            if (beta) {
                asm volatile("vle64.v v0, (%0);" ::"r"(C_l2_1));
                asm volatile("vle64.v v8, (%0);" ::"r"(C_l2_2));
            } else {
                asm volatile("vle64.v v16, (%0);" ::"r"(B_l3_1));
                B_l3_1 += ldb;
                asm volatile("vfmul.vf v0, v16, %0" ::"f"(t0));
                A_l3_1++; t0 = *A_l3_1;
                asm volatile("vfmul.vf v8, v16, %0" ::"f"(t1));
                A_l3_2++; t1 = *A_l3_2;
            }


            asm volatile(
                "frep.o  %[n_frep], 8, 0, 0 \n"
                // Body:
                "vle64.v v16, (%[ptr_b]) \n"
                "add     %[ptr_b], %[ptr_b], %[inc_b] \n"
                "vfmacc.vf v0, %[ft0], v16 \n"
                "add     %[ptr_a0], %[ptr_a0], %[inc_a] \n"
                "fld     %[ft0], 0(%[ptr_a0]) \n"
                "vfmacc.vf v8, %[ft1], v16 \n"
                "add     %[ptr_a1], %[ptr_a1], %[inc_a] \n"
                "fld     %[ft1], 0(%[ptr_a1]) \n"
                : [ptr_b] "+r"(B_l3_1), [ptr_a0] "+r"(A_l3_1),
                  [ptr_a1] "+r"(A_l3_2), [ft0] "+f"(t0), [ft1] "+f"(t1)
                :
                [inc_b] "r"(inc_b), [inc_a] "r"(inc_a), [n_frep] "r"(beta ? K - 1 : K - 2)
                :);

            asm volatile("vse64.v v0, (%0);" ::"r"(C_l2_1) : "memory");
            asm volatile("vse64.v v8, (%0);" ::"r"(C_l2_2) : "memory");
            
            C_l2_1 += ldc * 2;
            C_l2_2 = C_l2_1 + ldc;
            A_l2_1 += lda * 2;
            A_l2_2 = A_l2_1 + lda;
        }

        if (M % 2) {

            double* B_l3_1 = B_l1;
            double* A_l3_1 = A_l2_1;

            double t0 = *A_l3_1;

            if (beta) asm volatile("vle64.v v0, (%0);" ::"r"(C_l2_1));
            else {
                asm volatile("vle64.v v16, (%0);" ::"r"(B_l3_1));
                B_l3_1 += ldb;
                asm volatile("vfmul.vf v0, v16, %0" ::"f"(t0));
                A_l3_1++; t0 = *A_l3_1;
            }

            asm volatile(
                "frep.o  %[n_frep], 5, 0, 0 \n"
                // Body:
                "vle64.v v16, (%[ptr_b]) \n"
                "add     %[ptr_b], %[ptr_b], %[inc_b] \n"
                "vfmacc.vf v0, %[ft0], v16 \n"
                "add     %[ptr_a0], %[ptr_a0], %[inc_a] \n"
                "fld     %[ft0], 0(%[ptr_a0]) \n"
                : [ptr_b] "+r"(B_l3_1), [ptr_a0] "+r"(A_l3_1), [ft0] "+f"(t0)
                : [inc_b] "r"(inc_b), [inc_a] "r"(inc_a), [n_frep] "r"(beta ? K - 1 : K - 2)
                :);

            asm volatile("vse64.v v0, (%0);" ::"r"(C_l2_1) : "memory");
        }

        computed_col += vl;
    }

    asm volatile ("fence");
    snrt_mcycle();
}


// Requires that A and B is regular and not transposed!
// Should work for all MNK possible dimensions, not tested for very specific
// edge cases
static inline void gemm_fp64_vec_frep_test(
    uint32_t setup_ssr, uint32_t partition_banks, uint32_t transa,
    uint32_t transb, uint32_t M, uint32_t N, uint32_t K, void* A_p,
    uint32_t lda, void* B_p, uint32_t ldb, uint32_t beta, void* C_p,
    uint32_t ldc) {

    if (transa || transb || partition_banks)
        return;  // Fallback not implemented here.

    snrt_mcycle();

    uint64_t inc_b = (uint64_t)ldb * sizeof(double);
    uint64_t inc_a = sizeof(double);

    int computed_col = 0;
    while (computed_col < N) {
        int vl;
        SETVLEN(vl, N-computed_col)

        double* C_l1 = (double*)C_p + computed_col;
        double* B_l1 = (double*)B_p + computed_col;
        
        double* C_l2_1 = C_l1;
        double* A_l2_1 = (double*)A_p;

        for (int i = 0; i < M; i ++) {

            double* B_l3_1 = B_l1;
            double* A_l3_1 = A_l2_1;

            double t0 = *A_l3_1;

            if (beta) {
                asm volatile("vle64.v v0, (%0);" ::"r"(C_l2_1));
            } else {
                asm volatile("vle64.v v16, (%0);" ::"r"(B_l3_1));
                B_l3_1 += ldb;
                asm volatile("vfmul.vf v0, v16, %0" ::"f"(t0));
                A_l3_1++; t0 = *A_l3_1;
            }


            asm volatile(
                "frep.o  %[n_frep], 8, 0, 0 \n"
                // Body:
                "vle64.v v16, (%[ptr_b]) \n"
                "add     %[ptr_b], %[ptr_b], %[inc_b] \n"
                "vfmacc.vf v0, %[ft0], v16 \n"
                "add     %[ptr_a0], %[ptr_a0], %[inc_a] \n"
                "fld     %[ft0], 0(%[ptr_a0]) \n"
                : [ptr_b] "+r"(B_l3_1), [ptr_a0] "+r"(A_l3_1), [ft0] "+f"(t0)
                :
                [inc_b] "r"(inc_b), [inc_a] "r"(inc_a), [n_frep] "r"(beta ? K - 1 : K - 2)
                :);

            asm volatile("vse64.v v0, (%0);" ::"r"(C_l2_1) : "memory");
            
            C_l2_1 += ldc;
            A_l2_1 += lda;
        }

        computed_col += vl;
    }

    asm volatile ("fence");
    snrt_mcycle();
}



// Requires that A and B is regular and not transposed!
// Should work for all MNK possible dimensions, not tested for very specific
// edge cases
static inline void gemm_fp64_vec_frep_unrolled(
    uint32_t setup_ssr, uint32_t partition_banks, uint32_t transa,
    uint32_t transb, uint32_t M, uint32_t N, uint32_t K, void* A_p,
    uint32_t lda, void* B_p, uint32_t ldb, uint32_t beta, void* C_p,
    uint32_t ldc) {

    if (transa || transb || partition_banks)
        return;  // Fallback not implemented here.

    snrt_mcycle();

    uint64_t inc_b = (uint64_t)ldb * sizeof(double);
    uint64_t inc_a = sizeof(double);

    int computed_col = 0;
    while (computed_col < N) {
        int vl;
        SETVLEN(vl, N-computed_col)

        double* C_l1 = (double*)C_p + computed_col;
        double* B_l1 = (double*)B_p + computed_col;
        
        double* C_l2_1 = C_l1;
        double* C_l2_2 = C_l1 + ldc;
        double* A_l2_1 = (double*)A_p;
        double* A_l2_2 = (double*)A_p + lda;

        for (int i = 0; i+1 < M; i += 2) {

            double* B_l3_1 = B_l1;
            double* A_l3_1 = A_l2_1;
            double* A_l3_2 = A_l2_2;

            // Load the first two scalar values
            double t0 = *A_l3_1;
            double t1 = *A_l3_2;

            if (beta) {
                asm volatile("vle64.v v0, (%0);" ::"r"(C_l2_1));
                asm volatile("vle64.v v8, (%0);" ::"r"(C_l2_2));
            } else {
                asm volatile("vle64.v v16, (%0);" ::"r"(B_l3_1));
                B_l3_1 += ldb;
                asm volatile("vle64.v v24, (%0);" ::"r"(B_l3_1));
                B_l3_1 += ldb;
                asm volatile("vfmul.vf v0, v16, %0" ::"f"(t0));
                A_l3_1++; t0 = *A_l3_1;
                asm volatile("vfmul.vf v8, v16, %0" ::"f"(t1));
                A_l3_2++; t1 = *A_l3_2;
                asm volatile("vfmacc.vf v0, %0, v24" ::"f"(t0));
                A_l3_1++; t0 = *A_l3_1;
                asm volatile("vfmacc.vf v8, %0, v24" ::"f"(t1));
                A_l3_2++; t1 = *A_l3_2;
            }


            asm volatile(
                "frep.o  %[n_frep], 16, 0, 0 \n"
                // Body:
                "vle64.v v16, (%[ptr_b]) \n"
                "add     %[ptr_b], %[ptr_b], %[inc_b] \n"
                "vle64.v v24, (%[ptr_b]) \n"
                "add     %[ptr_b], %[ptr_b], %[inc_b] \n"
                "vfmacc.vf v0, %[ft0], v16 \n"
                "add     %[ptr_a0], %[ptr_a0], %[inc_a] \n"
                "fld     %[ft0], 0(%[ptr_a0]) \n"
                "vfmacc.vf v8, %[ft1], v16 \n"
                "add     %[ptr_a1], %[ptr_a1], %[inc_a] \n"
                "fld     %[ft1], 0(%[ptr_a1]) \n"
                "vfmacc.vf v0, %[ft0], v24 \n"
                "add     %[ptr_a0], %[ptr_a0], %[inc_a] \n"
                "fld     %[ft0], 0(%[ptr_a0]) \n"
                "vfmacc.vf v8, %[ft1], v24 \n"
                "add     %[ptr_a1], %[ptr_a1], %[inc_a] \n"
                "fld     %[ft1], 0(%[ptr_a1]) \n"
                : [ptr_b] "+r"(B_l3_1), [ptr_a0] "+r"(A_l3_1),
                  [ptr_a1] "+r"(A_l3_2), [ft0] "+f"(t0), [ft1] "+f"(t1)
                :
                [inc_b] "r"(inc_b), [inc_a] "r"(inc_a), [n_frep] "r"(beta ? K / 2 - 1 : K/2 - 2)
                :);

            if (K % 2) {
                asm volatile(
                    "vle64.v v16, (%[ptr_b]) \n"
                    "vfmacc.vf v0, %[ft0], v16 \n"
                    "vfmacc.vf v8, %[ft1], v16 \n"
                    :
                    : [ptr_b] "r"(B_l3_1), [ft0] "f"(t0), [ft1] "f"(t1)
                    :);
            }

            asm volatile("vse64.v v0, (%0);" ::"r"(C_l2_1) : "memory");
            asm volatile("vse64.v v8, (%0);" ::"r"(C_l2_2) : "memory");
            
            C_l2_1 += ldc * 2;
            C_l2_2 = C_l2_1 + ldc;
            A_l2_1 += lda * 2;
            A_l2_2 = A_l2_1 + lda;
        }

        if (M % 2) {

            double* B_l3_1 = B_l1;
            double* A_l3_1 = A_l2_1;

            double t0 = *A_l3_1;
            
            if (beta) asm volatile("vle64.v v0, (%0);" ::"r"(C_l2_1));
            else {
                asm volatile("vle64.v v16, (%0);" ::"r"(B_l3_1));
                B_l3_1 += ldb;
                asm volatile("vle64.v v24, (%0);" ::"r"(B_l3_1));
                B_l3_1 += ldb;
                asm volatile("vfmul.vf v0, v16, %0" ::"f"(t0));
                A_l3_1++; t0 = *A_l3_1;
                asm volatile("vfmacc.vf v0, %0, v24" ::"f"(t0));
                A_l3_1++; t0 = *A_l3_1;
            }


            asm volatile(
                "frep.o  %[n_frep], 10, 0, 0 \n"
                // Body:
                "vle64.v v16, (%[ptr_b]) \n"
                "add     %[ptr_b], %[ptr_b], %[inc_b] \n"
                "vle64.v v24, (%[ptr_b]) \n"
                "add     %[ptr_b], %[ptr_b], %[inc_b] \n"
                "vfmacc.vf v0, %[ft0], v16 \n"
                "add     %[ptr_a0], %[ptr_a0], %[inc_a] \n"
                "fld     %[ft0], 0(%[ptr_a0]) \n"
                "vfmacc.vf v0, %[ft0], v24 \n"
                "add     %[ptr_a0], %[ptr_a0], %[inc_a] \n"
                "fld     %[ft0], 0(%[ptr_a0]) \n"
                : [ptr_b] "+r"(B_l3_1), [ptr_a0] "+r"(A_l3_1), [ft0] "+f"(t0)
                : [inc_b] "r"(inc_b), [inc_a] "r"(inc_a), [n_frep] "r"(beta ? K / 2 - 1 : K/2 - 2)
                :);

            if (K % 2)
                asm volatile(
                    "vle64.v v16, (%[ptr_b]) \n"
                    "vfmacc.vf v0, %[ft0], v16 \n"
                    :
                    : [ptr_b] "r"(B_l3_1), [ft0] "f"(t0)
                    :);

            asm volatile("vse64.v v0, (%0);" ::"r"(C_l2_1) : "memory");
        }

        computed_col += vl;
    }

    asm volatile ("fence");
    snrt_mcycle();
}


// Needs at least K = 2
// The pointer are called according to which loop they are updated in
static inline void gemm_fp64_vec_base_unrolled(uint32_t setup_ssr, uint32_t partition_banks,
                                       uint32_t transa, uint32_t transb,
                                       uint32_t M, uint32_t N, uint32_t K,
                                       void *A_p, uint32_t lda, void *B_p, uint32_t ldb,
                                       uint32_t beta, void *C_p, uint32_t ldc) {
    // Only supports !transa && !transb and non bank-partitioned layout.
    if (transa || transb || partition_banks)
        return; // Fallback not implemented here.

    snrt_mcycle();

    int computed_col = 0;
    while (computed_col < N) {

        int vl;
        SETVLEN(vl, N-computed_col)

        double * C_l1 = (double *)C_p + computed_col;
        double * B_l1 = (double *)B_p + computed_col;

        double *C_l2_1 = C_l1;
        double *C_l2_2 = C_l1 + ldc;
        double *A_l2_1 = (double *)A_p;
        double *A_l2_2 = (double *)A_p + lda;

        for(int i = 0; i+1 < M; i+=2) {

            double * B_l3_1 = B_l1;

            double *A_l3_1 = A_l2_1;
            double *A_l3_2 = A_l2_2;

            double t0 = *A_l3_1;
            double t1 = *A_l3_2;

            int j;

            if (beta) {
                asm volatile("vle64.v v0, (%0);" ::"r"(C_l2_1));
                asm volatile("vle64.v v8, (%0);" ::"r"(C_l2_2));
                j=0;
            } else {
                asm volatile("vle64.v v16, (%0);" ::"r"(B_l3_1));
                B_l3_1 += ldb;
                asm volatile("vle64.v v24, (%0);" ::"r"(B_l3_1));
                B_l3_1 += ldb;
                asm volatile("vfmul.vf v0, v16, %0" ::"f"(t0));
                A_l3_1++; t0 = *A_l3_1;
                asm volatile("vfmul.vf v8, v16, %0" ::"f"(t1));
                A_l3_2++; t1 = *A_l3_2;
                asm volatile("vfmacc.vf v0, %0, v24" ::"f"(t0));
                A_l3_1++; t0 = *A_l3_1;
                asm volatile("vfmacc.vf v8, %0, v24" ::"f"(t1));
                A_l3_2++; t1 = *A_l3_2;
                j = 2;
            }
            
            #pragma clang loop unroll(disable)
            for (; j + 1 < K; j+=2) {
                asm volatile("vle64.v v16, (%0);" ::"r"(B_l3_1));
                B_l3_1 += ldb;
                asm volatile("vle64.v v24, (%0);" ::"r"(B_l3_1));
                B_l3_1 += ldb;
                asm volatile("vfmacc.vf v0, %0, v16" ::"f"(t0));
                A_l3_1++; t0 = *A_l3_1;
                asm volatile("vfmacc.vf v8, %0, v16" ::"f"(t1));
                A_l3_2++; t1 = *A_l3_2;
                asm volatile("vfmacc.vf v0, %0, v24" ::"f"(t0));
                A_l3_1++; t0 = *A_l3_1;
                asm volatile("vfmacc.vf v8, %0, v24" ::"f"(t1));
                A_l3_2++; t1 = *A_l3_2;
            }

            if (j < K) {
                asm volatile("vle64.v v16, (%0);" ::"r"(B_l3_1));
                asm volatile("vfmacc.vf v0, %0, v16" ::"f"(t0));
                asm volatile("vfmacc.vf v8, %0, v16" ::"f"(t1));
            }

            asm volatile("vse64.v v0, (%0);" ::"r"(C_l2_1));
            asm volatile("vse64.v v8, (%0);" ::"r"(C_l2_2));

            C_l2_1 += ldc * 2;
            C_l2_2 = C_l2_1 + ldc;
            A_l2_1 += lda * 2;
            A_l2_2 = A_l2_1 + lda;
        }

        if (M % 2) {
            double* B_l3_1 = B_l1;
            double* A_l3_1 = A_l2_1;

            double t0 = *A_l3_1;

            int j;

            if (beta) asm volatile("vle64.v v0, (%0);" ::"r"(C_l2_1));
            else {
                asm volatile("vle64.v v16, (%0);" ::"r"(B_l3_1));
                B_l3_1 += ldb;
                asm volatile("vle64.v v24, (%0);" ::"r"(B_l3_1));
                B_l3_1 += ldb;
                asm volatile("vfmul.vf v0, v16, %0" ::"f"(t0));
                A_l3_1++; t0 = *A_l3_1;
                asm volatile("vfmacc.vf v0, %0, v24" ::"f"(t0));
                A_l3_1++; t0 = *A_l3_1;
                j = 2;
            }


            #pragma clang loop unroll(disable)
            for (; j + 1 < K; j+=2) {
                asm volatile("vle64.v v16, (%0);" ::"r"(B_l3_1));
                B_l3_1 += ldb;
                asm volatile("vle64.v v24, (%0);" ::"r"(B_l3_1));
                B_l3_1 += ldb;
                asm volatile("vfmacc.vf v0, %0, v16" ::"f"(t0));
                A_l3_1++; t0 = *A_l3_1;
                asm volatile("vfmacc.vf v0, %0, v24" ::"f"(t0));
                A_l3_1++; t0 = *A_l3_1;
            }

            if (j < K) {
                asm volatile("vle64.v v16, (%0);" ::"r"(B_l3_1));
                asm volatile("vfmacc.vf v0, %0, v16" ::"f"(t0));
            }

            asm volatile("vse64.v v0, (%0);" ::"r"(C_l2_1) : "memory");
        }

        computed_col += vl;
    }

    asm volatile("fence");
    snrt_mcycle();
}


// The pointer are called according to which loop they are updated in
static inline void gemm_fp64_vec_base(uint32_t setup_ssr, uint32_t partition_banks,
                                       uint32_t transa, uint32_t transb,
                                       uint32_t M, uint32_t N, uint32_t K,
                                       void *A_p, uint32_t lda, void *B_p, uint32_t ldb,
                                       uint32_t beta, void *C_p, uint32_t ldc) {
    // Only supports !transa && !transb and non bank-partitioned layout.
    if (transa || transb || partition_banks)
        return; // Fallback not implemented here.

    snrt_mcycle();

    int computed_col = 0;
    while (computed_col < N) {

        int vl;
        SETVLEN(vl, N-computed_col)

        double * C_l1 = (double *)C_p + computed_col;
        double * B_l1 = (double *)B_p + computed_col;

        double *C_l2_1 = C_l1;
        double *C_l2_2 = C_l1 + ldc;
        double *A_l2_1 = (double *)A_p;
        double *A_l2_2 = (double *)A_p + lda;

        for(int i = 0; i+1 < M; i+=2) {

            int j = 0;

            double * B_l3_1 = B_l1;

            double *A_l3_1 = A_l2_1;
            double *A_l3_2 = A_l2_2;

            double t0 = *A_l3_1;
            double t1 = *A_l3_2;

            if (beta) {
                asm volatile("vle64.v v0, (%0);" ::"r"(C_l2_1));
                asm volatile("vle64.v v8, (%0);" ::"r"(C_l2_2));
            } else {
                asm volatile("vle64.v v16, (%0);" ::"r"(B_l3_1));
                B_l3_1 += ldb;
                asm volatile("vfmul.vf v0, v16, %0" ::"f"(t0));
                A_l3_1++; t0 = *A_l3_1;
                asm volatile("vfmul.vf v8, v16, %0" ::"f"(t1));
                A_l3_2++; t1 = *A_l3_2;
                j++;
            }
            
            #pragma clang loop unroll(disable)
            for (; j < K; j++) {
                asm volatile("vle64.v v16, (%0);" ::"r"(B_l3_1));
                B_l3_1 += ldb;
                asm volatile("vfmacc.vf v0, %0, v16" ::"f"(t0));
                A_l3_1++; t0 = *A_l3_1;
                asm volatile("vfmacc.vf v8, %0, v16" ::"f"(t1));
                A_l3_2++; t1 = *A_l3_2;
            }

            asm volatile("vse64.v v0, (%0);" ::"r"(C_l2_1));
            asm volatile("vse64.v v8, (%0);" ::"r"(C_l2_2));

            C_l2_1 += ldc * 2;
            C_l2_2 = C_l2_1 + ldc;
            A_l2_1 += lda * 2;
            A_l2_2 = A_l2_1 + lda;
        }

        if (M % 2) {

            double* B_l3_1 = B_l1;
            double* A_l3_1 = A_l2_1;

            double t0 = *A_l3_1;

            int j;

            if (beta) asm volatile("vle64.v v0, (%0);" ::"r"(C_l2_1));
            else {
                asm volatile("vle64.v v16, (%0);" ::"r"(B_l3_1));
                B_l3_1 += ldb;
                asm volatile("vfmul.vf v0, v16, %0" ::"f"(t0));
                A_l3_1++; t0 = *A_l3_1;
                j = 1;
            } 
                
            #pragma clang loop unroll(disable)
            for (; j < K; j++) {
                asm volatile("vle64.v v16, (%0);" ::"r"(B_l3_1));
                B_l3_1 += ldb;
                asm volatile("vfmacc.vf v0, %0, v16" ::"f"(t0));
                A_l3_1++; t0 = *A_l3_1;
            }

            asm volatile("vse64.v v0, (%0);" ::"r"(C_l2_1) : "memory");
        }

        computed_col += vl;
    }

    asm volatile("fence");
    snrt_mcycle();
}

