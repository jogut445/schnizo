# Copyright 2026 ETH Zurich and University of Bologna.
# Licensed under the Apache License, Version 2.0, see LICENSE for details.
# SPDX-License-Identifier: Apache-2.0

from ctypes import c_int32, c_uint32
from functools import lru_cache
import subprocess
from typing import Optional

import architecture as arch
from architecture import reg_abi_name, LSU_SIZE_TO_FLOAT, LSU_SIZES, FLOAT_FMTS, CSR_NAMES
from architecture import ZERO_REG

# Below this absolute value: use signed int representation. Above: unsigned 32-bit hex
MAX_SIGNED_INT_LIT = 0xFFFF


def flt_decode(val: int, fmt: int) -> float:
    """Interprets the binary encoding of an integer as a FP value.

    Args:
        val: The integer encoding of the FP variable to decode.
        fmt: The floating point number format, as an index into the
            `FLOAT_FMTS` array.
    Returns:
        The floating point value represented by the input integer.
    """
    # get format and bit vector
    w_exp, w_mnt = FLOAT_FMTS[fmt]
    width = 1 + w_exp + w_mnt
    bitstr = '{:064b}'.format(val)[-width:]
    # print(bitstr)
    # Read bit vector slices
    sgn = -1.0 if bitstr[0] == '1' else 1.0
    mnt = int(bitstr[w_exp + 1:], 2)
    exp_unb = int(bitstr[1:w_exp + 1], 2)
    # derive base and exponent
    bse = int('1' + bitstr[w_exp + 1:], 2) / (2**w_mnt)
    exp_bias = -(2**(w_exp - 1) - 1)
    exp = exp_unb + exp_bias
    # case analysis
    if exp_unb == 2**w_exp - 1:
        return sgn * float('inf' if mnt == 0 else 'nan')
    elif exp_unb == 0 and mnt == 0:
        return sgn * 0.0
    elif exp_unb == 0:
        return float(sgn * mnt / (2**w_mnt) * (2**(exp_bias + 1)))
    else:
        return float(sgn * bse * (2**exp))


def flt_fmt(flt: float, width: int = 6) -> str:
    """Formats a floating-point number rounding to a certain decimal precision.

    Args:
        flt: The floating-point number to format.
        width: The number of significant decimal digits to round to.
    Returns:
        The formatted floating-point number as a string.
    """
    fmt = '{:.' + str(width) + '}'
    return fmt.format(flt)


def int_lit(
        num: int, size: int = 2, as_hex: Optional[bool] = None,
        prefix: Optional[bool] = True
) -> str:
    width = (8 * int(2**size))
    size_mask = (0x1 << width) - 1
    num = num & size_mask  # num is unsigned
    num_signed = c_int32(c_uint32(num).value).value
    hex_needed = (
        as_hex is True or
        (abs(num_signed) > MAX_SIGNED_INT_LIT and as_hex is not False)
    )
    if hex_needed:
        if prefix is True:
            return '0x{0:0{1}x}'.format(num, width // 4)
        else:
            return '{0:0{1}x}'.format(num, width // 4)
    else:
        return str(num_signed)


# TODO(colluca): align with int_lit arguments, so we can have a single "format_literal" function
def flt_lit(num: int, fmt: int, width: int = 6, vlen: int = 1) -> str:
    """Formats an integer encoding into a floating-point literal.

    Args:
        num: The integer encoding of the floating-point number(s).
        fmt: The floating point number format, as an index into the
            `FLOAT_FMTS` array.
        width: The number of significant decimal digits to round to.
        vlen: The number of floating-point numbers packed in the encoding,
            >1 for SIMD vectors.
    """
    # Divide the binary encoding into individual encodings for each number in the SIMD vector.
    bitwidth = 1 + FLOAT_FMTS[fmt][0] + FLOAT_FMTS[fmt][1]
    vec = [num >> (bitwidth * i) & (2**bitwidth - 1) for i in reversed(range(vlen))]
    # Format each individual float encoding to a string.
    floats = [flt_fmt(flt_decode(val, fmt), width) for val in vec]
    # Represent the encodings as a vector if SIMD.
    if len(floats) > 1:
        return '[{}]'.format(', '.join(floats))
    else:
        return floats[0]


@lru_cache
def disasm_inst(hex_inst, mc_exec='llvm-mc', mc_flags='-disassemble -mcpu=snitch --mattr=+v'):
    """Disassemble a single RISC-V instruction using llvm-mc."""
    # Reverse the endianness of the hex instruction
    inst_fmt = ' '.join(f'0x{byte:02x}' for byte in bytes.fromhex(hex_inst)[::-1])

    # Use llvm-mc to disassemble the binary instruction
    result = subprocess.run(
        [mc_exec] + mc_flags.split(),
        input=inst_fmt,
        capture_output=True,
        text=True,
        check=True,
    )

    # Extract disassembled instruction from llvm-mc output
    return result.stdout.splitlines()[-1].strip().replace('\t', ' ')


def format_mnemonic(extras, mc_exec):
    return disasm_inst(int_lit(extras['instr_data'], as_hex=True, prefix=False), mc_exec)


def format_pc(extras):
    return int_lit(extras['pc_q'], as_hex=True)


def format_insn(extras, mc_exec):
    return f"{format_pc(extras):<10} {format_mnemonic(extras, mc_exec):<26}"


def format_lsu_extras(extras):
    addr = int_lit(extras['lsu_addr'], as_hex=True)
    if (extras['lsu_is_store']):
        register = reg_abi_name(extras['rs2'], extras['lsu_is_float'])
        fmt = LSU_SIZE_TO_FLOAT[extras['lsu_size']]
        if extras['lsu_is_float']:
            value = flt_lit(extras['lsu_store_data'], fmt)
        else:
            value = int_lit(extras['lsu_store_data'])
        return (f"{register} = {value} ~~> "
                f"{LSU_SIZES[extras['lsu_size']]}[{addr}]")
    elif (extras['lsu_is_load']):
        register = reg_abi_name(extras['rd'], extras['lsu_is_float'])
        return f"{register} <~~ {LSU_SIZES[extras['lsu_size']]}[{addr}]"


def format_alu_extras(extras):
    comments = []
    rs1 = reg_abi_name(extras['rs1'])
    rs2 = reg_abi_name(extras['rs2'])
    opa = int_lit(extras['alu_opa'])
    opb = int_lit(extras['alu_opb'])
    if rs1 != ZERO_REG:
        comments.append(f'{rs1} = {opa}')
    if rs2 != ZERO_REG:
        comments.append(f'{rs2} = {opb}')
    return ', '.join(comments)


def format_fpu_extras(extras):
    flt_fmt = 1  # TODO: somehow get the flt format? for now assume always double
    comments = []
    rs1_is_fp = extras['rs1_is_fp']
    rs2_is_fp = extras['rs2_is_fp']
    rs1 = reg_abi_name(extras['rs1'], is_float=rs1_is_fp)
    rs2 = reg_abi_name(extras['rs2'], is_float=rs2_is_fp)
    opa = flt_lit(extras['fpu_opa'], flt_fmt) if rs1_is_fp else int_lit(extras['fpu_opa'])
    opb = flt_lit(extras['fpu_opb'], flt_fmt) if rs1_is_fp else int_lit(extras['fpu_opb'])
    comments.append(f'{rs1} = {opa}')
    comments.append(f'{rs2} = {opb}')
    return ', '.join(comments)


def format_csr_extras(extras):
    if extras['csr_addr'] in CSR_NAMES:
        csr_name = CSR_NAMES[extras['csr_addr']]
    else:
        addr_hex = int_lit(extras['csr_addr'], as_hex=True)
        csr_name = f"csr@{addr_hex}"
    return f"{csr_name} = {int_lit(extras['csr_write_data'])}"


def format_extras(extras):
    # Build extras string, made as a collection of "comments"
    comments = []

    # Extras string formatting depends on the instruction type
    fu_type = extras['fu_type']
    if fu_type == arch.FU_LSU:
        comments.append(format_lsu_extras(extras))
    elif fu_type == arch.FU_CSR:
        comments.append(format_csr_extras(extras))
    elif fu_type == arch.FU_ALU:
        comments.append(format_alu_extras(extras))
    elif fu_type == arch.FU_FPU:
        comments.append(format_fpu_extras(extras))
    elif fu_type not in arch.FU_TYPES:
        raise ValueError(f'Invalid FU type {fu_type}')

    # Additional comment if branch
    if extras.get('is_branch', False):
        comments.append("taken" if extras['branch_taken'] else "not taken")

    # Additional comment if jump
    if not extras['stall'] and (extras.get('pc_d', 4) != (extras.get('pc_q', 0) + 4)):
        comments.append(f"goto {int_lit(extras['pc_d'], as_hex=True)}")

    return comments
