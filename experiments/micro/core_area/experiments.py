#!/usr/bin/env python3
# Copyright 2026 ETH Zurich and University of Bologna.
# Licensed under the Apache License, Version 2.0, see LICENSE for details.
# SPDX-License-Identifier: Apache-2.0

from snitch.util.experiments import experiment_utils as eu

EARLY_SYNTH_STAGE = '7'
FINAL_SYNTH_STAGE = '9'


class ExperimentManager(eu.ExperimentManager):

    def derive_axes(self, experiment):
        return eu.derive_axes_from_keys(experiment, keys=['name'])


def gen_experiments(designs=None):
    # IMPORTANT: HDL parameters should be listed in the same order they appear in the RTL
    experiments = [
        {
            'design': 'schnizo_synth',
            'name': 'default',
            'hdl_params': {
                'Xfrep': 1,
                'NofAlus': 3,
                'NofLsus': 3,
                'NofFpus': 1,
                'AluNofRss': 2,
                'LsuNofRss': 3,
                'FpuNofRss': 4,
                'AluNofConstants': 8,
                'LsuNofConstants': 8,
                'FpuNofConstants': 8,
                'MulInAlu0': 1,
                'AluNofResRspPorts': 2,
                'LsuNofResRspPorts': 2,
                'FpuNofResRspPorts': 2,
                'XF16': 1,
                'XF16ALT': 1,
                'XF8': 1,
                'XF8ALT': 1,
                'XFVEC': 1,
                'RVV': 1,
                'SpatzNofRss': 3,
                'NumSpatzFPUs': 4,
                'NumSpatzIPUs': 1,
            }
        }
    ]
    if designs is not None:
        experiments = [e for e in experiments if e['name'] in designs]
    return experiments


def results(dir=None):
    manager = ExperimentManager(gen_experiments(), dir=dir, parse_args=False)
    df = manager.get_results()
    df = df.set_index('name')
    df['synth_results'] = df['synth_results'].str[EARLY_SYNTH_STAGE]
    return df


def main():
    parser = ExperimentManager.parser()
    parser.add_argument('--designs', nargs='+')
    args = parser.parse_args()
    experiments = gen_experiments(designs=args.designs)
    manager = ExperimentManager(experiments=experiments, args=args, parse_args=False)

    manager.run()


if __name__ == '__main__':
    main()
