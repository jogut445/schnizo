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


def gen_experiments():
    experiments = [
        {
            'design': 'schnizo_fu_stage_synth',
            'name': 'default',
            'hdl_params': {
                'Xfrep': 1,
                'MulInAlu0': 1,
                'NofAlus': 3,
                'NofLsus': 3,
                'NofFpus': 1,
                'AluNofRss': 2,
                'LsuNofRss': 3,
                'FpuNofRss': 4,
                'AluNofConstants': 8,
                'LsuNofConstants': 8,
                'FpuNofConstants': 8,
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
    return experiments


def get_results():
    manager = ExperimentManager(gen_experiments())
    df = manager.get_results()
    df['synth_results'] = df['synth_results'].str[FINAL_SYNTH_STAGE]
    return df


def main():
    experiments = gen_experiments()
    manager = ExperimentManager(experiments=experiments)

    manager.run()


if __name__ == '__main__':
    main()
