"""Test helpers: the synthetic classroom map folders (autosdv_lab3.fixtures) in a tmp dir."""

import os

from autosdv_lab3.fixtures import VARIANTS, write_map

__all__ = ['VARIANTS', 'make_map']


def make_map(tmp_path, variant='good'):
    """Write the `variant` map folder under tmp_path; returns its path as str."""
    return write_map(os.path.join(str(tmp_path), variant), variant)
