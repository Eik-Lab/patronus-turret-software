import os

import yaml


def load_config(path):
    with open(path, "r") as f:
        config = yaml.safe_load(f)
    for section in ("dataset", "model", "training"):
        if section not in config:
            raise ValueError(f"config '{path}' is missing the '{section}' section")
    return config


def resolve(root, relative):
    """Join a dataset-relative path onto the dataset root."""
    return os.path.join(root, relative)
