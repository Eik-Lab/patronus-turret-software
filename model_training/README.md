# Patronus model training

Train YOLO, RF-DETR, or D-FINE detection models from YAML/JSON configs. Training requires
CUDA; CPU and MPS are not supported.

```bash
uv sync
uv run python train.py train_configs/yolo11_p2.yaml
uv run python train_multiple.py train_configs/yolo11_p2.yaml train_configs/dfine_large.yaml
```

`train_multiple.py` runs configs sequentially. Both commands require CUDA and accept
`--force` to reuse an existing output directory. Without `--force`, occupied output
directories get timestamped names. Without CUDA, training stops before preparing data
or creating a run directory.

Configs live in `train_configs/` and contain `dataset`, `model`, and `training` sections.
Relative paths in configs are resolved from `model_training/`. Results and dataset checks
are written under `runs/<run>/`.

## Evaluate and monitor

To evaluate a trained model, first add one or more COCO dataset directories to `DATASETS`
at the top of `eval.py`, then run:

```bash
uv run python eval.py --model_path runs/my_run/model.onnx
```

View training logs in TensorBoard (log locations vary by model backend):

```bash
uv run tensorboard --logdir runs
```
