from functools import partial
from pathlib import Path

import torch
from transformers import EarlyStoppingCallback, Trainer, TrainingArguments

from metrics import compute_metrics
from src.data.dataset import build_datasets
from src.samples import SampleGridCallback
from utils.filter_dataset import filter_coco_dataset


class Model:
    name = "model"
    coco_dataset = True

    def filter(self, config, output_dir):
        pass

    def train(self, config, train_stats):
        raise NotImplementedError


class HFDetModel(Model):
    model_class = None

    def filter(self, config, output_dir):
        filter_coco_dataset(config, output_dir)

    def train(self, config, train_stats):
        model_cfg = config["model"]
        train_cfg = config["training"]
        Path(train_cfg["tensorboard_dir"]).mkdir(parents=True, exist_ok=True)

        image_processor, train_dataset, val_dataset = build_datasets(config)

        categories = train_stats["class_distribution"]["categories"]
        id2label = {i: name for i, (_, name) in enumerate(sorted(categories.items()))}
        label2id = {name: i for i, name in id2label.items()}

        model = self.model_class.from_pretrained(
            model_cfg["name"],
            id2label=id2label,
            label2id=label2id,
            ignore_mismatched_sizes=True,
        )

        def collate_fn(batch):
            pixel_values = [item["pixel_values"] for item in batch]
            encoding = image_processor.pad(pixel_values, return_tensors="pt")
            return {
                "pixel_values": encoding["pixel_values"],
                "pixel_mask": encoding["pixel_mask"],
                "labels": [item["labels"] for item in batch],
            }

        # Prefer bf16 on capable GPUs (more stable for transformers than fp16).
        use_bf16 = torch.cuda.is_available() and torch.cuda.is_bf16_supported()
        early_stopping_patience = train_cfg.get("early_stopping_patience", 0)

        args = TrainingArguments(
            output_dir=train_cfg["output_dir"],
            logging_dir=train_cfg["tensorboard_dir"],
            report_to=["tensorboard"],
            num_train_epochs=train_cfg["epochs"],
            per_device_train_batch_size=train_cfg["batch_size"],
            per_device_eval_batch_size=train_cfg["batch_size"],
            learning_rate=float(train_cfg["learning_rate"]),
            weight_decay=float(train_cfg["weight_decay"]),
            gradient_accumulation_steps=train_cfg.get("gradient_accumulation_steps", 1),
            warmup_ratio=train_cfg.get("warmup_ratio", 0.0),
            lr_scheduler_type=train_cfg.get("lr_scheduler_type", "linear"),
            max_grad_norm=train_cfg.get("max_grad_norm", 1.0),
            optim=train_cfg.get("optimizer", "adamw_torch"),
            dataloader_num_workers=train_cfg["num_workers"],
            logging_steps=train_cfg["logging_steps"],
            save_total_limit=train_cfg["save_total_limit"],
            seed=train_cfg["seed"],
            data_seed=train_cfg["seed"],
            full_determinism=train_cfg.get("deterministic", False),
            eval_strategy="epoch",
            save_strategy="epoch",
            eval_do_concat_batches=False,
            bf16=use_bf16,
            fp16=torch.cuda.is_available() and not use_bf16,
            gradient_checkpointing=train_cfg.get("gradient_checkpointing", False),
            gradient_checkpointing_kwargs={"use_reentrant": False},
            load_best_model_at_end=True,
            metric_for_best_model="map",
            greater_is_better=True,
            remove_unused_columns=False,
        )

        trainer = Trainer(
            model=model,
            args=args,
            train_dataset=train_dataset,
            eval_dataset=val_dataset,
            data_collator=collate_fn,
            processing_class=image_processor,
            compute_metrics=partial(
                compute_metrics,
                image_processor=image_processor,
                id2label=id2label,
            ),
        )

        trainer.add_callback(
            SampleGridCallback(
                image_processor=image_processor,
                val_dataset=val_dataset,
                output_dir=train_cfg["output_dir"],
                logging_dir=train_cfg["tensorboard_dir"],
                id2label=id2label,
            )
        )
        if early_stopping_patience > 0:
            trainer.add_callback(
                EarlyStoppingCallback(early_stopping_patience=early_stopping_patience)
            )

        print(f"\n=== Training {model_cfg['name']} ({self.name}) ===")
        print(f"  TensorBoard: tensorboard --logdir {train_cfg['tensorboard_dir']}")
        trainer.train()
        trainer.save_model(train_cfg["output_dir"])
        image_processor.save_pretrained(train_cfg["output_dir"])
        print(f"  model saved to {train_cfg['output_dir']}")

        metrics = trainer.evaluate()
        return {key[len("eval_") :]: value for key, value in metrics.items()}
