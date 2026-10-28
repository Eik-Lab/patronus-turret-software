import os


def model_training_dir():
    return os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def generate_inference_artifacts(output_dir, id2label, image_size, model_name, config):
    labels_path = os.path.join(output_dir, "labels.txt")
    with open(labels_path, "w") as f:
        for i in range(len(id2label)):
            f.write(f"{id2label[i]}\n")

    model_name_clean = os.path.basename(output_dir)
    is_rfdetr = "rfdetr" in str(model_name).lower() or str(config.get("model", {}).get("type", "")).lower() == "rfdetr"
    if is_rfdetr:
        parser_lib = os.path.abspath(os.path.join(model_training_dir(), "deps", "deepstream-deformable-detr", "libdeepstream-deformable-detr.so"))
        config_content = f"""[property]
gpu-id=0
net-scale-factor=0.00392156862745098
offsets=0;0;0
model-color-format=0
onnx-file={os.path.abspath(os.path.join(output_dir, 'model.onnx'))}
model-engine-file={os.path.abspath(os.path.join(output_dir, 'model.onnx'))}_b1_gpu0_fp16.engine
batch-size=1
network-mode=2
num-detected-classes={len(id2label)}
labelfile-path={os.path.abspath(labels_path)}
network-input-order=0
infer-dims=3;{image_size};{image_size}
network-type=0
gie-unique-id=1
process-mode=1
maintain-aspect-ratio=0
symmetric-padding=0
parse-bbox-func-name=deepstream_deformable_detr_bbox
custom-lib-path={parser_lib}
cluster-mode=4
enable-dla=0

[class-attrs-all]
pre-cluster-threshold=0.5
"""
    else:
        parser_lib = os.path.abspath(os.path.join(model_training_dir(), "deps", "deepstream", "libnvdsinfer_custom_impl_Yolo.so"))
        config_content = f"""[property]
gpu-id=0
net-scale-factor=0.0039215697906911373
model-engine-file={os.path.abspath(os.path.join(output_dir, 'model.onnx'))}_b1_gpu0_fp16.engine
onnx-file={os.path.abspath(os.path.join(output_dir, 'model.onnx'))}
labelfile-path={os.path.abspath(labels_path)}
batch-size=1
network-mode=2
num-detected-classes={len(id2label)}
gie-unique-id=1
network-type=0
cluster-mode=2
infer-dims=3;{image_size};{image_size}
maintain-aspect-ratio=1
symmetric-padding=1
custom-lib-path={parser_lib}
parse-bbox-func-name=NvDsInferParseYolo
output-blob-names=output0

[class-attrs-all]
nms-iou-threshold=0.45
pre-cluster-threshold=0.25
topk=300
"""
    infer_path = os.path.join(output_dir, f"config_infer_primary_{model_name_clean}.txt")
    with open(infer_path, "w") as f:
        f.write(config_content)
