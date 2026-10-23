"""
整理 Label Studio 导出的标注数据为 YOLOv8 标准数据集
- 三个类别都已是 YOLO txt，需要重映射类别 id
- 按时间索引每 5 张抽 1 张做验证集，其余训练
输出: yolo_dataset/{images,labels}/{train,val} + data.yaml
"""
import os
import shutil
from collections import Counter

SRC = "/home/kkka/桌面/数据标注"
DST = "yolo_dataset"

# 类别顺序：0 smoking, 1 safety_sign, 2 fire_extinguisher
CLASSES = ["smoking", "safety_sign", "fire_extinguisher"]


def reset_dir(path):
    if os.path.exists(path):
        shutil.rmtree(path)
    os.makedirs(path, exist_ok=True)


def split_of(stem):
    """按文件名尾部的 4 位帧号划分验证集：帧号 %5==0 进 val"""
    try:
        idx = int(stem.rsplit("-", 1)[-1])
    except ValueError:
        return "train"
    return "val" if idx % 5 == 0 else "train"


def convert_existing_yolo(project_dir, class_name, new_class_id):
    """已是 YOLO 格式的标签，重映射类别 id 后复制"""
    img_dir = os.path.join(project_dir, "images")
    lbl_dir = os.path.join(project_dir, "labels")
    saved = Counter()
    empty = 0
    bad = 0

    for base in sorted(os.listdir(img_dir)):
        if not base.lower().endswith((".jpg", ".jpeg", ".png")):
            continue
        stem = os.path.splitext(base)[0]
        split = split_of(stem)
        out_stem = f"{class_name}_{stem}"

        shutil.copy2(os.path.join(img_dir, base),
                     os.path.join(DST, "images", split, out_stem + ".jpg"))

        lbl_path = os.path.join(lbl_dir, stem + ".txt")
        lines = []
        if os.path.exists(lbl_path):
            for raw in open(lbl_path, encoding="utf-8"):
                parts = raw.strip().split()
                if not parts:
                    continue
                if len(parts) < 5:
                    bad += 1
                    continue
                try:
                    vals = [float(v) for v in parts[1:5]]
                except ValueError:
                    bad += 1
                    continue
                if not all(0.0 <= v <= 1.0 for v in vals[1:]):
                    bad += 1
                    continue
                if vals[2] <= 0.001 or vals[3] <= 0.001:
                    bad += 1
                    continue
                lines.append(f"{new_class_id} " + " ".join(f"{v:.6f}" for v in vals))
        else:
            print(f"  ⚠️ 标签缺失: {stem}")

        if not lines:
            empty += 1
        with open(os.path.join(DST, "labels", split, out_stem + ".txt"), "w") as f:
            f.write("\n".join(lines) + ("\n" if lines else ""))

        saved[split] += 1

    print(f"  {class_name}: train={saved['train']} val={saved['val']} 无框图片={empty} 非法框={bad}")


def main():
    global SRC, DST
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--src", default=SRC, help="Label Studio 导出目录")
    ap.add_argument("--dst", default=DST, help="输出 YOLO 数据集目录")
    a = ap.parse_args()
    SRC, DST = a.src, a.dst

    for sub in ["images/train", "images/val", "labels/train", "labels/val"]:
        reset_dir(os.path.join(DST, sub))

    print("▶ 整理 smoking (重映射为 0)")
    smk_project = os.path.join(
        SRC, "smoking", "project-1-at-2026-10-23-13-30-00d47287")
    convert_existing_yolo(smk_project, "smoking", 0)

    print("▶ 整理 safety_sign (重映射为 1)")
    sgn_project = os.path.join(
        SRC, "safety_sign", "project-3-at-2026-10-23-13-27-99140997")
    convert_existing_yolo(sgn_project, "safety_sign", 1)

    print("▶ 整理 fire_extinguisher (重映射为 2)")
    fe_project = os.path.join(
        SRC, "fire_extinguisher", "project-4-at-2026-10-22-05-06-1f557770")
    convert_existing_yolo(fe_project, "fire_extinguisher", 2)

    # data.yaml（不写 path，Ultralytics 会以 yaml 所在目录为数据集根，保证仓库可移植）
    yaml_path = os.path.join(DST, "data.yaml")
    with open(yaml_path, "w", encoding="utf-8") as f:
        f.write("train: images/train\n")
        f.write("val: images/val\n")
        f.write("names:\n")
        for i, n in enumerate(CLASSES):
            f.write(f"  {i}: {n}\n")

    print("\n" + "=" * 55)
    for split in ["train", "val"]:
        n_img = len(os.listdir(os.path.join(DST, "images", split)))
        n_lbl = len(os.listdir(os.path.join(DST, "labels", split)))
        print(f"{split}: 图片 {n_img} 张, 标签 {n_lbl} 个")
    print(f"data.yaml: {yaml_path}")
    print("=" * 55)


if __name__ == "__main__":
    main()
