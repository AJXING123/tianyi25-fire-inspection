"""
视频帧提取脚本
从三个视频中各均匀提取100帧图片，分别存放
"""
import cv2
import os

# 视频配置：路径 → 类别名
VIDEO_CONFIG = [
    {
        "path": "/home/kkka/文档/xwechat_files/wxid_uaaqbs3l5l4a22_5c57/msg/video/2026-10/ef30fbc75d4934aac21e83f4066bf021.mp4",
        "label": "smoking",        # 吸烟
    },
    {
        "path": "/home/kkka/文档/xwechat_files/wxid_uaaqbs3l5l4a22_5c57/msg/video/2026-10/71bc5913e50dfba10266046fcc8351aa.mp4",
        "label": "fire_extinguisher",  # 灭火器
    },
    {
        "path": "/home/kkka/文档/xwechat_files/wxid_uaaqbs3l5l4a22_5c57/msg/video/2026-10/52aef0d3ff528c8d34a03b8d0d39568b.mp4",
        "label": "safety_sign",     # 安全指示牌
    },
]

OUTPUT_DIR = "dataset"  # 输出根目录（相对当前运行目录）
FRAMES_PER_VIDEO = 100                                     # 每个视频提取帧数


def extract_frames_uniformly(video_path: str, output_dir: str, num_frames: int):
    """从视频中均匀提取指定数量的帧"""
    os.makedirs(output_dir, exist_ok=True)

    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        print(f"  ❌ 无法打开视频: {video_path}")
        return 0

    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    fps = cap.get(cv2.CAP_PROP_FPS)
    duration = total_frames / fps if fps > 0 else 0
    print(f"  📹 总帧数: {total_frames}, FPS: {fps:.1f}, 时长: {duration:.1f}s")

    if total_frames <= 0:
        print(f"  ❌ 视频帧数异常")
        cap.release()
        return 0

    # 计算步长，均匀分布在整个视频时长上
    # 如果视频总帧数少于请求数，则取全部帧
    step = max(total_frames / num_frames, 1)

    saved = 0
    for i in range(num_frames):
        frame_idx = int(i * step)
        # 确保不越界
        if frame_idx >= total_frames:
            frame_idx = total_frames - 1

        cap.set(cv2.CAP_PROP_POS_FRAMES, frame_idx)
        ret, frame = cap.read()
        if not ret:
            # 读取失败，跳过这帧
            continue

        filename = f"{saved:04d}.jpg"
        filepath = os.path.join(output_dir, filename)
        cv2.imwrite(filepath, frame)
        saved += 1

    cap.release()
    return saved


def main():
    print("=" * 60)
    print("🎬 视频帧提取工具")
    print(f"输出目录: {OUTPUT_DIR}")
    print(f"每个视频提取: {FRAMES_PER_VIDEO} 帧")
    print("=" * 60)

    for config in VIDEO_CONFIG:
        label = config["label"]
        path = config["path"]
        out_dir = os.path.join(OUTPUT_DIR, label)

        print(f"\n▶ 处理: {label}")
        print(f"  视频: {os.path.basename(path)}")
        print(f"  输出: {out_dir}")

        count = extract_frames_uniformly(path, out_dir, FRAMES_PER_VIDEO)
        print(f"  ✅ 成功提取 {count} 帧")

    print("\n" + "=" * 60)
    print("🎉 全部完成！数据集结构如下：")
    for config in VIDEO_CONFIG:
        out_dir = os.path.join(OUTPUT_DIR, config["label"])
        actual = len([f for f in os.listdir(out_dir) if f.endswith(".jpg")]) if os.path.exists(out_dir) else 0
        print(f"  📁 {config['label']:25s} → {actual} 张")
    print("=" * 60)


if __name__ == "__main__":
    main()
