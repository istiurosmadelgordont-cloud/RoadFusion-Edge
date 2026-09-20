"""Distill UFLDv2 CULane 1600x320 features into an 800x320 student.

The expensive 91224-way output layer is copied and frozen.  Distillation
matches the 2048-dimensional hidden representation immediately before that
layer, which is both memory efficient and preserves all four official heads.
"""

from __future__ import annotations

import argparse
import json
import random
import sys
from pathlib import Path

import cv2
import numpy as np
import torch
from torch import nn
from torch.utils.data import DataLoader, Dataset


MEAN = np.asarray([0.485, 0.456, 0.406], np.float32)
STD = np.asarray([0.229, 0.224, 0.225], np.float32)


class FeatureEncoder(nn.Module):
    def __init__(self, backbone_factory, width: int):
        super().__init__()
        self.width = width
        self.input_dim = 320 // 32 * width // 32 * 8
        self.model = backbone_factory("18", pretrained=False)
        self.pool = nn.Conv2d(512, 8, 1)
        self.norm = nn.LayerNorm(self.input_dim)
        self.fc1 = nn.Linear(self.input_dim, 2048)
        self.relu = nn.ReLU()

    def forward(self, image):
        _x2, _x3, feature = self.model(image)
        feature = self.pool(feature).flatten(1)
        return self.relu(self.fc1(self.norm(feature)))


def load_checkpoint(path: Path):
    checkpoint = torch.load(path, map_location="cpu", weights_only=False)
    source = checkpoint["model"] if isinstance(checkpoint, dict) and "model" in checkpoint else checkpoint
    return {name.removeprefix("module."): value for name, value in source.items()}


def encoder_state(state):
    result = {}
    for name, value in state.items():
        if name.startswith("model.") or name.startswith("pool."):
            result[name] = value
        elif name.startswith("cls.0."):
            result["norm." + name[len("cls.0."):]] = value
        elif name.startswith("cls.1."):
            result["fc1." + name[len("cls.1."):]] = value
    return result


def merge_encoder_state(base, trained):
    for name, value in trained.items():
        if name.startswith("model.") or name.startswith("pool."):
            base[name] = value.cpu()
        elif name.startswith("norm."):
            base["cls.0." + name[len("norm."):]] = value.cpu()
        elif name.startswith("fc1."):
            base["cls.1." + name[len("fc1."):]] = value.cpu()


def normalized_tensor(bgr):
    rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
    rgb = (rgb - MEAN) / STD
    return torch.from_numpy(np.ascontiguousarray(rgb.transpose(2, 0, 1)))


def discover_videos(root: Path, maximum: int):
    excluded = {"66b5fa4b", "e716f3ed", "050541b1", "668b1ac0", "ddfc4b8d"}
    videos = []
    for path in sorted(root.rglob("*.camera_front_wide_120fov.mp4")):
        if any(token in path.name for token in excluded):
            continue
        videos.append(path)
        if len(videos) >= maximum:
            break
    return videos


def prepare(args, backbone_factory):
    images_dir = args.cache / "images"
    images_dir.mkdir(parents=True, exist_ok=True)
    teacher_state = load_checkpoint(args.teacher)
    teacher = FeatureEncoder(backbone_factory, 1600)
    teacher.load_state_dict(encoder_state(teacher_state), strict=True)
    teacher.eval().half().cuda()
    videos = discover_videos(args.video_root, args.max_videos)
    if not videos:
        raise RuntimeError("no NVIDIA front-wide videos found")
    targets, sources = [], []
    index = 0
    with torch.inference_mode():
        for video in videos:
            capture = cv2.VideoCapture(str(video))
            total = max(1, int(capture.get(cv2.CAP_PROP_FRAME_COUNT)))
            stride = max(1, total // args.frames_per_video)
            selected = 0
            frame_index = 0
            batch, batch_frames = [], []
            while selected < args.frames_per_video:
                ok, frame = capture.read()
                if not ok:
                    break
                if frame_index % stride == 0:
                    teacher_image = cv2.resize(frame, (1600, 533), interpolation=cv2.INTER_LINEAR)[-320:]
                    student_image = cv2.resize(frame, (800, 533), interpolation=cv2.INTER_LINEAR)[-320:]
                    batch.append(normalized_tensor(teacher_image))
                    batch_frames.append(student_image)
                    selected += 1
                frame_index += 1
                if len(batch) == args.teacher_batch or selected == args.frames_per_video:
                    if batch:
                        hidden = teacher(torch.stack(batch).cuda().half()).float().cpu().numpy().astype(np.float16)
                        for image, target in zip(batch_frames, hidden):
                            name = f"{index:06d}.jpg"
                            cv2.imwrite(str(images_dir / name), image, [cv2.IMWRITE_JPEG_QUALITY, 94])
                            targets.append(target)
                            sources.append({"image": name, "video": video.name, "frame": frame_index})
                            index += 1
                    batch, batch_frames = [], []
            capture.release()
            print(f"prepared {selected:3d} from {video.name}")
    np.save(args.cache / "targets.npy", np.asarray(targets, np.float16))
    (args.cache / "samples.json").write_text(json.dumps(sources, ensure_ascii=False, indent=2), encoding="utf-8")
    print(f"prepared_total={len(targets)} cache={args.cache}")


class DistillDataset(Dataset):
    def __init__(self, cache: Path):
        self.images = sorted((cache / "images").glob("*.jpg"))
        self.targets = np.load(cache / "targets.npy", mmap_mode="r")
        if len(self.images) != len(self.targets):
            raise RuntimeError("image/target count mismatch")

    def __len__(self):
        return len(self.images)

    def __getitem__(self, index):
        image = cv2.imread(str(self.images[index]))
        return normalized_tensor(image), torch.from_numpy(np.asarray(self.targets[index], np.float32))


def train(args, backbone_factory):
    random.seed(7)
    torch.manual_seed(7)
    base = load_checkpoint(args.student_init)
    student = FeatureEncoder(backbone_factory, 800)
    student.load_state_dict(encoder_state(base), strict=True)
    student.cuda().train()
    dataset = DistillDataset(args.cache)
    validation_count = max(32, len(dataset) // 10)
    generator = torch.Generator().manual_seed(7)
    training, validation = torch.utils.data.random_split(
        dataset, [len(dataset) - validation_count, validation_count], generator=generator)
    train_loader = DataLoader(training, batch_size=args.batch, shuffle=True,
                              num_workers=0, pin_memory=True)
    val_loader = DataLoader(validation, batch_size=args.batch, shuffle=False,
                            num_workers=0, pin_memory=True)
    optimizer = torch.optim.SGD(student.parameters(), lr=args.lr, momentum=0.9,
                                weight_decay=1e-5, nesterov=True)
    scaler = torch.amp.GradScaler("cuda")
    best = float("inf")
    best_state = None
    for epoch in range(args.epochs):
        student.train()
        total = count = 0
        for images, targets in train_loader:
            images = images.cuda(non_blocking=True)
            targets = targets.cuda(non_blocking=True)
            optimizer.zero_grad(set_to_none=True)
            with torch.amp.autocast("cuda", dtype=torch.float16):
                prediction = student(images)
                loss = torch.nn.functional.smooth_l1_loss(prediction, targets, beta=0.2)
            scaler.scale(loss).backward()
            scaler.unscale_(optimizer)
            torch.nn.utils.clip_grad_norm_(student.parameters(), 5.0)
            scaler.step(optimizer)
            scaler.update()
            total += float(loss) * images.shape[0]
            count += images.shape[0]
        student.eval()
        val_total = val_count = 0
        with torch.inference_mode():
            for images, targets in val_loader:
                with torch.amp.autocast("cuda", dtype=torch.float16):
                    loss = torch.nn.functional.smooth_l1_loss(
                        student(images.cuda()), targets.cuda(), beta=0.2)
                val_total += float(loss) * images.shape[0]
                val_count += images.shape[0]
        train_loss = total / max(count, 1)
        val_loss = val_total / max(val_count, 1)
        print(f"epoch={epoch + 1}/{args.epochs} train={train_loss:.6f} val={val_loss:.6f}")
        if val_loss < best:
            best = val_loss
            best_state = {name: value.detach().cpu().clone()
                          for name, value in student.state_dict().items()}
    merge_encoder_state(base, best_state)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    torch.save({"model": base, "student": {"input_width": 800, "input_height": 320,
               "teacher": str(args.teacher), "distill_samples": len(dataset),
               "best_validation_loss": best}}, args.output)
    print(f"saved={args.output.resolve()} best_val={best:.6f}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", choices=("prepare", "train", "all"), default="all")
    parser.add_argument("--upstream", type=Path, required=True)
    parser.add_argument("--teacher", type=Path, required=True)
    parser.add_argument("--student-init", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--video-root", type=Path, required=True)
    parser.add_argument("--cache", type=Path, required=True)
    parser.add_argument("--max-videos", type=int, default=12)
    parser.add_argument("--frames-per-video", type=int, default=60)
    parser.add_argument("--teacher-batch", type=int, default=2)
    parser.add_argument("--batch", type=int, default=8)
    parser.add_argument("--epochs", type=int, default=8)
    parser.add_argument("--lr", type=float, default=0.002)
    args = parser.parse_args()
    sys.path.insert(0, str(args.upstream.resolve()))
    from model.backbone import resnet
    args.cache.mkdir(parents=True, exist_ok=True)
    if args.mode in ("prepare", "all"):
        prepare(args, resnet)
    if args.mode in ("train", "all"):
        train(args, resnet)


if __name__ == "__main__":
    main()
