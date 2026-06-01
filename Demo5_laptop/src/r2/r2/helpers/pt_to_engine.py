"""
ABU Robocon 2026 — YOLO .pt → TensorRT .engine Converter
==========================================================
Converts one or more Ultralytics YOLO .pt weight files to TensorRT .engine
format using the built-in ultralytics export pipeline.

Supported precision modes:
  fp32  — full float32 (largest, most accurate)
  fp16  — half precision (recommended for Jetson / RTX GPUs)
  int8  — 8-bit quantisation (fastest, needs calibration data)

Usage examples:
  # Single model, FP16
  python3 pt_to_engine.py --weights best.pt --half

  # Multiple models, FP32, custom image size
  python3 pt_to_engine.py --weights main.pt auth.pt --imgsz 640

  # INT8 with calibration images
  python3 pt_to_engine.py --weights best.pt --int8 --data /path/to/data.yaml

  # Batch size > 1 (for throughput benchmarking only — ROS nodes use batch=1)
  python3 pt_to_engine.py --weights best.pt --half --batch 4
"""

import argparse
import sys
import time
from pathlib import Path


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="Convert YOLO .pt weights to TensorRT .engine",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    p.add_argument(
        "--weights", "-w",
        nargs="+",
        required=True,
        metavar="FILE",
        help="One or more .pt weight files to convert",
    )
    p.add_argument(
        "--imgsz", "-s",
        type=int,
        default=640,
        metavar="N",
        help="Square inference image size (default: 640)",
    )
    p.add_argument(
        "--batch",
        type=int,
        default=1,
        metavar="N",
        help="TensorRT batch size baked into the engine (default: 1)",
    )
    p.add_argument(
        "--half",
        action="store_true",
        help="Export in FP16 precision (recommended)",
    )
    p.add_argument(
        "--int8",
        action="store_true",
        help="Export in INT8 precision (requires --data for calibration)",
    )
    p.add_argument(
        "--data",
        type=str,
        default=None,
        metavar="YAML",
        help="Dataset YAML path — required when using --int8",
    )
    p.add_argument(
        "--workspace",
        type=int,
        default=4,
        metavar="GB",
        help="TensorRT builder workspace size in GB (default: 4)",
    )
    p.add_argument(
        "--device",
        type=str,
        default="0",
        metavar="ID",
        help="CUDA device id (default: 0)",
    )
    p.add_argument(
        "--dynamic",
        action="store_true",
        help="Export with dynamic input shapes (variable batch/size at runtime)",
    )
    p.add_argument(
        "--simplify",
        action="store_true",
        default=True,
        help="Run ONNX simplifier before TRT build (default: True)",
    )
    p.add_argument(
        "--outdir", "-o",
        type=str,
        default=None,
        metavar="DIR",
        help="Output directory (default: same folder as each .pt file)",
    )
    return p.parse_args()


def resolve_weights(paths: list[str]) -> list[Path]:
    resolved = []
    for p in paths:
        path = Path(p).expanduser().resolve()
        if not path.exists():
            print(f"[ERROR] Weight file not found: {path}", file=sys.stderr)
            sys.exit(1)
        if path.suffix != ".pt":
            print(f"[ERROR] Expected a .pt file, got: {path}", file=sys.stderr)
            sys.exit(1)
        resolved.append(path)
    return resolved


def _patch_missing_losses() -> None:
    """Stub out loss classes removed from ultralytics so old checkpoints can be loaded for export."""
    import torch.nn as nn
    import ultralytics.utils.loss as loss_mod

    for cls_name in ("E2ELoss", "v10DetectLoss"):
        if not hasattr(loss_mod, cls_name):
            stub = type(cls_name, (nn.Module,), {"forward": lambda *_: None})
            setattr(loss_mod, cls_name, stub)


def convert(weight: Path, args: argparse.Namespace) -> Path | None:
    _patch_missing_losses()
    from ultralytics import YOLO  # imported here so --help works without torch

    print(f"\n{'='*60}")
    print(f"  Converting : {weight.name}")
    print(f"  Image size : {args.imgsz}x{args.imgsz}")
    print(f"  Batch size : {args.batch}")
    precision = "INT8" if args.int8 else ("FP16" if args.half else "FP32")
    print(f"  Precision  : {precision}")
    print(f"  Device     : cuda:{args.device}")
    print(f"{'='*60}")

    model = YOLO(str(weight), task="detect")

    export_kwargs = dict(
        format    = "engine",
        imgsz     = args.imgsz,
        batch     = args.batch,
        half      = args.half and not args.int8,
        int8      = args.int8,
        data      = args.data,
        workspace = args.workspace,
        device    = args.device,
        dynamic   = args.dynamic,
        simplify  = args.simplify,
        verbose   = False,
    )

    t0 = time.perf_counter()
    exported = model.export(**export_kwargs)
    elapsed = time.perf_counter() - t0

    engine_path = Path(exported) if exported else weight.with_suffix(".engine")

    if args.outdir:
        out_dir = Path(args.outdir).expanduser().resolve()
        out_dir.mkdir(parents=True, exist_ok=True)
        dest = out_dir / engine_path.name
        engine_path.rename(dest)
        engine_path = dest

    if engine_path.exists():
        size_mb = engine_path.stat().st_size / 1_048_576
        print(f"\n  Done in {elapsed:.1f}s")
        print(f"  Engine   : {engine_path}")
        print(f"  Size     : {size_mb:.1f} MB")
        return engine_path
    else:
        print(f"\n[ERROR] Export finished but engine file not found: {engine_path}", file=sys.stderr)
        return None


def main() -> None:
    args = parse_args()

    if args.int8 and args.data is None:
        print("[WARN] INT8 calibration works best with --data pointing to a dataset YAML.")

    weights = resolve_weights(args.weights)
    succeeded, failed = [], []

    for w in weights:
        result = convert(w, args)
        (succeeded if result else failed).append(w)

    print(f"\n{'='*60}")
    print(f"  Results: {len(succeeded)}/{len(weights)} converted successfully")
    for w in succeeded:
        print(f"    OK  {w.name}")
    for w in failed:
        print(f"    ERR {w.name}")
    print(f"{'='*60}\n")

    if failed:
        sys.exit(1)


if __name__ == "__main__":
    main()
