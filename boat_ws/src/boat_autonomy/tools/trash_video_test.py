#!/usr/bin/env python3
"""GPU-only streaming video validation for the shared trash TargetFilter."""

from __future__ import annotations

import argparse
import json
import statistics
import subprocess
import sys
import time
from pathlib import Path

import cv2

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from boat_autonomy.target_filter import (  # noqa: E402
    TargetFilter,
    draw_filter_overlay,
    parse_yolo_detections,
)


def percentile(values, fraction):
    """Return a linearly interpolated percentile from numeric values."""
    ordered = sorted(values)
    position = (len(ordered) - 1) * fraction
    lower = int(position)
    upper = min(lower + 1, len(ordered) - 1)
    weight = position - lower
    return ordered[lower] * (1.0 - weight) + ordered[upper] * weight


def require_orin_cuda(torch):
    """Abort unless CUDA device zero is an available Orin GPU."""
    available = torch.cuda.is_available()
    print(f'CUDA available: {available}', flush=True)
    if not available:
        raise RuntimeError('CUDA is unavailable; CPU fallback is forbidden')
    gpu = torch.cuda.get_device_name(0)
    print(f'GPU: {gpu}', flush=True)
    print('device: cuda:0', flush=True)
    if 'orin' not in gpu.lower():
        raise RuntimeError(f'GPU is not Orin: {gpu}')


def transcode_h264(temporary, output):
    """Convert the temporary streaming output to Windows-friendly H.264."""
    subprocess.run([
        'ffmpeg', '-y', '-loglevel', 'error', '-i', str(temporary),
        '-c:v', 'libx264', '-preset', 'veryfast', '-pix_fmt', 'yuv420p',
        '-movflags', '+faststart', '-an', str(output),
    ], check=True)
    temporary.unlink()


def process_video(model, source, output, torch):
    """Stream one video through CUDA inference and the shared filter."""
    capture = cv2.VideoCapture(str(source))
    if not capture.isOpened():
        raise RuntimeError(f'Cannot open video: {source}')
    width = int(capture.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = float(capture.get(cv2.CAP_PROP_FPS))
    expected_frames = int(capture.get(cv2.CAP_PROP_FRAME_COUNT))
    temporary = output.with_name(output.stem + '_temporary.mp4')
    writer = cv2.VideoWriter(
        str(temporary), cv2.VideoWriter_fourcc(*'mp4v'), fps,
        (width, height)
    )
    if not writer.isOpened():
        capture.release()
        raise RuntimeError(f'Cannot open temporary writer: {temporary}')

    target_filter = TargetFilter(logger=lambda message: print(message, flush=True))
    inference_times = []
    raw_detection_frames = 0
    confirmed_target_frames = 0
    filtered_raw_frames = 0
    frame_count = 0
    timeline_start = time.monotonic()
    processing_start = time.monotonic()
    gpu_verified = False
    try:
        while True:
            success, frame = capture.read()
            if not success:
                break
            frame_count += 1
            results = model.track(
                source=frame, conf=0.30, device=0, imgsz=640,
                persist=True, verbose=False
            )
            result = results[0]
            if not gpu_verified:
                predictor_device = str(model.predictor.model.device)
                if not predictor_device.startswith('cuda'):
                    raise RuntimeError(
                        f'CPU fallback detected: {predictor_device}'
                    )
                gpu_verified = True
                print(f'Inference device verified: {predictor_device}', flush=True)
            if frame_count % 300 == 0 and not torch.cuda.is_available():
                raise RuntimeError('CUDA became unavailable during inference')
            inference_times.append(float(result.speed['inference']))
            detections = parse_yolo_detections(
                result, model.names, set(), width, height
            )
            if detections:
                raw_detection_frames += 1
            video_now = timeline_start + (frame_count - 1) / fps
            filtered = target_filter.update(detections, video_now)
            if filtered.target is not None:
                confirmed_target_frames += 1
            elif detections:
                filtered_raw_frames += 1
            writer.write(draw_filter_overlay(frame, detections, filtered))
            del results, result, detections, filtered, frame
            if frame_count % 300 == 0:
                print(
                    f'{source.name}: {frame_count}/{expected_frames}',
                    flush=True
                )
    finally:
        capture.release()
        writer.release()
    processing_seconds = time.monotonic() - processing_start
    transcode_h264(temporary, output)
    metrics = {
        'source': str(source),
        'output': str(output),
        'width': width,
        'height': height,
        'fps': fps,
        'frames': frame_count,
        'processing_seconds': processing_seconds,
        'processing_fps': frame_count / processing_seconds,
        'inference_mean_ms': statistics.fmean(inference_times),
        'inference_median_ms': statistics.median(inference_times),
        'inference_p95_ms': percentile(inference_times, 0.95),
        'inference_max_ms': max(inference_times),
        'raw_detection_frames': raw_detection_frames,
        'confirmed_target_frames': confirmed_target_frames,
        'raw_detection_frames_filtered': filtered_raw_frames,
        'target_lock_events': target_filter.lock_count,
        'target_lost_events': target_filter.lost_count,
        'cuda_available': torch.cuda.is_available(),
        'gpu': torch.cuda.get_device_name(0),
        'device': str(model.predictor.model.device),
    }
    metrics_path = output.with_suffix('.json')
    metrics_path.write_text(
        json.dumps(metrics, indent=2, ensure_ascii=False) + '\n',
        encoding='utf-8'
    )
    print(json.dumps(metrics, indent=2), flush=True)
    return metrics


def main():
    import torch
    from ultralytics import YOLO

    parser = argparse.ArgumentParser()
    parser.add_argument('--model', type=Path, required=True)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('videos', type=Path, nargs='+')
    args = parser.parse_args()
    require_orin_cuda(torch)
    args.output_dir.mkdir(parents=True, exist_ok=True)
    model = YOLO(str(args.model))
    print(f'Model classes: {model.names}', flush=True)
    for source in args.videos:
        output = args.output_dir / f'{source.stem}_filtered.mp4'
        process_video(model, source, output, torch)
        if not output.is_file():
            raise RuntimeError(f'Output was not created: {output}')


if __name__ == '__main__':
    main()
