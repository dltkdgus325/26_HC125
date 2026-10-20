"""Shared temporal and spatial filtering for YOLO trash detections."""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass
from typing import Callable, Iterable, Optional

import cv2


@dataclass
class TrashTarget:
    """Represent one YOLO detection in image coordinates."""

    detected: bool = False
    class_id: int = -1
    confidence: float = 0.0
    center_x: float = 0.0
    center_y: float = 0.0
    bbox_width: float = 0.0
    bbox_height: float = 0.0
    image_width: int = 0
    image_height: int = 0
    track_id: int = -1

    @property
    def bbox_area(self):
        """Return bounding-box area in pixels squared."""
        return self.bbox_width * self.bbox_height


@dataclass
class FilterResult:
    """Describe the filter output for one frame."""

    state: str
    target: Optional[TrashTarget]
    candidate: Optional[TrashTarget]
    hard_negative: bool = False


def parse_yolo_detections(result, names, target_classes, width, height,
                          warning: Optional[Callable[[str], None]] = None):
    """Convert an Ultralytics result to validated TrashTarget objects."""
    detections = []
    if result is None or result.boxes is None:
        return detections
    for box in result.boxes:
        try:
            class_id = int(box.cls.item())
            confidence = float(box.conf.item())
            if target_classes and str(names[class_id]) not in target_classes:
                continue
            cx, cy, bw, bh = map(float, box.xywh[0].tolist())
            if not all(math.isfinite(value) for value in
                       (confidence, cx, cy, bw, bh)):
                continue
            track_id = -1
            if getattr(box, 'id', None) is not None:
                try:
                    track_id = int(box.id.item())
                except (AttributeError, RuntimeError, TypeError, ValueError):
                    pass
            detections.append(TrashTarget(
                True, class_id, confidence, cx, cy, bw, bh,
                width, height, track_id
            ))
        except (AttributeError, IndexError, KeyError, RuntimeError,
                TypeError, ValueError) as exc:
            if warning is not None:
                warning(f'YOLO box parse failed: {exc}')
    return detections


class TargetFilter:
    """Confirm new targets and preserve a spatially continuous lock."""

    NONE = 'NONE'
    CANDIDATE = 'CANDIDATE'
    LOCKED = 'LOCKED'
    TEMP_LOST = 'TEMP_LOST'

    def __init__(self, target_entry_conf=0.50, target_keep_conf=0.30,
                 confirm_window=5, confirm_count=3,
                 target_lost_timeout=0.50, max_target_jump_ratio=0.20,
                 logger: Optional[Callable[[str], None]] = None):
        self.target_entry_conf = float(target_entry_conf)
        self.target_keep_conf = float(target_keep_conf)
        self.confirm_window = int(confirm_window)
        self.confirm_count = int(confirm_count)
        self.target_lost_timeout = float(target_lost_timeout)
        self.max_target_jump_ratio = float(max_target_jump_ratio)
        self.logger = logger or (lambda message: None)
        self.confirm_history = deque(maxlen=self.confirm_window)
        self.target_locked = False
        self.locked_track_id = -1
        self.locked_target = None
        self.candidate_target = None
        self.last_valid_time = None
        self.state = self.NONE
        self.lock_count = 0
        self.lost_count = 0

    def update(self, detections: Iterable[TrashTarget], now: float):
        """Process one frame of raw detections."""
        detections = list(detections)
        if self.target_locked:
            return self._update_locked(detections, now)
        return self._update_candidate(detections, now)

    def _update_candidate(self, detections, now):
        previous_had_candidate = self.candidate_target is not None
        candidate = self._select_candidate(detections)
        entry_hit = (
            candidate is not None
            and candidate.confidence >= self.target_entry_conf
        )
        if candidate is not None and self.candidate_target is not None:
            if not self._same_target(self.candidate_target, candidate):
                self.confirm_history.clear()
        self.confirm_history.append(entry_hit)

        if candidate is not None:
            if not previous_had_candidate:
                self.logger(
                    f'[TRASH] candidate detected conf={candidate.confidence:.2f}'
                )
            self.candidate_target = candidate
            self.state = self.CANDIDATE
            if entry_hit and sum(self.confirm_history) >= self.confirm_count:
                self.target_locked = True
                self.locked_target = candidate
                self.locked_track_id = candidate.track_id
                self.last_valid_time = now
                self.state = self.LOCKED
                self.lock_count += 1
                self.logger(
                    '[TRASH] TARGET LOCKED '
                    f'track_id={candidate.track_id} '
                    f'conf={candidate.confidence:.2f}'
                )
                return FilterResult(self.state, candidate, candidate)
            return FilterResult(self.state, None, candidate)

        hard_negative = (
            previous_had_candidate and not any(self.confirm_history)
        )
        if hard_negative:
            self.candidate_target = None
        self.state = self.NONE
        return FilterResult(self.state, None, None, hard_negative)

    def _update_locked(self, detections, now):
        target = self._select_locked(detections)
        if target is not None and target.confidence >= self.target_keep_conf:
            self.locked_target = target
            if target.track_id >= 0:
                self.locked_track_id = target.track_id
            self.last_valid_time = now
            self.state = self.LOCKED
            return FilterResult(self.state, target, None)

        elapsed = now - self.last_valid_time
        if self.state != self.TEMP_LOST:
            self.logger('[TRASH] target temporarily lost')
        if elapsed < self.target_lost_timeout:
            self.state = self.TEMP_LOST
            return FilterResult(self.state, self.locked_target, None)

        self.logger(
            f'[TRASH] TARGET LOST timeout={self.target_lost_timeout:.2f}s'
        )
        self.target_locked = False
        self.locked_track_id = -1
        self.locked_target = None
        self.candidate_target = None
        self.last_valid_time = None
        self.confirm_history.clear()
        self.state = self.NONE
        self.lost_count += 1
        return FilterResult(self.state, None, None)

    def _select_candidate(self, detections):
        if not detections:
            return None
        if self.candidate_target is not None:
            matches = [item for item in detections
                       if self._same_target(self.candidate_target, item)]
            if matches:
                return min(matches, key=lambda item: self._distance(
                    self.candidate_target, item
                ))
        return max(detections, key=lambda item: (
            item.bbox_area, item.confidence
        ))

    def _select_locked(self, detections):
        if not detections:
            return None
        if self.locked_track_id >= 0:
            same_id = [item for item in detections
                       if (item.track_id == self.locked_track_id
                           and self._distance(self.locked_target, item)
                           <= self.max_target_jump_ratio)]
            if same_id:
                return max(same_id, key=lambda item: item.confidence)
        nearby = [item for item in detections
                  if self._distance(self.locked_target, item)
                  <= self.max_target_jump_ratio]
        return min(nearby, key=lambda item: self._distance(
            self.locked_target, item
        )) if nearby else None

    def _same_target(self, previous, current):
        if previous.track_id >= 0 and current.track_id >= 0:
            if previous.track_id == current.track_id:
                return True
            # Trackers can reassign an ID after a short occlusion. In that
            # case spatial continuity is the safe fallback.
        return self._distance(previous, current) <= self.max_target_jump_ratio

    @staticmethod
    def _distance(previous, current):
        dx = (current.center_x - previous.center_x) / current.image_width
        dy = (current.center_y - previous.center_y) / current.image_height
        return math.hypot(dx, dy)


def draw_filter_overlay(frame, detections, filtered):
    """Draw raw YOLO detections and the confirmed target distinctly."""
    output = frame.copy()
    for item in detections:
        x1 = int(item.center_x - item.bbox_width / 2.0)
        y1 = int(item.center_y - item.bbox_height / 2.0)
        x2 = int(item.center_x + item.bbox_width / 2.0)
        y2 = int(item.center_y + item.bbox_height / 2.0)
        cv2.rectangle(output, (x1, y1), (x2, y2), (255, 180, 0), 2)
        cv2.putText(output,
                    f'RAW trash {item.confidence:.2f} id={item.track_id}',
                    (x1, max(18, y1 - 7)), cv2.FONT_HERSHEY_SIMPLEX,
                    0.55, (255, 180, 0), 2)
    if filtered.target is not None:
        item = filtered.target
        x1 = int(item.center_x - item.bbox_width / 2.0)
        y1 = int(item.center_y - item.bbox_height / 2.0)
        x2 = int(item.center_x + item.bbox_width / 2.0)
        y2 = int(item.center_y + item.bbox_height / 2.0)
        color = ((0, 255, 0) if filtered.state == TargetFilter.LOCKED
                 else (0, 165, 255))
        cv2.rectangle(output, (x1, y1), (x2, y2), color, 4)
        cv2.putText(output, 'CONFIRMED TARGET',
                    (x1, min(output.shape[0] - 8, y2 + 23)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, color, 2)
    cv2.rectangle(output, (12, 12), (420, 58), (0, 0, 0), -1)
    cv2.putText(output, f'STATE: {filtered.state}', (24, 44),
                cv2.FONT_HERSHEY_SIMPLEX, 0.85, (255, 255, 255), 2)
    return output
