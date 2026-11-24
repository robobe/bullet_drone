"""Small RGB/BGR red-truck centroid detector for owned SHM frames."""
from dataclasses import dataclass
from queue import Empty, Full, Queue
from threading import Event, Thread

import numpy as np


MIN_RED_PIXELS = 8
RESULT_QUEUE_CAPACITY = 1
CAMERA_POLL_MS = 20


@dataclass(frozen=True, slots=True)
class TargetOffset:
    dx: float
    dy: float
    pixels: int


@dataclass(frozen=True, slots=True)
class VisionSample:
    frame_step: int
    target: TargetOffset | None


def detect_target(metadata, pixels):
    width, height, step = metadata['width'], metadata['height'], metadata['step']
    if len(pixels) != step * height or metadata['encoding'] not in ('rgb8', 'bgr8'):
        raise ValueError('invalid camera frame layout')
    image = np.ndarray((height, width, 3), dtype=np.uint8, buffer=pixels,
                       strides=(step, 3, 1))
    red, green, blue = (image[:, :, i] for i in
                        ((0, 1, 2) if metadata['encoding'] == 'rgb8' else (2, 1, 0)))
    mask = (red >= 120) & (green <= 105) & (blue <= 105) & (red >= green * 1.35)
    ys, xs = np.nonzero(mask)
    count = len(xs)
    if count < MIN_RED_PIXELS:
        return None
    return TargetOffset(float((xs.mean() - (width - 1) / 2) / (width / 2)),
                        float((ys.mean() - (height - 1) / 2) / (height / 2)), count)


class VisionWorker:
    """Poll shared-memory camera frames and detect targets outside the flight loop."""

    def __init__(self, viewer):
        self.viewer = viewer
        self.stop = Event()
        self.results = Queue(RESULT_QUEUE_CAPACITY)
        self.latest = None
        self.error = None
        self.frames_read = 0
        self.detections = 0
        self.thread = Thread(target=self._run, name='red-truck-vision', daemon=True)

    def __enter__(self):
        self.thread.start()
        return self

    def __exit__(self, *exc):
        self.stop.set()
        self.thread.join()

    def sample(self, current_step, max_age_steps):
        if self.error is not None:
            raise RuntimeError(f'Camera vision worker failed: {self.error}') from self.error
        try:
            self.latest = self.results.get_nowait()
        except Empty:
            pass
        result = self.latest
        if result is None:
            return None
        frame_id, target = result
        if not 0 <= current_step - frame_id <= max_age_steps:
            return None
        return VisionSample(frame_id, target)

    def target(self, current_step, max_age_steps):
        sample = self.sample(current_step, max_age_steps)
        return None if sample is None else sample.target

    def _run(self):
        last_frame_id = -1
        while not self.stop.is_set():
            try:
                frame = self.viewer.poll_frame(timeout_ms=CAMERA_POLL_MS)
                if frame is None:
                    continue
                metadata, pixels = frame
                frame_id = metadata.get('header', {}).get('fid', -1)
                if type(frame_id) is not int or frame_id <= last_frame_id:
                    continue
                self.frames_read += 1
                target = detect_target(metadata, pixels)
                self.detections += 1
                last_frame_id = frame_id
                try:
                    self.results.put_nowait((frame_id, target))
                except Full:
                    try:
                        self.results.get_nowait()
                    except Empty:
                        pass
                    self.results.put_nowait((frame_id, target))
            except Exception as error:
                self.error = error
                self.stop.set()
