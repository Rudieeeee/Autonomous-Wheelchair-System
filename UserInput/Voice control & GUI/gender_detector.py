"""
gender_detector.py — Pitch-based speaker gender detector
========================================================

Estimates the fundamental frequency (F0) of a short voiced segment via
autocorrelation and classifies the speaker as 'male' or 'female'. The
result is intended to drive the address used by the assistant
("sir" vs "madame").

Design choices
--------------
- Offline, dependency-free beyond NumPy (matches REQ-UI-FR06).
- Works on the raw 16-bit PCM bytes Vosk already gets, so we don't
  duplicate the audio capture.
- A short rolling buffer (~1.0 s) is analysed per call, so the first
  utterance after wake-up is enough to lock a decision.
- The decision is sticky once we have N voiced frames; subsequent
  speech only refines it.

Heuristic threshold:
    F0 < 165 Hz                 → male
    F0 > 185 Hz                 → female
    165 ≤ F0 ≤ 185 Hz (overlap) → keep previous decision (or None)

These bands are well-supported by the speech-perception literature
for adult speakers and are conservative enough that adolescents and
softer voices still classify reliably in our demo context.
"""

from __future__ import annotations

import threading
from collections import deque
from typing import Deque, Optional

import numpy as np


class GenderDetector:
    """Streaming F0-based male/female classifier."""

    def __init__(
        self,
        sample_rate: int = 16000,
        analysis_window_s: float = 1.0,
        min_voiced_frames: int = 3,
        f0_min: float = 70.0,
        f0_max: float = 350.0,
        male_max_hz: float = 165.0,
        female_min_hz: float = 185.0,
        rms_voiced_threshold: float = 350.0,  # int16 RMS ≈ "user is talking"
    ) -> None:
        self.sample_rate = sample_rate
        self.window_samples = int(analysis_window_s * sample_rate)
        self.min_voiced_frames = min_voiced_frames
        self.f0_min = f0_min
        self.f0_max = f0_max
        self.male_max_hz = male_max_hz
        self.female_min_hz = female_min_hz
        self.rms_voiced_threshold = rms_voiced_threshold

        self._buffer: Deque[int] = deque(maxlen=self.window_samples)
        self._f0_history: Deque[float] = deque(maxlen=20)
        self._gender: Optional[str] = None
        self._lock = threading.Lock()

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------
    def feed(self, pcm_bytes: bytes) -> None:
        """Push a chunk of int16 mono PCM into the rolling buffer.

        Called from the audio thread between Vosk frames. Cheap; the
        actual F0 work happens in `analyse()`.
        """
        if not pcm_bytes:
            return
        samples = np.frombuffer(pcm_bytes, dtype=np.int16)
        with self._lock:
            self._buffer.extend(samples.tolist())

    def analyse(self) -> Optional[str]:
        """Look at the latest ~1 s and update the running decision.

        Returns the (possibly updated) gender label or None if we still
        don't have enough voiced data.
        """
        with self._lock:
            if len(self._buffer) < self.window_samples:
                return self._gender
            window = np.array(self._buffer, dtype=np.float32)

        rms = float(np.sqrt(np.mean(window * window)))
        if rms < self.rms_voiced_threshold:
            return self._gender  # silence — keep last decision

        f0 = self._estimate_f0(window)
        if f0 is None:
            return self._gender

        self._f0_history.append(f0)
        if len(self._f0_history) < self.min_voiced_frames:
            return self._gender

        median_f0 = float(np.median(self._f0_history))
        if median_f0 < self.male_max_hz:
            self._gender = "male"
        elif median_f0 > self.female_min_hz:
            self._gender = "female"
        # else: ambiguous band — keep prior decision
        return self._gender

    @property
    def gender(self) -> Optional[str]:
        return self._gender

    @property
    def address(self) -> str:
        """Polite address word: 'sir' if male, 'madame' if female,
        defaulting to 'sir' until we are confident — matches the
        Iron Man Jarvis convention."""
        if self._gender == "female":
            return "madame"
        return "sir"

    def reset(self) -> None:
        with self._lock:
            self._buffer.clear()
        self._f0_history.clear()
        self._gender = None

    # ------------------------------------------------------------------
    # Internals
    # ------------------------------------------------------------------
    def _estimate_f0(self, window: np.ndarray) -> Optional[float]:
        """Autocorrelation pitch estimator.

        We zero-mean and Hanning-window the signal, autocorrelate via
        FFT, then look for the highest peak inside the lag range
        corresponding to f0_min..f0_max. Cheap, robust enough for a
        clip-on directional mic.
        """
        # Pre-emphasis lifts higher harmonics — helps when the directional
        # mic rolls off lows.
        window = window - np.mean(window)
        window = np.append(window[0], window[1:] - 0.97 * window[:-1])
        window *= np.hanning(len(window))

        # FFT-based autocorrelation (faster than np.correlate for ~16k samples).
        n = 1 << (len(window) - 1).bit_length() << 1
        spectrum = np.fft.rfft(window, n=n)
        autocorr = np.fft.irfft(spectrum * np.conj(spectrum), n=n)
        autocorr = autocorr[: len(window)]

        if autocorr[0] <= 0:
            return None

        min_lag = int(self.sample_rate / self.f0_max)
        max_lag = int(self.sample_rate / self.f0_min)
        if max_lag >= len(autocorr):
            return None
        segment = autocorr[min_lag:max_lag]
        if len(segment) == 0:
            return None
        peak_lag = int(np.argmax(segment)) + min_lag
        peak_val = autocorr[peak_lag]

        # Confidence gate: peak must be a clear fraction of the zero-lag.
        if peak_val < 0.3 * autocorr[0]:
            return None

        # Parabolic interpolation around the peak for sub-sample accuracy.
        if 0 < peak_lag < len(autocorr) - 1:
            a, b, c = autocorr[peak_lag - 1], autocorr[peak_lag], autocorr[peak_lag + 1]
            denom = (a - 2 * b + c)
            if denom != 0:
                peak_lag = peak_lag + 0.5 * (a - c) / denom

        f0 = self.sample_rate / float(peak_lag)
        if not (self.f0_min <= f0 <= self.f0_max):
            return None
        return f0
