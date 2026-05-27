"""
voice_control.py — Voice Control Subsystem for Smart Wheelchair BAP
===================================================================

Wake-word activated, LLM-gated, conversational voice control with
optional GUI-friendly observer hooks and pitch-based gender detection.

Behavior:
  • Idle until the user says "Jarvis".
  • On wake, Jarvis replies "What do you want, sir/madame?" and stays
    in conversation mode for ~30 s. Every new utterance refreshes the
    timer, so the user can have a real back-and-forth without
    repeating the wake word.
  • Each utterance is first checked against the FastNavigator (zero-
    latency keyword matcher). If it matches a navigation command, the
    chair moves immediately — NO LLM call needed.
  • Non-navigate intents are classified by an LLM (Groq, free hosted):
        show_map  — bring up the on-screen venue map
        hide_map  — dismiss the map
        question  — answer it aloud (live weather/time injected as context)
        chatter   — gentle nudge to clarify
        goodbye   — end the conversation, return to idle.
  • The chair only moves on "navigate". Mentioning a place is not enough.
  • A streaming pitch detector decides whether the speaker is male or
    female; the LLM is told to address them as "sir" or "madame".
  • A neural DeepFilterDenoiser (DeepFilterNet3) suppresses background
    noise and competing voices before the audio reaches Vosk.

Recommended microphone: Plantronics Voyager 5200 UC (BT300M adapter)
  → Use device 24: Headset Microphone (Plantronics BT300M), WASAPI
    (lowest latency Windows audio path; device index already set below)

Setup:
  1) pip install -r requirements.txt
  2) Get a free Groq API key at https://console.groq.com
  3) Save the key in a .env file next to this script:
        GROQ_API_KEY=gsk_...
  4) Run:  python main.py        (GUI + voice)
        or python voice_control.py  (CLI fallback, no GUI)
"""

from __future__ import annotations

import asyncio
import json
import os
import queue
import re
import sys
import threading
import time
import urllib.request
from dataclasses import dataclass, field
from datetime import datetime
from typing import Callable, List, Optional, Tuple, Union

import numpy as np
import sounddevice as sd
from vosk import KaldiRecognizer, Model, SetLogLevel

from gender_detector import GenderDetector

SetLogLevel(-1)

# ---------------------------------------------------------------------------
# Noise / hallucination rejection constants
# ---------------------------------------------------------------------------
# Vosk commonly outputs these short common words when it hears silence or
# low-level background noise.  Any final result that consists ONLY of one of
# these words is discarded before it ever reaches the wake-word gate.
_STT_NOISE_WORDS: frozenset = frozenset({
    "the", "a", "an", "in", "it", "is", "of", "and", "to", "i",
    "oh", "ah", "mm", "hmm", "hm", "uh", "um", "so", "or",
})
# Average per-word confidence threshold.  Results below this are also dropped.
_STT_MIN_CONF: float = 0.45

# ---------------------------------------------------------------------------
# Fast navigation verb regex
# Matches explicit motion commands BEFORE sending to the LLM.
# Covers all phrases listed in the system prompt + common speech shortcuts.
# ---------------------------------------------------------------------------
_NAV_VERB_RE = re.compile(
    r'\b(?:'
    r'(?:take|bring|drive|escort|carry|wheel)\s+me\s+(?:to\b|towards?\b)?|'
    r'bring\s+me\b|take\s+me\b|'           # "bring me lab a" (no "to")
    r'go\s+to\b|'
    r"let'?s\s+go\s+to\b|"
    r'i\s+(?:want|need|wanna|would\s+like)\s+to\s+(?:go\s+)?to\b|'
    r'(?:head|navigate|move|roll|proceed)\s+(?:to\b|towards?\b)'
    r')',
    re.IGNORECASE,
)

# "stop" by itself (with optional politeness / emphasis words)
_STOP_ONLY_RE = re.compile(
    r'^\s*(?:please\s+)?(?:emergency\s+)?stop[.!]?\s*$',
    re.IGNORECASE,
)

# ---------------------------------------------------------------------------
# Voice confirmation word sets
# Used by VoiceController._voice_confirm() to interpret the user's yes/no.
# ---------------------------------------------------------------------------
_CONFIRM_YES: frozenset = frozenset({
    "yes", "yeah", "yep", "yup", "sure", "okay", "ok",
    "go", "go ahead", "do it", "proceed", "confirm", "confirmed",
    "correct", "affirmative", "please", "let's go", "lets go",
    "of course", "absolutely", "definitely",
})
_CONFIRM_NO: frozenset = frozenset({
    "no", "nope", "nah", "cancel", "stop", "abort",
    "don't", "do not", "negative", "wait", "hold on", "hold",
    "nevermind", "never mind", "back", "actually", "forget it",
    "forget that",
})

# Auto-load .env if present (so GROQ_API_KEY just works).
try:
    from dotenv import load_dotenv
    load_dotenv()
except ImportError:
    pass


# -----------------------------------------------------------------------------
# Configuration
# -----------------------------------------------------------------------------
@dataclass
class VoiceConfig:
    """All tunable parameters for the voice subsystem."""

    # ---- Vosk (speech-to-text, offline) --------------------------------
    model_path: str = "models/vosk-model-en-us-0.22"
    sample_rate: int = 16000
    # 2000 samples @ 16 kHz = 125 ms chunks — faster partial feedback than
    # the previous 250 ms without hurting Vosk accuracy.
    block_size: int = 2000
    # Device 24 = Headset Microphone (Plantronics BT300M), Windows WASAPI
    # WASAPI has the lowest latency of all Windows audio APIs.
    input_device: Optional[int] = 24
    grammar_locked: bool = False
    show_partials: bool = True

    # ---- Wake word & conversation ---------------------------------------
    wake_word: str = "jarvis"
    awake_timeout_s: float = 30.0
    wake_acknowledgement: str = "What do you want?"

    # ---- Voice confirmation -------------------------------------------
    # After a navigate command is recognised, Jarvis asks "Shall I take
    # you to X, sir?" and listens for a spoken yes/no reply.
    # confirm_timeout_s is how long Jarvis waits before giving up (cancel).
    confirm_timeout_s: float = 8.0

    # ---- Confirmation window (legacy keyboard fallback) ---------------
    confirmation_seconds: float = 2.0

    # ---- Fast navigation (keyword bypass — no LLM needed) --------------
    # When True, utterances matching a motion verb + known destination are
    # dispatched immediately without calling the LLM (~0 ms vs ~300 ms).
    fast_navigate: bool = True

    # ---- Noise gate (energy-based mic filter) --------------------------
    # The Plantronics Voyager 5200 UC has hardware Acoustic Fence which
    # rejects most off-axis crowd noise physically.  Field testing in a
    # noisy open-day-like environment showed that loud neighbours still
    # bled through and were being picked up by Vosk as low-confidence
    # partials, so the software gate is now ON by default.
    #
    # Threshold rationale: the user's mouth is ~3 cm from the boom mic,
    # so the user's voice arrives at roughly −20 dBFS RMS.  Crowd voices
    # that survive the Acoustic Fence arrive at roughly −50 dBFS RMS.
    # −42 dBFS sits cleanly between the two and gives a healthy margin
    # for quiet consonants in "Jarvis" without letting crowd through.
    # If you switch to a plain omni mic, drop this to ~−55 dBFS.
    noise_gate_enabled: bool = True
    noise_gate_threshold_db: float = -42.0
    # Keep the gate open this many extra 125 ms frames after speech drops
    # below threshold so trailing consonants are not swallowed.
    noise_gate_hold_frames: int = 10

    # ---- Silence gate (utterance completion by pause detection) --------
    # Buffer Vosk final results and only dispatch them after the user has
    # been silent for at least silence_commit_s seconds.  This prevents
    # natural mid-sentence pauses from cutting an utterance short and
    # triggering the command pipeline on a fragment.
    #
    # Emergency stop commands ("stop" alone) always bypass this gate and
    # are dispatched immediately regardless of this setting.
    #
    # Set use_silence_gate=False to revert to the original Vosk-boundary
    # behaviour (instantaneous dispatch on each Vosk final result).
    use_silence_gate: bool = True
    silence_commit_s: float = 2.0

    # ---- Neural denoiser (DeepFilterNet3 before Vosk) ------------------
    # Third stage of the rejection pipeline.  Runs every speech chunk
    # through DeepFilterNet3, a full-band neural speech enhancement model
    # that suppresses both stationary noise and competing voices (babble,
    # crowd) — the exact scenario at an open-day demo.
    #
    # Requires the deepfilternet package.  Falls back to passthrough with
    # a printed warning if not installed.
    spectral_denoiser_enabled: bool = True

    # ---- LLM intent gate -----------------------------------------------
    llm_provider: str = "groq"          # "groq" | "ollama"
    groq_api_key: Optional[str] = None  # None → reads GROQ_API_KEY env / .env
    groq_model: str = "llama-3.1-8b-instant"   # ~200 ms; fine for intent
    ollama_host: str = "http://localhost:11434"
    ollama_model: str = "llama3.2:3b"
    llm_timeout_s: float = 6.0
    llm_temperature: float = 0.0
    history_turns: int = 8

    # ---- Tool context (real-time facts injected into prompt) -----------
    enable_weather_tool: bool = True
    enable_time_tool: bool = True
    location_lat: float = 52.0116        # Delft
    location_lon: float = 4.3571
    location_name: str = "Delft"

    # ---- Text-to-speech ------------------------------------------------
    tts_enabled: bool = True
    tts_provider: str = "edge"           # "sapi" | "edge"
    tts_rate: int = 180
    tts_voice: Optional[str] = "George"
    edge_voice: str = "en-GB-ThomasNeural"

    # ---- Speaker gender / address --------------------------------------
    enable_gender_detection: bool = True
    default_address: str = "sir"

    # ---- Destinations --------------------------------------------------
    destinations: dict = field(default_factory=lambda: {
        "lab a": "LOC_LAB_A",
        "lab b": "LOC_LAB_B",
        "cafeteria": "LOC_CAFETERIA",
        "entrance": "LOC_ENTRANCE",
        "exit": "LOC_EXIT",
        "office": "LOC_OFFICE",
        "stop": "EMERGENCY_STOP",
    })


# -----------------------------------------------------------------------------
# Noise gate (energy-based, real-time, near-zero latency)
# -----------------------------------------------------------------------------
class NoiseGate:
    """
    Suppresses audio chunks that are almost certainly background noise.

    How it works
    ------------
    For every 125 ms PCM chunk coming off the microphone:
      1. Calculate the RMS energy of the int16 samples.
      2. If the energy is below `threshold_db` dBFS AND the hold counter
         has expired, return a block of silence (zeros) instead of the
         original audio.  Vosk treats silence as a clean word boundary, so
         the transcription pipeline still works correctly.
      3. If the energy is above threshold, reset the hold counter and pass
         the audio through unchanged.

    The hold period prevents trailing consonants and gentle word endings
    from being swallowed by the gate.

    The Plantronics Voyager 5200 UC already applies hardware acoustic
    echo cancellation and noise suppression via its DSP.  This gate is a
    second, software-side layer for any residual noise that slips through.
    """

    def __init__(
        self,
        threshold_db: float = -42.0,
        hold_frames: int = 6,
    ) -> None:
        # Convert dBFS to a linear RMS value for int16 audio (range 0–32768).
        self._threshold = 32768.0 * (10.0 ** (threshold_db / 20.0))
        self._hold = hold_frames
        self._hold_count: int = 0

    def process(self, pcm_bytes: bytes) -> Tuple[bytes, bool]:
        """
        Classify a PCM chunk as speech or silence and return the audio to
        forward downstream.

        Returns a tuple ``(audio, is_speech)``:
          * audio    — original chunk if it is speech (or still in the
                       hold window after speech), or a zero-filled block
                       of the same length if it is silence.  Vosk treats
                       the zero block as a clean word boundary.
          * is_speech — True when the caller should forward the chunk to
                       the recogniser, False when the chunk is silence.
                       The DeepFilterDenoiser uses this flag to decide
                       whether to enhance the chunk (speech) or skip it
                       (silence).
        """
        samples = np.frombuffer(pcm_bytes, dtype=np.int16).astype(np.float32)
        rms = float(np.sqrt(np.mean(samples ** 2))) if samples.size else 0.0

        if rms >= self._threshold:
            self._hold_count = self._hold   # speech detected — reset hold
            return pcm_bytes, True
        else:
            if self._hold_count > 0:
                self._hold_count -= 1
                return pcm_bytes, True      # still holding open after speech
            # Replace with silence so Vosk sees a clean gap.
            return bytes(len(pcm_bytes)), False


# -----------------------------------------------------------------------------
# Deep neural denoiser (DeepFilterNet3 in front of Vosk)
# -----------------------------------------------------------------------------
class DeepFilterDenoiser:
    """
    Neural speech enhancement for the third stage of the rejection pipeline.

    Stage map
    ---------
      1. Plantronics Voyager 5200 UC hardware Acoustic Fence — physical
         beamforming, rejects most off-axis crowd noise at the mic.
      2. NoiseGate (RMS) — gates out anything quieter than the speaker's
         own voice (close mic always wins on energy).
      3. DeepFilterDenoiser (this class) — runs each speech chunk through
         DeepFilterNet3, a neural full-band speech enhancement model that
         suppresses stationary noise, transient events, and competing
         voices alike, leaving only the target speaker.

    Why DeepFilterNet over spectral subtraction
    --------------------------------------------
    The previous noisereduce stage learned a stationary noise floor from
    silence frames and subtracted it.  That works for HVAC hum but falls
    apart when the competition is another person speaking — a neighbour's
    voice is not in the silence frames so it never enters the reference.
    DeepFilterNet was trained on thousands of hours of mixed speech and
    noise including babble and crowd scenes, so it suppresses competing
    voices as well as it suppresses fan noise.

    Resampling
    ----------
    DeepFilterNet operates at 48 kHz.  Audio arriving from the 16 kHz
    pipeline is upsampled before enhancement and downsampled afterwards,
    so the rest of the pipeline (Vosk, NoiseGate) remains unchanged.

    Graceful degradation
    --------------------
    If the ``deepfilternet`` package is not installed the denoiser prints
    a one-time warning and acts as passthrough so the system still runs.
    Install with:  pip install deepfilternet
    """

    def __init__(self, sample_rate: int = 16000) -> None:
        self.sample_rate = sample_rate
        self._model = None
        self._df_state = None
        self._torch = None
        self._torchaudio = None
        self._available = False
        try:
            import torch
            import torchaudio
            from df import init_df
            self._torch = torch
            self._torchaudio = torchaudio
            self._model, self._df_state, _ = init_df()
            self._available = True
            print("[DeepFilterDenoiser] Active — DeepFilterNet3 loaded.")
        except Exception as e:
            print(f"[DeepFilterDenoiser] DeepFilterNet unavailable: {e}. "
                  "Passthrough.  Install with: pip install deepfilternet")

    def feed_noise(self, pcm_bytes: bytes) -> None:
        """No-op — DeepFilterNet is a neural model and needs no noise profile."""

    def process(self, pcm_bytes: bytes) -> bytes:
        """
        Enhance a speech chunk using DeepFilterNet3.

        Internally resamples from the pipeline sample rate (16 kHz) to
        48 kHz for the model, then downsamples back before returning.
        Falls back to passthrough on any error.
        """
        if not self._available:
            return pcm_bytes
        try:
            samples = (np.frombuffer(pcm_bytes, dtype=np.int16)
                       .astype(np.float32) / 32768.0)
            # Shape: (1, T) for a single-channel stream.
            audio = self._torch.from_numpy(samples).unsqueeze(0)
            # Upsample 16 kHz → 48 kHz.
            audio_48k = self._torchaudio.functional.resample(
                audio, self.sample_rate, 48000
            )
            from df import enhance
            enhanced_48k = enhance(self._model, self._df_state, audio_48k)
            # Downsample 48 kHz → 16 kHz.
            enhanced = self._torchaudio.functional.resample(
                enhanced_48k, 48000, self.sample_rate
            )
            out = enhanced.squeeze(0).numpy()
            np.clip(out, -1.0, 1.0, out=out)
            return (out * 32767.0).astype(np.int16).tobytes()
        except Exception as e:
            print(f"[DeepFilterDenoiser] enhance error: {e}")
            return pcm_bytes


# -----------------------------------------------------------------------------
# Fast navigation keyword matcher (zero LLM latency for drive commands)
# -----------------------------------------------------------------------------
class FastNavigator:
    """
    Resolves explicit navigation commands without calling the LLM.

    For utterances like "take me to lab a" or "bring me to the cafeteria",
    this class extracts the destination keyword using a compiled regex and
    maps it directly to a location ID — saving the ~200–500 ms Groq round-
    trip that would otherwise be needed.

    The LLM is still used for everything else (questions, map, goodbye,
    ambiguous chatter).

    Lookup table (destinations → location IDs)
    -------------------------------------------
    The table is derived from VoiceConfig.destinations, so adding a new
    location in the config automatically makes it fast-matchable.

    Matching rules
    --------------
    1. "stop" (alone, ≤3 words) → EMERGENCY_STOP immediately, no LLM.
    2. Motion verb (go to / take me / bring me / …) + known destination
       → dispatch immediately.
    3. Destination name preceded by optional "the" is accepted
       (e.g., "take me to the cafeteria").
    4. Anything that doesn't match falls through to the LLM.
    """

    def __init__(self, destinations: dict) -> None:
        self.destinations = destinations
        # Build a compiled regex for all non-stop destination names.
        # Sorted longest-first to avoid partial matches ("lab a" before "lab").
        dest_keys = sorted(
            [k for k in destinations if k != "stop"],
            key=len,
            reverse=True,
        )
        if dest_keys:
            pattern = (
                r'\b(?:the\s+)?('
                + '|'.join(re.escape(k) for k in dest_keys)
                + r')\b'
            )
            self._dest_re = re.compile(pattern, re.IGNORECASE)
        else:
            self._dest_re = None

    def match(
        self,
        text: str,
        address: str = "sir",
    ) -> Optional[tuple]:
        """
        Returns ``(dest_key, location_id, reply_text)`` if this utterance is
        a clear navigation command, or ``None`` if the LLM should handle it.
        """
        # ---- Emergency stop (highest priority) ----
        if _STOP_ONLY_RE.match(text):
            loc_id = self.destinations.get("stop")
            if loc_id:
                return ("stop", loc_id, f"Stopping immediately, {address}.")

        # ---- Motion verb check ----
        if not _NAV_VERB_RE.search(text):
            return None     # no motion verb → let LLM decide intent

        # ---- Destination keyword check ----
        if self._dest_re is None:
            return None

        m = self._dest_re.search(text)
        if not m:
            return None     # motion verb present but no known destination

        dest_key = m.group(1).lower()
        # Normalise to the exact key stored in the dict
        actual_key = next(
            (k for k in self.destinations if k.lower() == dest_key),
            None,
        )
        if actual_key is None or actual_key == "stop":
            return None

        loc_id = self.destinations[actual_key]
        dest_display = actual_key.title()
        reply = f"On my way to {dest_display}, {address}."
        return (actual_key, loc_id, reply)


# -----------------------------------------------------------------------------
# Audio capture
# -----------------------------------------------------------------------------
class AudioStream:
    def __init__(self, config: VoiceConfig):
        self.config = config
        self.queue: "queue.Queue[bytes]" = queue.Queue()
        self._stream: Optional[sd.RawInputStream] = None

    def _callback(self, indata, frames, time_info, status):
        if status:
            print(f"[AudioStream] {status}", file=sys.stderr)
        self.queue.put(bytes(indata))

    def start(self) -> None:
        self._stream = sd.RawInputStream(
            samplerate=self.config.sample_rate,
            blocksize=self.config.block_size,
            dtype="int16",
            channels=1,
            device=self.config.input_device,
            callback=self._callback,
        )
        self._stream.start()
        print(f"[AudioStream] Capturing @ {self.config.sample_rate} Hz, "
              f"block={self.config.block_size} samples "
              f"({self.config.block_size / self.config.sample_rate * 1000:.0f} ms), "
              f"device={self.config.input_device or 'default'}")

    def stop(self) -> None:
        if self._stream is not None:
            self._stream.stop()
            self._stream.close()
            self._stream = None

    @staticmethod
    def list_devices() -> None:
        print(sd.query_devices())


# -----------------------------------------------------------------------------
# Vosk engine
# -----------------------------------------------------------------------------
class VoskEngine:
    def __init__(self, config: VoiceConfig):
        self.config = config
        print(f"[VoskEngine] Loading model from '{config.model_path}' ...")
        self._model = Model(config.model_path)
        if config.grammar_locked:
            grammar = json.dumps(
                list(config.destinations.keys()) + [config.wake_word, "[unk]"]
            )
            self._recognizer = KaldiRecognizer(
                self._model, config.sample_rate, grammar
            )
        else:
            self._recognizer = KaldiRecognizer(self._model, config.sample_rate)
        self._recognizer.SetWords(True)
        print("[VoskEngine] Ready.")

    def feed(self, pcm_bytes: bytes) -> Optional[dict]:
        if self._recognizer.AcceptWaveform(pcm_bytes):
            return json.loads(self._recognizer.Result())
        return None

    def partial(self) -> dict:
        return json.loads(self._recognizer.PartialResult())


# -----------------------------------------------------------------------------
# Wake-word + conversation gate
# -----------------------------------------------------------------------------
class _WakeAwoken:
    pass
AWOKEN = _WakeAwoken()


class WakeWordGate:
    """
    Idle until "Jarvis" is heard, then enter a conversation window where
    every subsequent utterance is processed without needing the wake word
    again. The window auto-extends on each utterance and resets to idle
    on (a) timeout, (b) successful navigate command, or (c) goodbye.
    """

    def __init__(self, config: VoiceConfig):
        self.wake_word = config.wake_word.lower()
        self.awake_timeout_s = config.awake_timeout_s
        self._awake_until = 0.0
        self._active = False

    @property
    def is_awake(self) -> bool:
        return self._active and time.time() < self._awake_until

    def filter(self, transcript: str) -> Optional[Union[str, _WakeAwoken]]:
        text = transcript.lower().strip()
        if not text:
            return None

        if self.wake_word in text:
            idx = text.find(self.wake_word) + len(self.wake_word)
            tail = text[idx:].lstrip(" ,.:;!?-")
            self._enter_conversation()
            return tail if tail else AWOKEN

        if self.is_awake:
            self._refresh()
            return text

        return None

    def _enter_conversation(self) -> None:
        self._active = True
        self._refresh()

    def _refresh(self) -> None:
        self._awake_until = time.time() + self.awake_timeout_s

    def end_conversation(self) -> None:
        self._active = False
        self._awake_until = 0.0


# -----------------------------------------------------------------------------
# Silence-gated utterance committer
# -----------------------------------------------------------------------------
class SilenceCommitter:
    """
    Buffers wake-gate-passed Vosk finals and only dispatches once the
    user has been silent for a configurable window.

    Why this helps
    --------------
    Vosk's internal endpointing fires on short natural pauses (~0.3 s),
    so a sentence like "take me to … uh … lab a" can arrive as two
    separate finals: "take me to" and "uh lab a".  Without buffering,
    the first fragment reaches the command pipeline on its own and may
    be misclassified.  Waiting for a longer silence (default 2 s) lets
    the user finish a sentence naturally before any processing begins.

    Emergency-stop bypass
    ---------------------
    The caller (VoiceController.run) matches _STOP_ONLY_RE before
    feeding here and dispatches those directly, so "stop" is always
    instantaneous regardless of this gate.

    API
    ---
    feed(text)          — add a wake-gate result to the buffer and mark
                          the current time as the last speech moment.
    tick(is_speech)     — call on every audio chunk; returns the
                          committed utterance string when the silence
                          window has elapsed, otherwise None.
    reset()             — discard pending buffer (called on re-wake or
                          after an emergency dispatch).
    """

    def __init__(self, silence_s: float = 2.0) -> None:
        self._silence_s = silence_s
        self._buffer: list = []
        self._last_speech_t: float = 0.0

    @property
    def has_pending(self) -> bool:
        return bool(self._buffer)

    def feed(self, text: str) -> None:
        """Append a wake-gate result and stamp the last-speech time."""
        if text.strip():
            self._buffer.append(text.strip())
            # Treat the arrival of a new final as implicit speech activity
            # so the silence timer resets on each new fragment.
            self._last_speech_t = time.time()

    def tick(self, is_speech: bool) -> Optional[str]:
        """
        Update speech-activity tracking and return any committed utterance.

        Parameters
        ----------
        is_speech:
            True when the NoiseGate classified the current 125 ms chunk
            as speech.  When the gate is disabled, pass True whenever
            Vosk produced a non-empty partial in the same chunk.

        Returns
        -------
        The accumulated utterance string when silence_s consecutive
        seconds of silence have been detected, otherwise None.
        """
        if is_speech:
            self._last_speech_t = time.time()
        if not self._buffer:
            return None
        elapsed = time.time() - self._last_speech_t
        if elapsed >= self._silence_s:
            utterance = " ".join(self._buffer).strip()
            self._buffer.clear()
            return utterance or None
        return None

    def reset(self) -> None:
        """Discard pending buffer without dispatching."""
        self._buffer.clear()
        self._last_speech_t = 0.0


# -----------------------------------------------------------------------------
# LLM intent classifier (Groq or Ollama) + conversation history + live tools
# -----------------------------------------------------------------------------
@dataclass
class Intent:
    intent_type: str           # "navigate" | "show_map" | "hide_map" |
                               # "question" | "chatter" | "goodbye"
    destination: Optional[str]
    reply: Optional[str]
    raw: str = ""


_INTENT_SYSTEM_PROMPT = """\
You are Jarvis, the polite British conversational assistant for an
autonomous smart wheelchair built as a Bachelor End Project at TU Delft.
The user wears a clip-on microphone and is in a public open day demo.
They have already addressed you (the wake word "Jarvis" triggered) and
may chat with you across multiple turns until they say goodbye.

You speak in a warm, butler-like, lightly formal British register —
short sentences, slight wit, never sycophantic. Always address the
user using the term provided in the live "Address user as" context
(default: "sir"). Use the address naturally inside the sentence; do
not force it into every reply.

Classify each user utterance into ONE of six intents:

  navigate  — an EXPLICIT drive command. The user is telling the chair
              to MOVE. Look for verbs of motion: "go to", "take me to",
              "bring me to", "drive to", "let's go to", "i want to go to",
              "head to". Naming a place WITHOUT such a verb is NOT a
              command. Destination MUST be one of the valid destinations.
              "stop" alone is always navigate (emergency stop).

  show_map  — the user wants to see the venue map. Triggers: "show me
              the map", "show the map", "open the map", "where am i",
              "where can i go", "what are my options", "let me see the
              floor plan".

  hide_map  — the user wants to dismiss the map: "close the map",
              "hide the map", "go back", "dismiss this", "okay close it".

  question  — the user is asking you anything: facts, time, weather,
              jokes, opinions, status, small talk questions. Answer
              concisely and warmly (≤30 words). Use any "Real-time facts"
              context provided. Be honest if you don't know.

  goodbye   — the user is ending the chat: "thanks", "thank you", "bye",
              "that's all", "never mind", "goodbye", "stop talking",
              "we're done", "ok cool". If the map is currently visible,
              prefer hide_map instead.

  chatter   — false alarm or incomplete: a place name with no command
              verb, a fragment, or speech clearly aimed at someone else.

Valid destinations (use the exact phrasing): lab a, lab b, cafeteria,
entrance, exit, office.

Output ONLY a single JSON object, no prose, exactly this schema:
{
  "intent": "navigate" | "show_map" | "hide_map" | "question" | "chatter" | "goodbye",
  "destination": "<one valid destination>" or null,
  "reply": "<short spoken response, <=30 words>"
}

Examples (assume address = "sir"; substitute "madame" if so instructed):
"take me to lab a"             -> {"intent":"navigate","destination":"lab a","reply":"On my way to lab A, sir."}
"bring me to the entrance"     -> {"intent":"navigate","destination":"entrance","reply":"Heading to the entrance, sir."}
"i want to go to the cafeteria"-> {"intent":"navigate","destination":"cafeteria","reply":"Going to the cafeteria, sir."}
"stop"                         -> {"intent":"navigate","destination":"stop","reply":"Stopping, sir."}
"show me the map"              -> {"intent":"show_map","destination":null,"reply":"Of course, sir."}
"open the floor plan"          -> {"intent":"show_map","destination":null,"reply":"Right away, sir."}
"close the map"                -> {"intent":"hide_map","destination":null,"reply":"Closing the map, sir."}
"the cafeteria"                -> {"intent":"chatter","destination":null,"reply":"Would you like me to take you there, sir? Just say take me to the cafeteria."}
"what's the weather like"      -> {"intent":"question","destination":null,"reply":"Partly cloudy and 16 degrees in Delft, sir."}
"and tomorrow"                 -> {"intent":"question","destination":null,"reply":"I don't have the forecast, sir, but expect typical Dutch spring weather."}
"who built you"                -> {"intent":"question","destination":null,"reply":"A Bachelor End Project team at TU Delft, sir."}
"tell me a joke"               -> {"intent":"question","destination":null,"reply":"Why don't wheelchairs play chess, sir? Because they always roll into checkmate."}
"thanks"                       -> {"intent":"goodbye","destination":null,"reply":"Anytime, sir. Just say Jarvis when you need me."}
"that's all"                   -> {"intent":"goodbye","destination":null,"reply":"Very good, sir."}
"""


# Open-Meteo weather code -> human description.
_WEATHER_CODES = {
    0: "clear", 1: "mostly clear", 2: "partly cloudy", 3: "cloudy",
    45: "foggy", 48: "foggy with frost",
    51: "drizzle", 53: "drizzle", 55: "heavy drizzle",
    61: "rain", 63: "rain", 65: "heavy rain",
    71: "snow", 73: "snow", 75: "heavy snow",
    80: "rain showers", 81: "rain showers", 82: "violent rain showers",
    95: "thunderstorm", 96: "thunderstorm with hail", 99: "thunderstorm with hail",
}


class IntentClassifier:
    """Provider-agnostic LLM intent gate with conversation history + tools."""

    def __init__(self, config: VoiceConfig):
        self.config = config
        self._available = False
        self._provider = config.llm_provider
        self._history: List[dict] = []

        if self._provider == "groq":
            self._init_groq()
        elif self._provider == "ollama":
            self._init_ollama()
        else:
            print(f"[IntentClassifier] unknown provider '{self._provider}'.")

    # ---- providers -------------------------------------------------
    def _init_groq(self) -> None:
        try:
            from openai import OpenAI
            api_key = self.config.groq_api_key or os.getenv("GROQ_API_KEY")
            if not api_key:
                print("[IntentClassifier] GROQ_API_KEY not set. "
                      "Add it to a .env file: GROQ_API_KEY=gsk_...")
                return
            self._client = OpenAI(
                api_key=api_key,
                base_url="https://api.groq.com/openai/v1",
                timeout=self.config.llm_timeout_s,
            )
            self._model = self.config.groq_model
            self._available = True
            print(f"[IntentClassifier] Groq ready ({self._model}).")
        except Exception as e:
            print(f"[IntentClassifier] Groq init failed: {e}")

    def _init_ollama(self) -> None:
        try:
            import ollama
            self._client = ollama.Client(
                host=self.config.ollama_host,
                timeout=self.config.llm_timeout_s,
            )
            self._client.show(self.config.ollama_model)
            self._model = self.config.ollama_model
            self._available = True
            print(f"[IntentClassifier] Ollama ready ({self._model}).")
        except Exception as e:
            print(f"[IntentClassifier] Ollama init failed: {e}")

    # ---- public API ------------------------------------------------
    def classify(self, utterance: str, address: str = "sir") -> Intent:
        if not self._available:
            return Intent("chatter", None,
                          "My language model is offline; please set "
                          "GROQ_API_KEY in a .env file.", "")

        # Build messages: system + tool context + history + new turn.
        messages = [{"role": "system", "content": _INTENT_SYSTEM_PROMPT}]
        addr_ctx = (
            f'Address user as: "{address}". '
            "Use this exact word when politely addressing them."
        )
        messages.append({"role": "system", "content": addr_ctx})
        tool_ctx = self._build_tool_context(utterance)
        if tool_ctx:
            messages.append({"role": "system", "content": tool_ctx})
        # Trim history to last N turns.
        messages.extend(self._history[-2 * self.config.history_turns:])
        messages.append({"role": "user", "content": utterance})

        try:
            content = (self._call_groq(messages) if self._provider == "groq"
                       else self._call_ollama(messages))
            data = json.loads(content)
            intent = Intent(
                intent_type=data.get("intent", "chatter"),
                destination=data.get("destination"),
                reply=data.get("reply") or "I'm not sure how to answer that.",
                raw=content,
            )
            self._history.append({"role": "user", "content": utterance})
            self._history.append({"role": "assistant", "content": content})
            return intent
        except Exception as e:
            print(f"[IntentClassifier] classify error: {e}")
            return Intent("chatter", None, "Sorry, I didn't catch that.", "")

    def reset_history(self) -> None:
        self._history = []

    # ---- LLM transports --------------------------------------------
    def _call_groq(self, messages: List[dict]) -> str:
        resp = self._client.chat.completions.create(
            model=self._model,
            response_format={"type": "json_object"},
            temperature=self.config.llm_temperature,
            max_tokens=200,
            messages=messages,
        )
        return resp.choices[0].message.content

    def _call_ollama(self, messages: List[dict]) -> str:
        resp = self._client.chat(
            model=self._model,
            format="json",
            options={"temperature": self.config.llm_temperature,
                     "num_predict": 200},
            messages=messages,
        )
        return resp["message"]["content"]

    # ---- Live tool context (weather, time) -------------------------
    def _build_tool_context(self, utterance: str) -> Optional[str]:
        u = utterance.lower()
        facts: List[str] = []

        if (self.config.enable_weather_tool
                and any(k in u for k in
                        ["weather", "temperature", "rain", "sun", "cold",
                         "hot", "wind", "snow"])):
            try:
                facts.append(self._fetch_weather())
            except Exception as e:
                facts.append(f"Weather data unavailable: {e}")

        if (self.config.enable_time_tool
                and any(k in u for k in
                        ["time", "what time", "clock", "date", "day"])):
            now = datetime.now()
            facts.append(
                f"Current local time: {now.strftime('%A %d %B, %H:%M')}"
            )

        if not facts:
            return None
        return "Real-time facts you may use in your reply:\n- " + \
               "\n- ".join(facts)

    def _fetch_weather(self) -> str:
        url = (
            "https://api.open-meteo.com/v1/forecast"
            f"?latitude={self.config.location_lat}"
            f"&longitude={self.config.location_lon}"
            "&current=temperature_2m,weather_code,wind_speed_10m"
        )
        with urllib.request.urlopen(url, timeout=3) as resp:
            data = json.loads(resp.read())
        cur = data["current"]
        desc = _WEATHER_CODES.get(cur.get("weather_code"), "unknown")
        return (f"Weather in {self.config.location_name}: {desc}, "
                f"{cur['temperature_2m']} degrees C, "
                f"wind {cur['wind_speed_10m']} km/h.")


# -----------------------------------------------------------------------------
# Text-to-speech (SAPI by default; Edge-TTS optional for natural voices)
# -----------------------------------------------------------------------------
class Speaker:
    def __init__(self, config: VoiceConfig):
        self.config = config
        self._mode = "off"
        self._engine = None
        self._edge_tts = None
        self._pygame = None
        self._audio_queue: Optional["queue.Queue[bytes]"] = None
        self._tts_cache: dict = {}   # text → pre-synthesised .mp3 tmp path
        if not config.tts_enabled:
            return

        if config.tts_provider == "edge":
            self._init_edge()
        if self._mode == "off":
            self._init_sapi()

    def bind_audio_queue(self, q: "queue.Queue[bytes]") -> None:
        """Give the speaker a reference to the mic queue so it can mute
        itself while speaking (prevents Jarvis from hearing its own voice)."""
        self._audio_queue = q

    # ---- Pre-synthesis cache (Edge-TTS only) -----------------------
    def prime(self, phrases: list) -> None:
        """Pre-synthesize a list of known phrases in a background thread so
        the first playback is instant.  Only does anything in edge mode."""
        if self._mode != "edge":
            return
        threading.Thread(
            target=self._prime_worker, args=(phrases,), daemon=True
        ).start()

    def _prime_worker(self, phrases: list) -> None:
        for text in phrases:
            if text and text not in self._tts_cache:
                try:
                    path = self._synthesise_to_file(text)
                    self._tts_cache[text] = path
                    print(f"[Speaker] Pre-cached: '{text}'")
                except Exception as e:
                    print(f"[Speaker] Pre-cache failed for '{text}': {e}")

    def _drain_mic(self) -> None:
        """Throw away any audio that accumulated while TTS was playing."""
        if self._audio_queue is None:
            return
        drained = 0
        while True:
            try:
                self._audio_queue.get_nowait()
                drained += 1
            except queue.Empty:
                break
        if drained:
            print(f"[Speaker] Drained {drained} stale mic chunks after TTS.")

    def _init_sapi(self) -> None:
        try:
            import pyttsx3
            self._engine = pyttsx3.init()
            self._engine.setProperty("rate", self.config.tts_rate)
            if self.config.tts_voice:
                voices = self._engine.getProperty("voices")
                target = self.config.tts_voice.lower()
                for v in voices:
                    if target in v.name.lower():
                        self._engine.setProperty("voice", v.id)
                        print(f"[Speaker] Voice: {v.name}")
                        break
                else:
                    print(f"[Speaker] Voice '{self.config.tts_voice}' not "
                          f"found. Available: "
                          f"{[v.name for v in voices]}")
            self._mode = "sapi"
            print("[Speaker] TTS ready (pyttsx3 / SAPI).")
        except Exception as e:
            print(f"[Speaker] SAPI unavailable: {e}")

    def _init_edge(self) -> None:
        try:
            import edge_tts          # noqa: F401
            import pygame
            pygame.mixer.init()
            self._pygame = pygame
            self._mode = "edge"
            print(f"[Speaker] TTS ready (Edge-TTS, voice="
                  f"{self.config.edge_voice}).")
        except Exception as e:
            print(f"[Speaker] Edge-TTS unavailable: {e}. "
                  "Falling back to SAPI.")

    def say(self, text: Optional[str], drain: bool = True) -> None:
        """Speak *text*.

        Parameters
        ----------
        drain:
            When True (default), flush any microphone audio that
            accumulated while Jarvis was speaking so Vosk doesn't
            mis-transcribe the TTS output as a new command.

            Set to False for very short one-word acks (e.g. the AWOKEN
            "Yes, sir?" response on a headset) where the earphone has
            negligible acoustic bleed into the mic and we want to keep
            any user audio that arrived in the queue (e.g. an inline
            command spoken right after the wake word).
        """
        if not text or not text.strip():
            return
        print(f"[Speaker] >>> {text}")
        if self._mode == "edge":
            self._say_edge(text)
        elif self._mode == "sapi" and self._engine is not None:
            try:
                self._engine.say(text)
                self._engine.runAndWait()
            except Exception as e:
                print(f"[Speaker] SAPI error: {e}")
        if drain:
            self._drain_mic()

    def _synthesise_to_file(self, text: str) -> str:
        """Synthesise *text* to a new temp .mp3 and return the path."""
        import tempfile, edge_tts
        tmp_fd, tmp_path = tempfile.mkstemp(suffix=".mp3", prefix="jarvis_tts_")
        os.close(tmp_fd)

        async def _synth():
            await edge_tts.Communicate(text, self.config.edge_voice).save(tmp_path)

        loop = asyncio.new_event_loop()
        try:
            loop.run_until_complete(_synth())
        finally:
            loop.close()
        return tmp_path

    def _say_edge(self, text: str) -> None:
        # Check pre-synthesis cache first — cache hit = zero network wait.
        cached_path = self._tts_cache.get(text)
        tmp_path: Optional[str] = None
        owns_file = False

        try:
            if cached_path and os.path.exists(cached_path):
                tmp_path = cached_path
            else:
                tmp_path = self._synthesise_to_file(text)
                owns_file = True

            self._pygame.mixer.music.load(tmp_path)
            self._pygame.mixer.music.play()
            while self._pygame.mixer.music.get_busy():
                self._pygame.time.wait(50)
            self._pygame.mixer.music.unload()
        except Exception as e:
            print(f"[Speaker] Edge-TTS error: {e} - falling back to SAPI.")
            self._say_sapi_fallback(text)
        finally:
            if owns_file and tmp_path:
                try:
                    os.remove(tmp_path)
                except OSError:
                    pass

    def _say_sapi_fallback(self, text: str) -> None:
        """Emergency fallback: use SAPI even when edge is the preferred mode."""
        try:
            import pyttsx3
            eng = pyttsx3.init()
            eng.setProperty("rate", self.config.tts_rate)
            eng.say(text)
            eng.runAndWait()
        except Exception as e2:
            print(f"[Speaker] SAPI fallback also failed: {e2}")


# -----------------------------------------------------------------------------
# Confirmation window (REQ-UI-FR05)
# -----------------------------------------------------------------------------
@dataclass
class Hit:
    phrase: str
    location_id: str
    confidence: float
    raw_text: str


class ConfirmationWindow:
    def __init__(self, seconds: float = 2.0):
        self.seconds = seconds

    def confirm(self, hit: Hit) -> bool:
        if hit.location_id == "EMERGENCY_STOP":
            print("\n[Confirm] EMERGENCY STOP - dispatching immediately.")
            return True
        print(f"\n>>> Driving to {hit.location_id} ('{hit.phrase}') "
              f"in {self.seconds:.1f}s. Press ENTER to CANCEL.")
        cancelled = threading.Event()

        def watch_stdin():
            try:
                sys.stdin.readline()
                cancelled.set()
            except Exception:
                pass

        threading.Thread(target=watch_stdin, daemon=True).start()
        deadline = time.time() + self.seconds
        while time.time() < deadline:
            if cancelled.is_set():
                print("[Confirm] Cancelled by user.")
                return False
            time.sleep(0.05)
        print("[Confirm] Confirmed - sending to pathfinding.")
        return True


# -----------------------------------------------------------------------------
# Payload emitter
# -----------------------------------------------------------------------------
class PayloadEmitter:
    def __init__(self, transport: Optional[Callable[[dict], None]] = None):
        self.transport = transport or self._default_transport

    @staticmethod
    def _default_transport(payload: dict) -> None:
        print("[PayloadEmitter] >>>")
        print(json.dumps(payload, indent=2))

    def emit(self, hit: Hit, mode: str = "VOICE",
             confirmed: bool = True) -> None:
        payload = {
            "mode": mode,
            "destination_id": hit.location_id,
            "destination_phrase": hit.phrase,
            "confidence": round(hit.confidence, 3),
            "confirmed": confirmed,
            "timestamp": time.time(),
        }
        self.transport(payload)


# -----------------------------------------------------------------------------
# Observer hook
# -----------------------------------------------------------------------------
class VoiceObserver:
    """Override any subset of these in your GUI / logger / test harness."""

    def on_state(self, state: str) -> None: ...
    def on_partial(self, text: str) -> None: ...
    def on_final(self, text: str) -> None: ...
    def on_reply(self, text: str) -> None: ...
    def on_address(self, address: str) -> None: ...
    def on_show_map(self) -> None: ...
    def on_hide_map(self) -> None: ...
    def on_navigate(self, payload: dict) -> None: ...


# -----------------------------------------------------------------------------
# Orchestrator
# -----------------------------------------------------------------------------
class VoiceController:
    def __init__(
        self,
        config: Optional[VoiceConfig] = None,
        observer: Optional[VoiceObserver] = None,
    ):
        self.config = config or VoiceConfig()
        self.observer = observer or VoiceObserver()
        self.audio = AudioStream(self.config)
        self.engine = VoskEngine(self.config)
        self.wake_gate = WakeWordGate(self.config)
        self.intent_classifier = IntentClassifier(self.config)
        self.speaker = Speaker(self.config)
        self.speaker.bind_audio_queue(self.audio.queue)
        self.confirmer = ConfirmationWindow(self.config.confirmation_seconds)
        self.emitter = PayloadEmitter(transport=self._on_payload)
        self.gender_detector = (
            GenderDetector(sample_rate=self.config.sample_rate)
            if self.config.enable_gender_detection else None
        )

        # Fast keyword navigator (bypass LLM for drive commands)
        self.fast_navigator = (
            FastNavigator(self.config.destinations)
            if self.config.fast_navigate else None
        )
        if self.fast_navigator:
            print("[FastNavigator] Active — navigate commands bypass LLM.")

        # Software noise gate (second layer after Plantronics hardware DSP)
        self.noise_gate = (
            NoiseGate(
                threshold_db=self.config.noise_gate_threshold_db,
                hold_frames=self.config.noise_gate_hold_frames,
            )
            if self.config.noise_gate_enabled else None
        )
        if self.noise_gate:
            print(f"[NoiseGate] Active — threshold {self.config.noise_gate_threshold_db} dBFS, "
                  f"hold {self.config.noise_gate_hold_frames} frames.")

        # Neural denoiser (third layer, DeepFilterNet3).
        # Disabled if the noise gate is off because the gate's is_speech
        # flag drives which chunks are sent through enhancement.
        self.denoiser = (
            DeepFilterDenoiser(sample_rate=self.config.sample_rate)
            if (self.config.spectral_denoiser_enabled
                and self.noise_gate is not None)
            else None
        )

        self._address: str = self.config.default_address
        self._map_visible: bool = False
        self._running = False
        self.observer.on_address(self._address)
        self.observer.on_state("STANDING BY")

        # Pre-synthesise common phrases so the first TTS response is
        # instant (no Edge-TTS network round-trip on the critical path).
        _prime_phrases = [
            f"Yes, {self._address}?",                          # AWOKEN ack
            f"Stopping immediately, {self._address}.",         # emergency stop
            f"On my way, {self._address}.",                    # confirm yes
            f"Understood, {self._address}. Let me know when you're ready.",
            f"No confirmation received, {self._address}. Staying put.",
            f"Sorry, {self._address} — yes or no?",
        ]
        for dest in self.config.destinations:
            if dest != "stop":
                _prime_phrases.append(
                    f"Shall I take you to {dest.title()}, {self._address}?"
                )
                _prime_phrases.append(
                    f"On my way to {dest.title()}, {self._address}."
                )
        self.speaker.prime(_prime_phrases)

    # -- payload bridge so the GUI sees navigations too ---------------
    def _on_payload(self, payload: dict) -> None:
        try:
            self.observer.on_navigate(payload)
        finally:
            print("[PayloadEmitter] >>>")
            print(json.dumps(payload, indent=2))

    def stop(self) -> None:
        self._running = False

    def run(self) -> None:
        self.audio.start()
        self._running = True

        valid = list(self.config.destinations.keys())
        print(f"\n[VoiceController] Say '{self.config.wake_word.title()}' "
              f"to wake the chair. Valid destinations: {valid}")
        print("Press Ctrl+C to stop.\n")
        self._broadcast_state("LISTENING")

        last_partial = ""
        # Silence-gate committer — buffers commands until the user has
        # been silent for silence_commit_s seconds so fragments of a
        # sentence are joined before the pipeline processes them.
        _committer = (
            SilenceCommitter(self.config.silence_commit_s)
            if self.config.use_silence_gate else None
        )
        _commit_full: str = ""   # full Vosk transcript for the pending command
        try:
            while self._running:
                try:
                    pcm = self.audio.queue.get(timeout=0.2)
                except queue.Empty:
                    # No audio for 0.2 s — pure silence.  Let the committer
                    # decide whether the pending buffer is ready to dispatch.
                    if _committer is not None:
                        committed = _committer.tick(is_speech=False)
                        if committed:
                            self._handle_command(committed,
                                                 _commit_full or committed)
                            _commit_full = ""
                    continue

                # ---- Software noise gate + neural denoiser -------------
                # Pipeline: NoiseGate classifies the chunk as speech or
                # silence.  Speech chunks are enhanced by DeepFilterNet3
                # before Vosk sees them.  Silence chunks are skipped
                # (DeepFilterNet needs no noise reference — it's neural).
                is_speech: bool = True   # default when gate is disabled
                if self.noise_gate is not None:
                    pcm, is_speech = self.noise_gate.process(pcm)
                    if self.denoiser is not None:
                        if is_speech:
                            pcm = self.denoiser.process(pcm)
                        else:
                            self.denoiser.feed_noise(pcm)

                # Tick the committer on every chunk so silence detection
                # works at audio resolution (125 ms) rather than waiting
                # for the next queue.Empty timeout.
                if _committer is not None:
                    committed = _committer.tick(is_speech=is_speech)
                    if committed:
                        self._handle_command(committed,
                                             _commit_full or committed)
                        _commit_full = ""

                # Feed every chunk into the gender detector.
                if self.gender_detector is not None:
                    self.gender_detector.feed(pcm)
                    if self.gender_detector.analyse():
                        new_addr = self.gender_detector.address
                        if new_addr != self._address:
                            self._address = new_addr
                            print(f"[Gender ] Detected -> addressing as "
                                  f"'{self._address}'.")
                            self.observer.on_address(self._address)

                result = self.engine.feed(pcm)

                if result is None:
                    if self.config.show_partials:
                        partial = self.engine.partial().get("partial", "")
                        if partial and partial != last_partial:
                            print(f"\r[live ] {partial:<70}",
                                  end="", flush=True)
                            self.observer.on_partial(partial)
                            last_partial = partial
                    continue

                final_text = (result.get("text") or "").strip()

                # ---- Noise / hallucination guard -----------------------
                if final_text:
                    words = final_text.lower().split()
                    if len(words) == 1 and words[0] in _STT_NOISE_WORDS:
                        print(f"\r[noise ] noise-word rejected: '{final_text}'"
                              f"{' ' * 50}", end="", flush=True)
                        final_text = ""
                    else:
                        word_confs = [w.get("conf", 1.0)
                                      for w in result.get("result", [])]
                        if (word_confs
                                and (sum(word_confs) / len(word_confs))
                                < _STT_MIN_CONF):
                            print(f"\r[noise ] low-conf rejected: "
                                  f"'{final_text}'{' ' * 50}",
                                  end="", flush=True)
                            final_text = ""
                # --------------------------------------------------------

                if final_text:
                    print(f"\r[final] {final_text:<70}")
                    self.observer.on_final(final_text)
                else:
                    print("\r" + " " * 78 + "\r", end="")
                self.observer.on_partial("")
                last_partial = ""

                command = self.wake_gate.filter(final_text)
                if command is None:
                    continue
                if isinstance(command, _WakeAwoken):
                    # User said "Jarvis" alone (no command in the same
                    # utterance).  Discard any partial command buffered
                    # from a previous utterance so the committer starts
                    # fresh for this conversation window.
                    if _committer is not None:
                        _committer.reset()
                    _commit_full = ""
                    self._broadcast_state("AWAITING COMMAND")
                    reply = f"Yes, {self._address}?"
                    self.observer.on_reply(reply)
                    self._broadcast_state("SPEAKING")
                    self.speaker.say(reply, drain=False)
                    self._broadcast_state("AWAITING COMMAND")
                    continue

                # Emergency stop is time-critical — bypass the silence gate
                # and dispatch the command immediately without buffering.
                if _STOP_ONLY_RE.match(command):
                    self._handle_command(command, final_text)
                    if _committer is not None:
                        _committer.reset()
                    _commit_full = ""
                elif _committer is not None:
                    # All other commands: buffer and wait for silence_commit_s
                    # seconds of quiet before dispatching to the pipeline.
                    _committer.feed(command)
                    _commit_full = final_text
                else:
                    self._handle_command(command, final_text)

        except KeyboardInterrupt:
            print("\n[VoiceController] Shutting down (Ctrl+C).")
        finally:
            self.audio.stop()
            self._broadcast_state("STANDING BY")

    # -- observer helpers --------------------------------------------
    def _broadcast_state(self, state: str) -> None:
        try:
            self.observer.on_state(state)
        except Exception as e:
            print(f"[VoiceController] observer.on_state error: {e}")

    # -- voice confirmation -----------------------------------------
    def _voice_confirm(self, dest_display: str) -> bool:
        """
        Ask the user to confirm navigation verbally, then listen for a
        spoken yes or no reply.

        Jarvis says: "Shall I take you to <dest>, <address>?"
        User says  : "yes" / "go" / "okay" → returns True
                     "no" / "cancel" / "stop" → returns False
        Timeout    : returns False (safe default — chair stays put).

        The confirmation loop feeds audio through the noise gate and
        Vosk recogniser but bypasses the wake gate and LLM — it only
        looks for simple yes/no vocabulary.
        """
        question = f"Shall I take you to {dest_display}, {self._address}?"
        self._broadcast_state("SPEAKING")
        self.speaker.say(question)   # drain=True: flush any leftover audio
        self._broadcast_state("CONFIRMING")
        print(f"[Confirm] Listening for yes/no (timeout {self.config.confirm_timeout_s}s) …")

        deadline = time.time() + self.config.confirm_timeout_s
        while time.time() < deadline:
            try:
                pcm = self.audio.queue.get(timeout=0.2)
            except queue.Empty:
                continue

            # Apply the same gate + neural denoiser pipeline as the main loop.
            if self.noise_gate is not None:
                pcm, is_speech = self.noise_gate.process(pcm)
                if self.denoiser is not None:
                    if is_speech:
                        pcm = self.denoiser.process(pcm)
                    else:
                        self.denoiser.feed_noise(pcm)

            # Feed to gender detector while confirming (keeps pitch model fresh)
            if self.gender_detector is not None:
                self.gender_detector.feed(pcm)

            result = self.engine.feed(pcm)
            if result is None:
                # Show partial so the user sees they're being heard
                partial = self.engine.partial().get("partial", "")
                if partial:
                    print(f"\r[confirm live] {partial:<60}", end="", flush=True)
                continue

            text = (result.get("text") or "").strip().lower()
            if not text:
                continue

            print(f"\r[Confirm] Heard: '{text}'{' ' * 50}")

            # Match yes
            words = set(text.split())
            if words & _CONFIRM_YES or any(y in text for y in _CONFIRM_YES):
                self.speaker.say(f"On my way, {self._address}.", drain=False)
                return True

            # Match no
            if words & _CONFIRM_NO or any(n in text for n in _CONFIRM_NO):
                self.speaker.say(
                    f"Understood, {self._address}. Let me know when you're ready.",
                    drain=False,
                )
                return False

            # Heard something but it wasn't a clear yes/no — prompt once more
            self.speaker.say(
                f"Sorry, {self._address} — yes or no?", drain=False
            )

        # Timed out — stay put for safety
        print("[Confirm] Timed out — cancelling navigation.")
        self.speaker.say(
            f"No confirmation received, {self._address}. Staying put.",
            drain=False,
        )
        return False

    # -- command pipeline -------------------------------------------
    def _handle_command(self, command: str, full_transcript: str) -> None:
        # ---- Fast path: keyword router (no LLM, ~0 ms) ----
        if self.fast_navigator is not None:
            fast_match = self.fast_navigator.match(command, address=self._address)
            if fast_match:
                dest_key, loc_id, reply = fast_match
                print(f"[FastNav] '{dest_key}' → {loc_id}  (LLM skipped)")
                hit = Hit(phrase=dest_key, location_id=loc_id,
                          confidence=1.0, raw_text=full_transcript)
                self.observer.on_reply(reply)

                if loc_id == "EMERGENCY_STOP":
                    self._broadcast_state("SPEAKING")
                    self.speaker.say(reply)
                    self.emitter.emit(hit)
                    self.wake_gate.end_conversation()
                    self.intent_classifier.reset_history()
                    self._broadcast_state("LISTENING")
                    return

                # Voice confirmation before moving
                confirmed = self._voice_confirm(dest_key.title())
                if confirmed:
                    self.emitter.emit(hit)
                self.wake_gate.end_conversation()
                self.intent_classifier.reset_history()
                self._broadcast_state("LISTENING")
                return

        # ---- Slow path: LLM classification (~200–500 ms) ----
        # THINKING is shown during the LLM call for all intents.  After
        # the call, if the intent is anything other than a question
        # (navigate, show_map, hide_map, goodbye, chatter) the state is
        # cleared immediately — the orb should only keep spinning when
        # Jarvis is genuinely formulating a spoken answer.
        self._broadcast_state("THINKING")
        intent = self.intent_classifier.classify(command, address=self._address)
        print(f"[Intent ] {intent.raw or '(no LLM)'}")
        self.observer.on_reply(intent.reply or "")

        if intent.intent_type != "question":
            self._broadcast_state("AWAITING COMMAND")

        if intent.intent_type == "navigate":
            self._handle_navigate(intent, full_transcript)
        elif intent.intent_type == "show_map":
            self._broadcast_state("SPEAKING")
            self.speaker.say(intent.reply)
            self._map_visible = True
            self.observer.on_show_map()
            self._broadcast_state("AWAITING COMMAND")
        elif intent.intent_type == "hide_map":
            self._broadcast_state("SPEAKING")
            self.speaker.say(intent.reply)
            self._map_visible = False
            self.observer.on_hide_map()
            self._broadcast_state("AWAITING COMMAND")
        elif intent.intent_type == "question":
            self._broadcast_state("SPEAKING")
            self.speaker.say(intent.reply)
            self._broadcast_state("AWAITING COMMAND")
        elif intent.intent_type == "goodbye":
            self._broadcast_state("SPEAKING")
            self.speaker.say(intent.reply)
            if self._map_visible:
                self._map_visible = False
                self.observer.on_hide_map()
            self.wake_gate.end_conversation()
            self.intent_classifier.reset_history()
            self._broadcast_state("LISTENING")
        else:  # chatter
            self._broadcast_state("SPEAKING")
            self.speaker.say(intent.reply)
            self._broadcast_state("AWAITING COMMAND")

    def _handle_navigate(self, intent: Intent, full_transcript: str) -> None:
        """LLM-classified navigate fallback (used when FastNavigator didn't match)."""
        dest = (intent.destination or "").lower().strip()
        loc_id = self.config.destinations.get(dest)
        if loc_id is None:
            self._broadcast_state("SPEAKING")
            self.speaker.say(
                f"I don't know a destination called {dest or 'that'}, "
                f"{self._address}."
            )
            self._broadcast_state("AWAITING COMMAND")
            return

        hit = Hit(phrase=dest, location_id=loc_id,
                  confidence=0.95, raw_text=full_transcript)

        if loc_id == "EMERGENCY_STOP":
            self._broadcast_state("SPEAKING")
            self.speaker.say(f"Stopping immediately, {self._address}.")
            self.emitter.emit(hit)
            self.wake_gate.end_conversation()
            self.intent_classifier.reset_history()
            self._broadcast_state("LISTENING")
            return

        # Voice confirmation before moving
        confirmed = self._voice_confirm(dest.title())
        if confirmed:
            self.emitter.emit(hit)
        self.wake_gate.end_conversation()
        self.intent_classifier.reset_history()
        self._broadcast_state("LISTENING")


# -----------------------------------------------------------------------------
# Entrypoint (CLI fallback - prefer running main.py for the GUI)
# -----------------------------------------------------------------------------
def main() -> None:
    # AudioStream.list_devices(); return   # uncomment to find mic index

    config = VoiceConfig(
        # Device 24 = Headset Microphone (Plantronics BT300M), Windows WASAPI
        # WASAPI gives lowest latency.  Use AudioStream.list_devices() to confirm.
        input_device=24,
        tts_provider="edge",
        edge_voice="en-GB-ThomasNeural",
        fast_navigate=True,
        # Noise gate + spectral denoiser are now ON by default in
        # VoiceConfig.  See section 2.1 of SKILL.md for the rationale.
    )
    VoiceController(config).run()


if __name__ == "__main__":
    main()
