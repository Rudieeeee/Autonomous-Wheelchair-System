"""
voice_control.py — Voice Control Subsystem for Smart Wheelchair BAP
===================================================================

Wake-word activated, LLM-gated, conversational voice control with
optional GUI-friendly observer hooks.

Behavior:
  • Idle until the user says "Jarvis".
  • On wake, Jarvis replies "Yes, Master?" and stays in conversation
    mode for ~30 s. Every new utterance refreshes the timer, so the
    user can have a real back-and-forth without repeating the wake word.
  • Each utterance is first checked against the FastNavigator (zero-
    latency keyword matcher). If it matches a navigation command, the
    chair moves immediately — NO LLM call needed.
  • Non-navigate intents are classified by an LLM (Groq, free hosted):
        show_map  — bring up the on-screen venue map
        hide_map  — dismiss the map
        question  — answer it aloud (live weather/time injected as context)
        chatter   — false alarm or speech aimed at someone else; Jarvis
                    goes back to idle and waits for the wake word again
        goodbye   — end the conversation, return to idle.
  • The chair only moves on "navigate". Mentioning a place is not enough.
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
import concurrent.futures
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

# How close a spoken word must be to "jarvis" (difflib ratio, 0-1) before the
# wake gate treats it as the wake word even though it is not in the alias list.
# 0.8 catches one-or-two-letter slips ("jarvas", "jervis", "darvis") while
# staying well clear of ordinary words so the chair does not false-wake.
_WAKE_FUZZY_THRESHOLD: float = 0.8


def _strip_edge_noise_words(text: str) -> str:
    """
    Trim filler words like a dangling "the" off the front and back of a
    final transcript.

    Vosk often tacks a trailing "the" or "a" onto the end of an utterance
    when it is still deciding whether more words are coming.  That dangling
    word holds the live partial open longer before it commits to a final,
    and it leaves a stray "the" sitting on screen after the user has
    finished speaking.  We keep the meaningful words in the middle and drop
    the noise words that bracket them, so "take me to lab a the" becomes
    "take me to lab a".  A line that is nothing but noise words collapses
    to empty and gets rejected upstream.
    """
    words = text.split()
    while words and words[0].lower() in _STT_NOISE_WORDS:
        words.pop(0)
    while words and words[-1].lower() in _STT_NOISE_WORDS:
        words.pop()
    return " ".join(words)

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

# Emergency motion stop.  Matches a bare "stop" and the natural ways a
# rider tells the chair to quit moving ("stop driving", "stop moving",
# "stop the chair", "halt").  These all halt the chair immediately through
# the e-stop path.  The object words are kept deliberately narrow so this
# never swallows the operator commands "stop mapping / localization /
# navigation" or the "stop listening" sleep command, which route to their
# own handlers below.  A leading "jarvis" and a trailing "please" / "now"
# are tolerated so "jarvis stop driving" and "stop please" both match.
_STOP_ONLY_RE = re.compile(
    r'^\s*(?:jarvis[\s,]+)?(?:please\s+)?(?:emergency\s+)?'
    r'(?:'
    r'stop(?:\s+(?:right\s+now|now|driving|moving|going|the\s+(?:chair|wheelchair|car)))?'
    r'|halt'
    r'|brake'
    r'|freeze'
    r')'
    r'(?:[\s,]+(?:please|now|jarvis))?[.!]?\s*$',
    re.IGNORECASE,
)

# Explicit "go to sleep" command — the spoken twin of the wake word.
# Lets the user dismiss Jarvis on demand ("go to sleep", "stop listening",
# "naptime", "shut down") instead of waiting for the conversation window to
# time out.  An optional leading/trailing "jarvis" is allowed so "jarvis go
# to sleep" and "go to sleep jarvis" both match.  This is NOT an emergency
# stop — it only ends the listening session, it does not touch the chair.
_SLEEP_RE = re.compile(
    r'^\s*(?:jarvis[\s,]+)?'
    r'(?:'
    r'stop\s+listening|'
    r'go\s+(?:back\s+)?to\s+sleep|'
    r'nap\s*time|'
    r'shut\s*down|'
    r'power\s+down|'
    r'sleep(?:\s+now)?'
    r')'
    r'(?:[\s,]+jarvis)?[.!]?\s*$',
    re.IGNORECASE,
)

# ---------------------------------------------------------------------------
# Operator / developer commands (resolved locally, no LLM)
# ---------------------------------------------------------------------------
# These were the slowest and least reliable spoken commands because each one
# had to survive a Vosk transcript and a Groq classification before the
# matching launcher button fired.  A garbled "stop navigation" came back as
# chatter and dropped the whole conversation.  Matching them here makes them
# react the same way the fast navigation path does, with no network round
# trip and no dependence on the model.  Order matters: the broad "stop all"
# is checked before the per-subsystem stops so "stop all navigation" is not
# read as a navigation-only stop.  Vosk often clips "localization" to
# "localize" / "local", so both are accepted.
_DEV_CMD_PATTERNS = [
    ("stop_all", re.compile(
        r'\b(?:stop|shut\s*down|shutdown|kill|end|quit)\s+(?:everything|all)\b'
        r'|\bshut\s+everything\s+down\b'
        r'|\bstop\s+all\s+systems\b', re.IGNORECASE)),
    ("start_all", re.compile(
        r'\b(?:start|launch|boot|bring\s+up|begin)\s+(?:everything|all)\b'
        r'|\blaunch\s+everything\b', re.IGNORECASE)),
    ("stop_mapping", re.compile(
        r'\b(?:stop|end|quit|halt|finish)\s+(?:the\s+)?mapping\b', re.IGNORECASE)),
    ("start_mapping", re.compile(
        r'\b(?:start|begin|launch|initialize|initialise)\s+(?:the\s+)?mapping\b',
        re.IGNORECASE)),
    ("stop_localization", re.compile(
        r'\b(?:stop|end|quit|halt)\s+(?:the\s+)?local(?:ization|isation|ize|ise|izing|ising)?\b',
        re.IGNORECASE)),
    ("start_localization", re.compile(
        r'\b(?:start|begin|launch|initialize|initialise)\s+(?:the\s+)?'
        r'local(?:ization|isation|ize|ise|izing|ising)?\b', re.IGNORECASE)),
    ("stop_navigation", re.compile(
        r'\b(?:stop|end|quit|halt|cancel)\s+(?:the\s+)?navigation\b', re.IGNORECASE)),
    ("start_navigation", re.compile(
        r'\b(?:start|begin|launch|initialize|initialise)\s+(?:the\s+)?navigation\b',
        re.IGNORECASE)),
]


def _match_dev_command(text: str) -> Optional[str]:
    """Resolve an operator command from a transcript without the LLM.

    Returns the dev-command string (the same value the LLM would put in
    ``Intent.destination``) or ``None`` if nothing matches and the command
    should fall through to the normal pipeline.
    """
    for cmd, pattern in _DEV_CMD_PATTERNS:
        if pattern.search(text):
            return cmd
    return None


# Spoken acknowledgements for the locally-resolved dev commands.  "{address}"
# is filled in at runtime so they match the address used everywhere else.
_DEV_CMD_REPLIES: dict = {
    "start_mapping":       "Starting mapping, {address}.",
    "stop_mapping":        "Stopping mapping, {address}.",
    "start_localization":  "Starting localization, {address}.",
    "stop_localization":   "Stopping localization, {address}.",
    "start_navigation":    "Starting navigation, {address}.",
    "stop_navigation":     "Stopping navigation, {address}.",
    "start_all":           "Starting all systems, {address}.",
    "stop_all":            "Stopping all systems, {address}.",
}

# ---------------------------------------------------------------------------
# Map show / hide commands (resolved locally, no LLM)
# ---------------------------------------------------------------------------
# "show me the map" is the demo's most-used command, yet it was routed to the
# LLM where a slightly garbled transcript came back as "chatter" and dropped
# the whole conversation.  These patterns resolve the map intent the same way
# FastNavigator resolves navigation, so a clear map phrase never depends on
# the network or the model's mood.  A show verb plus a map noun anywhere in
# the utterance is enough.
_MAP_SHOW_RE = re.compile(
    r'\b(?:show|see|view|open|display|bring\s+up|pull\s+up|put\s+up|give)\b'
    r'[\w\s]*\b(?:map|floor\s*plan|layout)\b',
    re.IGNORECASE,
)
_MAP_HIDE_RE = re.compile(
    r'\b(?:hide|close|dismiss|remove|clear|take\s+away|get\s+rid\s+of)\b'
    r'[\w\s]*\b(?:map|floor\s*plan|layout)\b',
    re.IGNORECASE,
)

# Vosk reliably mangles "the map" into "them up" ("map" is short and out of
# context), so "show me the map" arrives as "show me them up".  The bigram is
# corrected before any matcher sees it so both the map regex above and the LLM
# read the phrase the user actually said.
_MAP_PHRASE_RE = re.compile(r'\bthem\s+up\b', re.IGNORECASE)

# ---------------------------------------------------------------------------
# Homophone correction
# ---------------------------------------------------------------------------
# Vosk regularly swaps the important command verbs for similar-sounding
# everyday words ("wake me to the exit" instead of "take me to the exit").
# These swaps are corrected on the whole-word level before the command
# reaches FastNavigator / the LLM, so a single misheard verb does not lose
# an otherwise clear instruction.  The keys are what Vosk tends to output,
# the values are what the user actually said.
_HOMOPHONE_SUBS: dict = {
    "wake":   "take",
    "break":  "take",
    "road":   "go",
    # "elevator" is a long word Vosk often clips to "elevate".
    "elevate": "elevator",
    # "please" is short and easily clipped; "police" is its most common
    # mishearing, so it is folded back to "please" to keep politeness and
    # the "please stop" emergency phrasing working.
    "police": "please",
    "pleas":  "please",
}
_HOMOPHONE_RE = re.compile(
    r'\b(' + '|'.join(re.escape(k) for k in _HOMOPHONE_SUBS) + r')\b',
    re.IGNORECASE,
)

# Multi-word mishearings, fixed before the single-word swaps above so the
# leftover word ("their"/"there") is not stranded.  Vosk splits "elevator"
# into "elevate their" / "elevate there" surprisingly often.
_PHRASE_SUBS: dict = {
    "elevate their": "elevator",
    "elevate there": "elevator",
}
_PHRASE_RE = re.compile(
    r'\b(' + '|'.join(re.escape(k) for k in _PHRASE_SUBS) + r')\b',
    re.IGNORECASE,
)

# Floor numbers spoken straight after "elevator" are routinely misheard as
# their function-word homophones ("elevator to" → "elevator two", "elevator
# for" → "elevator four").  These are only corrected when they follow
# "elevator" (optionally across a filler word like "number"/"floor") so the
# everyday "to"/"for" elsewhere in a sentence is left untouched.
_ELEVATOR_NUMS: dict = {
    "to": "two", "too": "two", "tu": "two",
    "for": "four", "fore": "four",
}
_ELEVATOR_FILLERS: frozenset = frozenset({"number", "floor", "level", "the", "on"})


def _fix_elevator_numbers(text: str) -> str:
    """Correct floor-number homophones spoken right after "elevator"."""
    if "elevator" not in text:
        return text
    out: list = []
    armed = False
    for tok in text.split():
        if armed and tok in _ELEVATOR_NUMS:
            out.append(_ELEVATOR_NUMS[tok])
            armed = False
            continue
        if armed and tok in _ELEVATOR_FILLERS:
            out.append(tok)            # stay armed across a filler word
            continue
        out.append(tok)
        armed = (tok == "elevator")
    return " ".join(out)


def _normalise_homophones(text: str) -> str:
    """Replace known Vosk mishearings with the intended command word.

    Whole-word, case-insensitive.  Returns lower-cased text because every
    downstream matcher (FastNavigator, wake gate, confirmation) already
    works in lower case.  Runs phrase fixes first, then the single-word
    swaps, then the elevator floor-number fix.
    """
    if not text:
        return text
    text = text.lower()
    text = _MAP_PHRASE_RE.sub("the map", text)
    text = _PHRASE_RE.sub(lambda m: _PHRASE_SUBS[m.group(1)], text)
    text = _HOMOPHONE_RE.sub(lambda m: _HOMOPHONE_SUBS[m.group(1)], text)
    text = _fix_elevator_numbers(text)
    return text

# ---------------------------------------------------------------------------
# Voice confirmation word sets
# Used by VoiceController._voice_confirm() to interpret the user's yes/no.
# ---------------------------------------------------------------------------
import difflib

# Single-word answers. These also include common Vosk mishearings of the
# short words (e.g. "yes" often comes back as "yet" or "yas", "no" as "now")
# so a clear answer is not lost just because the recogniser slipped a letter.
_CONFIRM_YES_WORDS: frozenset = frozenset({
    "yes", "yeah", "yep", "yup", "ya", "yah", "yas", "yus", "yet", "yess",
    "sure", "okay", "ok", "okey", "kay", "go", "proceed", "confirm",
    "confirmed", "correct", "right", "affirmative", "please", "absolutely",
    "definitely", "yup", "yepp",
})
_CONFIRM_NO_WORDS: frozenset = frozenset({
    "no", "nope", "nah", "naw", "now", "cancel", "stop", "abort",
    "negative", "wait", "hold", "nevermind", "back", "actually",
})

# Multi-word answers, matched as a substring of the whole phrase so filler
# around them ("the no please thanks") still resolves.
_CONFIRM_YES_PHRASES: tuple = (
    "yes please", "go ahead", "do it", "let's go", "lets go", "of course",
    "go for it", "sounds good", "that's right", "thats right",
)
_CONFIRM_NO_PHRASES: tuple = (
    "no please", "no thanks", "never mind", "hold on", "do not",
    "don't", "forget it", "forget that", "not now", "stay put",
)


def _classify_confirmation(text: str) -> Optional[str]:
    """Return 'yes', 'no', or None for a spoken confirmation reply.

    Tolerant by design: it ignores filler words around the answer, accepts
    common Vosk mishearings, and falls back to a fuzzy match so a slightly
    garbled "yes"/"no" still lands. 'no' is checked before 'yes' so a mixed
    reply errs on the safe side (cancel). Works on partial transcripts too,
    so the chair can react the moment the answer appears.
    """
    if not text:
        return None
    text = text.lower().strip()
    if not text:
        return None

    # Phrases first (substring of the full reply).
    for phrase in _CONFIRM_NO_PHRASES:
        if phrase in text:
            return "no"
    for phrase in _CONFIRM_YES_PHRASES:
        if phrase in text:
            return "yes"

    words = text.split()

    # Exact single-word match.
    for w in words:
        if w in _CONFIRM_NO_WORDS:
            return "no"
        if w in _CONFIRM_YES_WORDS:
            return "yes"

    # Fuzzy backstop for anything the explicit lists missed.
    for w in words:
        if len(w) < 2:
            continue
        if difflib.get_close_matches(w, _CONFIRM_NO_WORDS, n=1, cutoff=0.8):
            return "no"
        if difflib.get_close_matches(w, _CONFIRM_YES_WORDS, n=1, cutoff=0.8):
            return "yes"

    return None


# Leading words to peel off a "no" reply when looking for a fresh command
# hidden behind it ("no, take me to lab a" -> "take me to lab a").
_NO_LEAD_WORDS: frozenset = _CONFIRM_NO_WORDS | frozenset({
    "thanks", "thank", "you", "please", "just", "well", "oh", "um", "uh",
    "not", "never", "mind", "nevermind", "instead", "rather",
    "and", "but", "then", "ok", "okay",
})


def _command_after_no(text: str) -> str:
    """Return any fresh command tucked behind a 'no' reply.

    Drops a leading run of negation and filler words and returns what is
    left, so "no take me to lab a" yields "take me to lab a" while a plain
    "no thanks" yields an empty string (pure cancel, nothing to do next).
    """
    if not text:
        return ""
    words = text.lower().split()
    i = 0
    while i < len(words) and words[i] in _NO_LEAD_WORDS:
        i += 1
    return " ".join(words[i:]).strip()

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
    # 1600 samples @ 16 kHz = 100 ms chunks — tighter audio loop for
    # faster endpoint detection.  Values below 1000 (62.5 ms) can starve
    # DeepFilterNet which buffers at 10 ms internally, so 100 ms is a
    # safe middle ground.
    block_size: int = 1600
    # Preferred device index for the Plantronics BT300M headset mic (WASAPI).
    # If this index is missing or not an input device, AudioStream falls back
    # to auto-detection by name, then the system default.
    # Set to None to always use the system default.
    input_device: Optional[int] = 24
    grammar_locked: bool = False
    show_partials: bool = True

    # ---- Wake word & conversation ---------------------------------------
    wake_word: str = "jarvis"
    # Vosk often mishears "Jarvis" as other English words because the name
    # is not in its everyday vocabulary.  "harvest" is by far the most
    # common substitution, so it is accepted as an alias for the wake word.
    # The rest are real mishearings observed in noisy open-day testing.
    wake_word_aliases: list = field(default_factory=lambda: [
        "harvest", "harvard", "nervous", "therapist", "jurors", "server", "service", "services", "carvers", "garbage",
        "jervis", "orvis", "drivers", "brothers",
    ])
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
    #
    # Tightened from −42 to −35 dBFS so the gate only opens for the loud
    # close-mic wearer.  The wearer still arrives at roughly −20, so there
    # is a 15 dB margin for quiet consonants in "Jarvis", while a neighbour
    # leaning in at −40 now stays shut out instead of slipping through.
    # If quiet starts of "Jarvis" ever get clipped, ease this back towards
    # −40.
    noise_gate_enabled: bool = True
    noise_gate_threshold_db: float = -35.0
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
    #
    # 0.6 s is short enough that the chair reacts quickly after the user
    # stops talking, while still bridging a normal mid-sentence pause.
    # Lower values (< 0.5 s) risk cutting off slow speakers. Raise it
    # back towards 1.0 s only if users find sentences are split mid-thought.
    use_silence_gate: bool = True
    silence_commit_s: float = 0.6

    # ---- Neural denoiser (DeepFilterNet3 before Vosk) ------------------
    # Third stage of the rejection pipeline.  Runs every speech chunk
    # through DeepFilterNet3, a full-band neural speech enhancement model
    # that suppresses both stationary noise and competing voices (babble,
    # crowd) — the exact scenario at an open-day demo.
    #
    # Requires the deepfilternet package.  Falls back to passthrough with
    # a printed warning if not installed.
    spectral_denoiser_enabled: bool = True
    # Post-filter is the extra suppression stage from the DeepFilterNet
    # demo.  On by default so competing voices get pushed down as hard as
    # the model allows.  Turn it off if the user's own voice starts to
    # sound thin or watery.
    deepfilter_post_filter: bool = True
    # Attenuation limit in dB.  None is the demo bar at maximum: noise is
    # suppressed with no cap.  Put a number here (e.g. 12.0) only if you
    # want to deliberately leave that many dB of background in.
    deepfilter_atten_lim_db: Optional[float] = None

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

    # ---- Barge-in (talk over Jarvis while he is speaking) --------------
    # When True, the Speaker keeps an ear on the microphone while it is
    # talking and cuts its own playback the moment the user starts speaking,
    # so the user can interrupt a reply instead of waiting for it to finish.
    # The captured speech is fed straight back into the recogniser so the
    # interrupting command is not lost.
    #
    # The trigger threshold is deliberately louder than the noise gate
    # (-42 dBFS) so residual crowd noise and the headset's own bleed do not
    # falsely interrupt Jarvis — only the close-boom user voice (~-20 dBFS)
    # crosses it.  barge_in_frames is how many consecutive 125 ms speech
    # chunks are needed to trigger (3 = ~375 ms), which rejects coughs and
    # single clicks.
    barge_in_enabled: bool = True
    barge_in_threshold_db: float = -30.0
    barge_in_frames: int = 3

    # ---- Address -------------------------------------------------------
    default_address: str = "Master"

    # ---- Destinations --------------------------------------------------
    # Only "stop" is built in. Every real destination is seeded at runtime
    # from the map markers (main.py reads map_points.json and calls
    # add_destination for each one), so the chair can only ever drive to a
    # location that exists as a marker on the map.
    destinations: dict = field(default_factory=lambda: {
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

    def __init__(self, sample_rate: int = 16000,
                 post_filter: bool = True,
                 atten_lim_db: Optional[float] = None) -> None:
        # post_filter is the extra suppression stage exposed in the
        # DeepFilterNet demo.  It costs a touch of speech naturalness and
        # buys the last few dB of rejection on competing voices, which is
        # exactly the trade we want at a crowded open day.
        #
        # atten_lim_db caps how hard noise is pushed down.  None means no
        # cap, i.e. the model suppresses as much as it can.  That is the
        # "bar at maximum" setting from the demo.  Set a number like 12.0
        # to deliberately leave that many dB of background in.
        self.sample_rate = sample_rate
        self.post_filter = post_filter
        self.atten_lim_db = atten_lim_db
        self._model = None
        self._df_state = None
        self._torch = None
        self._torchaudio = None
        self._available = False
        try:
            import sys
            import types
            import torch
            import torchaudio
            # Cap torch to 2 threads.  On a laptop CPU it otherwise grabs every
            # core for DeepFilterNet inference and starves Vosk, the GUI, and
            # the ROS2 spin loop, which makes the stalls worse.  2 threads is
            # plenty for real-time 100 ms chunks.
            try:
                torch.set_num_threads(2)
            except Exception:
                pass
            # torchaudio 2.x removed torchaudio.backend and the AudioMetaData
            # class. deepfilterlib 0.5.6 still does, at import time,
            #   from torchaudio.backend.common import AudioMetaData
            # (df.io, pulled in by df.enhance -> init_df). Recreate just enough
            # of that surface so the import resolves. We never call df's file IO
            # helpers (raw PCM in, torchaudio.functional.resample out), so a
            # lightweight stand-in class is all df needs.
            class AudioMetaData:  # mirrors the old torchaudio dataclass
                def __init__(self, sample_rate=0, num_frames=0, num_channels=0,
                             bits_per_sample=0, encoding="UNKNOWN"):
                    self.sample_rate = sample_rate
                    self.num_frames = num_frames
                    self.num_channels = num_channels
                    self.bits_per_sample = bits_per_sample
                    self.encoding = encoding

            if "torchaudio.backend" not in sys.modules:
                _backend = types.ModuleType("torchaudio.backend")
                _backend.__path__ = []          # marks it as a package
                _backend.__package__ = "torchaudio.backend"
                sys.modules["torchaudio.backend"] = _backend
                torchaudio.backend = _backend
            _common = types.ModuleType("torchaudio.backend.common")
            _common.AudioMetaData = AudioMetaData
            sys.modules["torchaudio.backend.common"] = _common
            sys.modules["torchaudio.backend"].common = _common
            from df import init_df
            # DeepFilterNet's logger asks git for the current commit while
            # it loads the model, via subprocess(["git", ...]).  It only
            # catches CalledProcessError, so on a machine where git is not
            # on PATH the call raises FileNotFoundError ([WinError 2] on
            # Windows) and takes the whole model load down with it — the
            # denoiser then silently falls back to passthrough and never
            # suppresses anything.  Stub the git lookup so the model loads
            # whether or not git is installed.
            import df.utils as _df_utils
            _df_utils.get_git_root = lambda: None
            self._torch = torch
            self._torchaudio = torchaudio
            self._model, self._df_state, _ = init_df(
                post_filter=self.post_filter
            )
            self._available = True
            lim = ("max" if self.atten_lim_db is None
                   else f"{self.atten_lim_db:g} dB cap")
            print(f"[DeepFilterDenoiser] Active — DeepFilterNet3 loaded "
                  f"(post-filter {'on' if self.post_filter else 'off'}, "
                  f"attenuation {lim}).")
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
            enhanced_48k = enhance(self._model, self._df_state, audio_48k,
                                   atten_lim_db=self.atten_lim_db)
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
        address: str = "Master",
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
        # Bounded queue: the recogniser loop stops draining the queue while
        # Jarvis is talking or waiting on the LLM, and an unbounded queue let
        # raw PCM pile up the whole time.  When the loop resumed it pushed the
        # entire backlog through DeepFilterNet at once, which is the lag burst
        # and the "everything I said arrives in one dump" behaviour.  Capping
        # the queue (~5 s at 100 ms blocks) and dropping the oldest chunk when
        # it is full means a backlog can never build.
        self.queue: "queue.Queue[bytes]" = queue.Queue(maxsize=50)
        self._stream: Optional[sd.RawInputStream] = None

    def _callback(self, indata, frames, time_info, status):
        if status:
            print(f"[AudioStream] {status}", file=sys.stderr)
        try:
            self.queue.put_nowait(bytes(indata))
        except queue.Full:
            # Drop the oldest chunk to make room for the newest.  Always keep
            # the freshest audio so Vosk works on what the user just said.
            try:
                self.queue.get_nowait()
                self.queue.put_nowait(bytes(indata))
            except (queue.Empty, queue.Full):
                pass

    @staticmethod
    def _find_device_by_name(name_fragment: str) -> Optional[int]:
        """Return the best-matching input device for name_fragment.

        Prefers WASAPI devices for lowest latency on Windows (sounddevice
        lists MME first, WASAPI last — so a higher index for the same
        physical mic = more likely to be the WASAPI endpoint).  Falls back
        to the first match if no WASAPI host API is found.
        """
        devices = sd.query_devices()
        try:
            hostapis = sd.query_hostapis()
            wasapi_idx = next(
                (i for i, h in enumerate(hostapis)
                 if "wasapi" in h["name"].lower()),
                None,
            )
        except Exception:
            wasapi_idx = None

        first_match: Optional[int] = None
        for i, dev in enumerate(devices):
            if (name_fragment.lower() in dev["name"].lower()
                    and dev["max_input_channels"] > 0):
                if wasapi_idx is not None and dev.get("hostapi") == wasapi_idx:
                    return i          # WASAPI match — best possible, take it
                if first_match is None:
                    first_match = i   # non-WASAPI fallback
        return first_match

    def start(self) -> None:
        device = self.config.input_device

        # Print all available input devices on startup so index mismatches are
        # immediately visible in the console.
        all_devs = sd.query_devices()
        print("[AudioStream] Available input devices:")
        for i, dev in enumerate(all_devs):
            if dev["max_input_channels"] > 0:
                print(f"  [{i}] {dev['name']}")

        # If a preferred device index is set, verify it still exists; if not
        # (Windows reshuffles indices when USB devices connect/disconnect),
        # fall back to looking up the Plantronics by name.
        if device is not None:
            try:
                info = sd.query_devices(device)
                if info["max_input_channels"] < 1:
                    raise ValueError("not an input device")
            except Exception:
                print(f"[AudioStream] Device {device} not available — "
                      "searching by name for Plantronics BT300M …")
                for fragment in ("Plantronics BT300M", "BT300M", "Plantronics",
                                 "Voyager", "Poly"):
                    device = self._find_device_by_name(fragment)
                    if device is not None:
                        break
                if device is None:
                    print("\n" + "!" * 70)
                    print("[AudioStream] WARNING: Plantronics BT300M NOT found.")
                    print("  Falling back to the SYSTEM DEFAULT mic (laptop / pulse).")
                    print("  The default mic is omnidirectional, so it hears")
                    print("  everyone in the room and the off-axis noise rejection")
                    print("  the system is designed around is GONE.  Plug in the")
                    print("  BT300M USB adapter (with the headset paired) and")
                    print("  re-run, or set VoiceConfig.input_device to the right")
                    print("  index from the device list printed above.")
                    print("!" * 70 + "\n")

        # Try the resolved device first, then fall back to the system default.
        # Without this outer try/except the voice thread died silently when the
        # stream open failed (e.g. wrong sample rate for a non-Plantronics mic).
        for attempt in (device, None):
            try:
                self._stream = sd.RawInputStream(
                    samplerate=self.config.sample_rate,
                    blocksize=self.config.block_size,
                    dtype="int16",
                    channels=1,
                    device=attempt,
                    callback=self._callback,
                )
                self._stream.start()
                dev_label = f"device {attempt}" if attempt is not None else "system default"
                print(f"[AudioStream] Capturing @ {self.config.sample_rate} Hz, "
                      f"block={self.config.block_size} samples "
                      f"({self.config.block_size / self.config.sample_rate * 1000:.0f} ms), "
                      f"{dev_label}")
                return
            except Exception as exc:
                print(f"[AudioStream] Failed to open device {attempt!r}: {exc}")
                if attempt is None:
                    print("[AudioStream] Could not open any input device — voice disabled.")
                    raise

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
                list(config.destinations.keys())
                + [config.wake_word] + list(config.wake_word_aliases)
                + ["[unk]"]
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

    def reset(self) -> None:
        """Clear any in-progress utterance so it does not bleed into the next
        command. Used after a confirmation is matched on a partial result."""
        try:
            self._recognizer.Reset()
        except Exception:
            pass


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
        # The wake word plus any known mishearings (e.g. "harvest" for
        # "Jarvis").  Each is matched the same way so the chair wakes on
        # either spelling without the user having to retry.
        self.wake_words = [self.wake_word] + [
            w.lower() for w in getattr(config, "wake_word_aliases", [])
        ]
        # Set form for O(1) exact lookups during the fuzzy fallback below.
        self._wake_word_set = set(self.wake_words)
        self.awake_timeout_s = config.awake_timeout_s
        self._awake_until = 0.0
        self._active = False

    @property
    def is_awake(self) -> bool:
        return self._active and time.time() < self._awake_until

    def _is_wake_word(self, word: str) -> bool:
        """True if a single spoken word is the wake word or a near-miss of it.

        The fixed alias list only covers mishearings we have already seen.
        Vosk invents a fresh one most sessions ("jarvas", "darvis", "jervis"),
        so each token is also compared phonetically to "jarvis" and accepted
        when it is close enough.  This is what lets the chair wake on a
        misheard name instead of forcing the user to repeat themselves.
        """
        w = word.strip(" ,.:;!?-'\"")
        if not w:
            return False
        if w in self._wake_word_set:
            return True
        return (difflib.SequenceMatcher(None, w, self.wake_word).ratio()
                >= _WAKE_FUZZY_THRESHOLD)

    def filter(self, transcript: str) -> Optional[Union[str, _WakeAwoken]]:
        text = transcript.lower().strip()
        if not text:
            return None

        # Wake on the first word that is the wake word, a known alias, or a
        # close mishearing of "jarvis" (see _is_wake_word).
        words = text.split()
        wake_idx = next(
            (i for i, w in enumerate(words) if self._is_wake_word(w)),
            None,
        )
        if wake_idx is not None:
            tail = " ".join(words[wake_idx + 1:]).lstrip(" ,.:;!?-")
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
    The default window is 0.6 s (see VoiceConfig.silence_commit_s).

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
user as "Master". Use it naturally inside the sentence; do not force
it into every reply.

Classify each user utterance into ONE of six intents:

  navigate  — an EXPLICIT drive command. The user is telling the chair
              to MOVE. Look for verbs of motion: "go to", "take me to",
              "bring me to", "drive to", "let's go to", "i want to go to",
              "head to". Naming a place WITHOUT such a verb is NOT a
              command. Destination MUST be one of the valid destinations.
              Any stop or halt phrase ("stop", "stop driving", "stop the
              chair", "halt") is NOT navigate. Classify it as dev_command
              with destination "emergency_stop".

  show_map  — the user wants to see the venue map. Triggers: "show me
              the map", "show the map", "open the map", "where am i",
              "where can i go", "what are my options", "let me see the
              floor plan".

  hide_map  — the user wants to dismiss the map: "close the map",
              "hide the map", "go back", "dismiss this", "okay close it".

  create_point — the user wants to mark a custom waypoint on the map.
              Triggers: "create a point", "add a point", "mark a location",
              "place a marker", "drop a pin", "save this spot", "add
              waypoint".

  dev_command — the operator wants to control a developer system function.
              Set destination to one of the sub-commands below.
              "start mapping" / "initialize mapping" / "begin mapping"  → "start_mapping"
              "stop mapping" / "end mapping"                            → "stop_mapping"
              "start localization" / "initialize localization"          → "start_localization"
              "stop localization"                                       → "stop_localization"
              "start navigation" / "begin navigation"                   → "start_navigation"
              "stop navigation"                                         → "stop_navigation"
              "start all" / "launch everything"                         → "start_all"
              "stop all" / "shut everything down"                       → "stop_all"
              "developer mode" / "dev mode" / "switch to developer"     → "mode_developer"
              "user mode" / "switch to user" / "visitor mode"          → "mode_user"
              "emergency stop" / "activate e-stop" / "e stop"          → "emergency_stop"
              "stop" / "stop driving" / "stop the chair" / "halt"       → "emergency_stop"

  question  — the user is asking you anything: facts, time, weather,
              jokes, opinions, status, small talk questions. Answer
              concisely and warmly (≤30 words). Use any "Real-time facts"
              context provided. Be honest if you don't know.

  goodbye   — the user is ending the chat: "thanks", "thank you", "bye",
              "that's all", "never mind", "goodbye", "stop talking",
              "we're done", "ok cool". If the map is currently visible,
              prefer hide_map instead.

  clarify   — you think you know which destination the user means but you
              are not certain, so you ask them to confirm. Set destination
              to the valid destination you are checking and phrase the
              reply as a yes/no question ("Did you mean lab a, Master?").
              The user will answer with a simple yes or no, so do not ask
              them to repeat the destination.

  chatter   — a false alarm: background noise, an unrelated remark, or
              speech clearly aimed at someone else rather than you. The
              wake word was caught but nothing here is for Jarvis. After a
              chatter turn Jarvis stops listening until the wake word is
              spoken again, so only use it when the utterance is genuinely
              not a question or a command. A place name spoken on its own
              (the user naming where they want to go) is NOT chatter — use
              clarify for that.

Valid destinations (use the exact phrasing — see "Runtime destinations" context).

Output ONLY a single JSON object, no prose, exactly this schema:
{
  "intent": "navigate" | "show_map" | "hide_map" | "create_point" | "dev_command" | "question" | "chatter" | "goodbye" | "clarify",
  "destination": "<one valid destination>" or null,
  "reply": "<short spoken response, <=30 words>"
}

Examples:
"take me to lab a"             -> {"intent":"navigate","destination":"lab a","reply":"On my way to lab A, Master."}
"bring me to the entrance"     -> {"intent":"navigate","destination":"entrance","reply":"Heading to the entrance, Master."}
"i want to go to the cafeteria"-> {"intent":"navigate","destination":"cafeteria","reply":"Going to the cafeteria, Master."}
"stop"                         -> {"intent":"dev_command","destination":"emergency_stop","reply":"Stopping immediately, Master."}
"stop driving"                 -> {"intent":"dev_command","destination":"emergency_stop","reply":"Stopping immediately, Master."}
"stop navigation"              -> {"intent":"dev_command","destination":"stop_navigation","reply":"Stopping navigation, Master."}
"show me the map"              -> {"intent":"show_map","destination":null,"reply":"Of course, Master."}
"open the floor plan"          -> {"intent":"show_map","destination":null,"reply":"Right away, Master."}
"close the map"                -> {"intent":"hide_map","destination":null,"reply":"Closing the map, Master."}
"create a waypoint"            -> {"intent":"create_point","destination":null,"reply":"Of course, Master. Tap the map to place your marker."}
"start mapping"                -> {"intent":"dev_command","destination":"start_mapping","reply":"Starting mapping, Master."}
"initialize localization"      -> {"intent":"dev_command","destination":"start_localization","reply":"Starting localization, Master."}
"stop all"                     -> {"intent":"dev_command","destination":"stop_all","reply":"Stopping all systems, Master."}
"developer mode"               -> {"intent":"dev_command","destination":"mode_developer","reply":"Switching to developer mode, Master."}
"user mode"                    -> {"intent":"dev_command","destination":"mode_user","reply":"Switching to user mode, Master."}
"the cafeteria"                -> {"intent":"clarify","destination":"cafeteria","reply":"Did you mean the cafeteria, Master?"}
"i think i want lab a"         -> {"intent":"clarify","destination":"lab a","reply":"Did you mean lab a, Master?"}
"no i was talking to you"      -> {"intent":"chatter","destination":null,"reply":""}
"so anyway like i was saying"  -> {"intent":"chatter","destination":null,"reply":""}
"the place with the elevators" -> {"intent":"clarify","destination":"elevator one","reply":"Did you mean elevator one, Master?"}
"what's the weather like"      -> {"intent":"question","destination":null,"reply":"Partly cloudy and 16 degrees in Delft, Master."}
"and tomorrow"                 -> {"intent":"question","destination":null,"reply":"I don't have the forecast, Master, but expect typical Dutch spring weather."}
"who built you"                -> {"intent":"question","destination":null,"reply":"A Bachelor End Project team at TU Delft, Master."}
"tell me a joke"               -> {"intent":"question","destination":null,"reply":"Why don't wheelchairs play chess, Master? Because they always roll into checkmate."}
"thanks"                       -> {"intent":"goodbye","destination":null,"reply":"Anytime, Master. Just say Jarvis when you need me."}
"that's all"                   -> {"intent":"goodbye","destination":null,"reply":"Very good, Master."}
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
    def classify(self, utterance: str, address: str = "Master") -> Intent:
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
        # Inject the live destinations list so the LLM always knows the current
        # set of valid places — including any added from map_points.json at runtime.
        dest_list = ", ".join(
            k for k in self.config.destinations if k != "stop"
        )
        messages.append({
            "role": "system",
            "content": f'Runtime destinations (use exact phrasing): {dest_list or "none defined yet"}.',
        })
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

        # ---- Barge-in (interrupt playback when the user talks) ----
        self._barge_enabled: bool = bool(getattr(config, "barge_in_enabled", False))
        # dBFS → linear RMS threshold for int16 audio (same maths as NoiseGate).
        self._barge_threshold: float = 32768.0 * (
            10.0 ** (getattr(config, "barge_in_threshold_db", -30.0) / 20.0)
        )
        self._barge_frames: int = int(getattr(config, "barge_in_frames", 3))
        self._barge_speech_run: int = 0      # consecutive speech chunks seen
        self._last_interrupted: bool = False  # set by _say_edge, read by worker
        self._barge_captured: list = []       # user speech to re-feed after a cut
        self._barge_collected: list = []      # every chunk read during playback

        # All TTS calls are routed through this queue so that every pygame /
        # COM operation runs on one dedicated thread (the one that initialized
        # the audio backend).  This avoids the silent failure that occurs when
        # pygame.mixer or pyttsx3 COM objects are called from a different
        # thread than the one that created them (notably: VoiceController.run()
        # is called in a daemon thread in the test harness).
        self._tts_queue: "queue.Queue[Optional[tuple]]" = queue.Queue()
        self._tts_done: threading.Event = threading.Event()
        self._tts_thread: Optional[threading.Thread] = None

        if not config.tts_enabled:
            return

        if config.tts_provider == "edge":
            self._init_edge()
        if self._mode == "off":
            self._init_sapi()

        if self._mode != "off":
            self._tts_thread = threading.Thread(
                target=self._tts_worker, daemon=True, name="Speaker-TTS"
            )
            self._tts_thread.start()

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

    def _tts_worker(self) -> None:
        """Dedicated thread that owns the audio backend and processes all
        say() requests.  Keeps pygame / COM calls on one thread."""
        while True:
            item = self._tts_queue.get()
            if item is None:          # shutdown sentinel
                break
            text, drain_flag, done_event = item
            self._last_interrupted = False
            self._barge_captured = []
            self._barge_collected = []
            try:
                if self._mode == "edge":
                    self._say_edge(text)
                elif self._mode == "sapi" and self._engine is not None:
                    try:
                        self._engine.say(text)
                        self._engine.runAndWait()
                    except Exception as e:
                        print(f"[Speaker] SAPI error: {e}")
                if self._last_interrupted and self._barge_captured:
                    # The user cut Jarvis off — push their speech back onto the
                    # mic queue so the recogniser still gets the command, and
                    # skip the drain that would otherwise discard it.
                    self._refeed(self._barge_captured)
                elif not drain_flag and self._barge_collected:
                    # drain=False means the caller wants to keep audio that
                    # arrived during playback (e.g. a command spoken right
                    # after the wake-word ack).  Barge monitoring already read
                    # those chunks off the queue, so put them back.
                    self._refeed(self._barge_collected)
                elif drain_flag:
                    self._drain_mic()
                self._barge_captured = []
                self._barge_collected = []
            except Exception as e:
                print(f"[Speaker] TTS worker error: {e}")
            finally:
                if done_event is not None:
                    done_event.set()

    def _refeed(self, chunks: list) -> None:
        """Put captured mic chunks back onto the queue (preserving order)."""
        if self._audio_queue is None:
            return
        for chunk in chunks:
            # Non-blocking: the mic queue is bounded now, so a blocking put
            # could stall the TTS thread if the queue happened to be full.
            try:
                self._audio_queue.put_nowait(chunk)
            except queue.Full:
                break

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
        if self._tts_thread is not None and self._tts_thread.is_alive():
            # Route through the dedicated TTS thread so that all pygame /
            # COM operations happen on the thread that initialized them.
            done = threading.Event()
            self._tts_queue.put((text, drain, done))
            done.wait()
        else:
            # Fallback (no worker thread — tts_enabled=False or init failed):
            # call directly, accepting the cross-thread risk.
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
            # Hard timeout so a stalled cloud connection can never hang the
            # caller forever.  Edge-TTS is a network service, and say() waits
            # on this synchronously, so without a timeout one bad round trip
            # froze the whole recogniser thread (the "stopped talking" + freeze
            # seen on Linux).  On timeout this raises and _say_edge falls back.
            await asyncio.wait_for(
                edge_tts.Communicate(text, self.config.edge_voice).save(tmp_path),
                timeout=6.0,
            )

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
            self._barge_speech_run = 0
            collected: list = []
            while self._pygame.mixer.music.get_busy():
                # Listen for the user talking over Jarvis and cut playback if so.
                if self._barge_detected(collected):
                    self._pygame.mixer.music.stop()
                    self._last_interrupted = True
                    # Keep only the chunks from the onset of the user's speech
                    # (plus a small lead-in) so we don't re-feed Jarvis's own
                    # bleed from earlier in the reply.
                    keep = self._barge_speech_run + 2
                    self._barge_captured = collected[-keep:]
                    print("[Speaker] Barge-in — user interrupted, stopping playback.")
                    break
                self._pygame.time.wait(50)
            self._pygame.mixer.music.unload()
            # Hand the chunks read during playback to the worker so it can
            # decide whether to drop, re-feed (drain=False), or re-feed just
            # the interrupting speech (barge-in).
            self._barge_collected = collected
        except Exception as e:
            print(f"[Speaker] Edge-TTS error: {e} - falling back to SAPI.")
            self._say_sapi_fallback(text)
        finally:
            if owns_file and tmp_path:
                try:
                    os.remove(tmp_path)
                except OSError:
                    pass

    def _barge_detected(self, collected: list) -> bool:
        """Drain mic chunks that arrived during playback and report whether the
        user has started talking over Jarvis.

        Each available chunk is read off the mic queue (so it is not lost),
        appended to *collected*, and tested against the barge-in RMS
        threshold.  Returns True once `barge_in_frames` consecutive speech
        chunks have been seen.  Returns False (and leaves the run counter
        intact) when there is nothing loud enough yet.
        """
        if not self._barge_enabled or self._audio_queue is None:
            return False
        while True:
            try:
                chunk = self._audio_queue.get_nowait()
            except queue.Empty:
                break
            collected.append(chunk)
            samples = np.frombuffer(chunk, dtype=np.int16).astype(np.float32)
            rms = float(np.sqrt(np.mean(samples ** 2))) if samples.size else 0.0
            if rms >= self._barge_threshold:
                self._barge_speech_run += 1
            else:
                self._barge_speech_run = 0
            if self._barge_speech_run >= self._barge_frames:
                return True
        return False

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
    def on_create_point(self) -> None: ...
    def on_navigate(self, payload: dict) -> None: ...
    def on_dev_command(self, command: str) -> None: ...
    def on_stop_driving(self) -> None: ...


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
        # Fast keyword navigator (bypass LLM for drive commands)
        self.fast_navigator = (
            FastNavigator(self.config.destinations)
            if self.config.fast_navigate else None
        )
        if self.fast_navigator:
            print("[FastNavigator] Active — navigate commands bypass LLM.")

        # Single-worker pool for the parallel LLM+FastNav race.  One worker
        # is enough because commands are processed sequentially; the pool is
        # only here to let the LLM HTTP call start before FastNav finishes its
        # < 1 ms keyword check, so misrecognised-verb commands reach Groq with
        # a head start instead of waiting for FastNav to give up first.
        self._llm_pool = concurrent.futures.ThreadPoolExecutor(
            max_workers=1, thread_name_prefix="llm-race"
        )

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
            DeepFilterDenoiser(
                sample_rate=self.config.sample_rate,
                post_filter=self.config.deepfilter_post_filter,
                atten_lim_db=self.config.deepfilter_atten_lim_db,
            )
            if (self.config.spectral_denoiser_enabled
                and self.noise_gate is not None)
            else None
        )

        self._address: str = self.config.default_address
        self._map_visible: bool = False
        # When the LLM asks "Did you mean X?" (clarify intent) the candidate
        # destination is parked here so the user can reply with a plain
        # yes/no on the next turn instead of repeating the destination.
        self._pending_clarify: Optional[str] = None
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

    def add_destination(self, name: str, loc_id: str) -> None:
        """Register a new named destination at runtime (thread-safe).

        Updates config.destinations and rebuilds FastNavigator so the new
        location is immediately matchable without restarting the system.
        Called by main.py when a waypoint is placed via the map overlay or
        loaded from map_points.json.
        """
        key = name.lower().strip()
        self.config.destinations[key] = loc_id
        if self.fast_navigator is not None:
            self.fast_navigator = FastNavigator(self.config.destinations)
        print(f"[VoiceController] Destination registered: '{key}' → {loc_id}")

    def _resolve_map_destination(self, dest_phrase: str) -> Optional[str]:
        """Return the location id for a destination only if it is a map marker.

        Matches case-insensitively against the live destinations table (which
        holds nothing but map markers plus "stop"). Returns None for anything
        not on the map so callers can refuse the command instead of inventing
        a location the pathfinding team has no coordinates for.
        """
        key = (dest_phrase or "").lower().strip()
        if not key or key == "stop":
            return None
        for name, loc_id in self.config.destinations.items():
            if name == "stop":
                continue
            if name.lower() == key:
                return loc_id
        return None

    def _reject_unknown_destination(self, dest_phrase: str) -> None:
        """Tell the user the requested place is not a marker on the map."""
        valid = [k for k in self.config.destinations if k != "stop"]
        if valid:
            options = ", ".join(sorted(valid))
            reply = (
                f"{dest_phrase.title()} is not on the map, {self._address}. "
                f"I can take you to {options}."
            )
        else:
            reply = (
                f"There are no destinations on the map yet, {self._address}."
            )
        self.observer.on_reply(reply)
        self._broadcast_state("SPEAKING")
        self.speaker.say(reply)
        self._broadcast_state("LISTENING")

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
                    # Backlog probe: prints only when audio is piling up behind
                    # the loop (queue should normally sit at 0-1).  A spike here
                    # while Jarvis is replying points straight at the stall.
                    _qd = self.audio.queue.qsize()
                    if _qd >= 3:
                        print(f"[qsize] backlog building: {_qd} chunks waiting")
                except queue.Empty:
                    # No audio for 0.2 s — pure silence.  Let the committer
                    # decide whether the pending buffer is ready to dispatch.
                    if _committer is not None:
                        committed = _committer.tick(is_speech=False)
                        if committed:
                            self._handle_command(committed,
                                                 _commit_full or committed)
                            _commit_full = ""
                            self._drop_stale_audio("after command")
                    continue

                # ---- Idle backlog cap -----------------------------------
                # While Jarvis is idle (not awake), the only thing that
                # should fill the queue is the open mic hearing the room.
                # On a slow CPU the large Vosk model cannot decode constant
                # background speech in real time, so the queue creeps up and
                # everything lags.  We do not need that backlog — drop it so
                # idle listening always stays close to real time.  Once awake
                # we keep every chunk so the actual command is never clipped.
                if (not self.wake_gate.is_awake
                        and self.audio.queue.qsize() > 15):
                    self._drop_stale_audio("idle backlog")

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
                            # Only run the heavy DeepFilterNet pass once Jarvis
                            # is awake and actually listening for a command.
                            # On a CPU it cannot denoise every 100 ms chunk in
                            # under 100 ms, so running it on the always-on idle
                            # stream made the loop fall permanently behind and
                            # the queue sat several seconds deep (the backlog in
                            # the screenshot).  Wake-word detection works fine on
                            # the noise-gated audio, so the idle loop stays real
                            # time and only the short command window pays the
                            # denoiser cost.
                            if self.wake_gate.is_awake:
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
                        self._drop_stale_audio("after command")
                        continue

                result = self.engine.feed(pcm)

                if result is None:
                    if self.config.show_partials:
                        partial = self.engine.partial().get("partial", "")
                        # A lone "the"/"a" partial is Vosk chewing on room
                        # noise, not the user.  Suppress it so a stray "the"
                        # does not sit flashing on screen between commands.
                        p_words = partial.split()
                        if (len(p_words) == 1
                                and p_words[0].lower() in _STT_NOISE_WORDS):
                            partial = ""
                        if partial and partial != last_partial:
                            print(f"\r[live ] {partial:<70}",
                                  end="", flush=True)
                            self.observer.on_partial(partial)
                            last_partial = partial
                    continue

                final_text = (result.get("text") or "").strip()
                # Drop a dangling "the"/"a" off the ends before anything
                # else looks at the line.  This stops a stray trailing word
                # from sitting on screen and from holding the live partial
                # open longer than it needs to.
                final_text = _strip_edge_noise_words(final_text)

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

                # Correct verb mishearings (wake/break -> take, road -> go,
                # elevate -> elevator, …) at the earliest point so the wake
                # gate, committer and fast path all work on the intended words.
                # This is what keeps a misheard command from being any slower
                # than a cleanly-heard one.
                cmd_text = _normalise_homophones(final_text) if final_text else final_text
                command = self.wake_gate.filter(cmd_text)
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
                    self._pending_clarify = None
                    self._broadcast_state("AWAITING COMMAND")
                    reply = f"Yes, {self._address}?"
                    self.observer.on_reply(reply)
                    self._broadcast_state("SPEAKING")
                    self.speaker.say(reply, drain=False)
                    # Throw away the mic backlog that piled up while "Yes,
                    # Master?" was playing.  Without this the loop decodes a
                    # second or two of stale room audio first, which both
                    # delays the real command and smears it together with
                    # whatever leaked in (the garbled finals after a wake).
                    self._drop_stale_audio("after wake ack")
                    self._broadcast_state("AWAITING COMMAND")
                    continue

                # Emergency stop and already-complete navigation commands are
                # dispatched immediately, bypassing the silence window.  A
                # full "verb + destination" match (e.g. "take me to lab a")
                # is a finished sentence, so there is nothing to wait for —
                # this is what kept misheard verbs feeling slow, since they
                # only reached the fast path after the commit delay.
                fast_hit = (
                    self.fast_navigator.match(command, address=self._address)
                    if self.fast_navigator is not None else None
                )
                # A clear map command is also a finished sentence, so it skips
                # the silence window too and reacts immediately.
                map_hit = bool(_MAP_SHOW_RE.search(command)
                               or _MAP_HIDE_RE.search(command))
                # Operator commands ("stop navigation", "start mapping", ...)
                # are finished sentences too, so they skip the silence window
                # and react immediately like the stop and map commands.
                dev_hit = _match_dev_command(command) is not None
                if (_STOP_ONLY_RE.match(command)
                        or _SLEEP_RE.match(command)
                        or fast_hit is not None
                        or map_hit
                        or dev_hit):
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
        self.observer.on_reply(question)   # show the prompt on the GUI
        self._broadcast_state("SPEAKING")
        # drain=False: keep audio captured during TTS so the user can say
        # "yes" while Jarvis is still asking and the confirmation loop
        # picks it up.  Barge-in stops playback early when the user talks
        # over Jarvis; drain=False ensures non-barge audio is also kept.
        self.speaker.say(question, drain=False)
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

            result = self.engine.feed(pcm)
            if result is None:
                # Show partial so the user sees they're being heard, and try to
                # match it live — Vosk often never emits a final result inside
                # the window (no endpoint when the user goes quiet after a short
                # "yes"), so waiting for the final is what caused the timeouts.
                partial = self.engine.partial().get("partial", "")
                if partial:
                    print(f"\r[confirm live] {partial:<60}", end="", flush=True)
                    # Only "yes" is acted on live — a quick yes is all we need,
                    # so we commit the moment it appears.  "no" is deliberately
                    # NOT matched on the partial: the user may be saying "no,
                    # take me to lab a instead", and acting early would throw
                    # the rest away.  We wait for the final to capture it all.
                    if _classify_confirmation(_normalise_homophones(partial)) == "yes":
                        print(f"\r[Confirm] Heard (live): '{partial}' → YES{' ' * 20}")
                        self.engine.reset()
                        reply = f"On my way, {self._address}."
                        self.observer.on_reply(reply)
                        self.speaker.say(reply, drain=False)
                        return True
                continue

            text = (result.get("text") or "").strip().lower()
            if not text:
                continue

            print(f"\r[Confirm] Heard: '{text}'{' ' * 50}")

            norm_text = _normalise_homophones(text)
            verdict = _classify_confirmation(norm_text)
            if verdict == "yes":
                # A yes settles it — ignore anything said after it.
                reply = f"On my way, {self._address}."
                self.observer.on_reply(reply)
                self.speaker.say(reply, drain=False)
                return True
            if verdict == "no":
                # A no cancels.  If the user tacked a fresh instruction on the
                # end ("no, take me to lab a"), run that instead of just
                # parking the chair.
                remainder = _command_after_no(norm_text)
                self.engine.reset()
                if remainder:
                    print(f"[Confirm] 'no' + follow-up command: '{remainder}'")
                    ack = f"Of course, {self._address}."
                    self.observer.on_reply(ack)
                    self.speaker.say(ack, drain=False)
                    self._handle_command(remainder, remainder)
                else:
                    reply = f"Understood, {self._address}. Let me know when you're ready."
                    self.observer.on_reply(reply)
                    self.speaker.say(reply, drain=False)
                return False

            # Heard something but it wasn't a clear yes/no — prompt once more
            reply = f"Sorry, {self._address} — yes or no?"
            self.observer.on_reply(reply)
            self.speaker.say(reply, drain=False)

        # Timed out — stay put for safety
        print("[Confirm] Timed out — cancelling navigation.")
        reply = f"No confirmation received, {self._address}. Staying put."
        self.observer.on_reply(reply)
        self.speaker.say(reply, drain=False)
        return False

    # -- command pipeline -------------------------------------------
    def _drop_stale_audio(self, reason: str) -> int:
        """Throw away everything sitting in the mic queue and reset Vosk.

        _handle_command blocks the audio thread for the whole Groq round
        trip and the spoken reply, and on an open mic the queue fills with
        room audio the entire time.  If we then processed that backlog the
        loop would lag several seconds behind and decode a wall of crowd
        speech.  We only care about fresh audio after a command, so drop it
        and clear the recogniser's half-built partial.
        """
        dropped = 0
        while True:
            try:
                self.audio.queue.get_nowait()
                dropped += 1
            except queue.Empty:
                break
        if dropped:
            self.engine.reset()
            print(f"[drain] dropped {dropped} stale chunks ({reason})")
        return dropped

    def _trigger_stop_driving(self) -> None:
        """End the current drive in response to a spoken stop command.

        Hands off to ``on_stop_driving()``, which main.py turns into a goal at
        the chair's current AMCL pose.  Nav2 then plans to where the chair
        already is and finishes at once, so it stops driving without killing
        the nav stack, the voice subsystem, or the GUI.  The earlier nav-goal
        route never stopped the chair because main.py rejects any goal that is
        not a map marker, which is why a bare emit did nothing.
        """
        reply = f"Stopping here, {self._address}."
        self.observer.on_reply(reply)
        self._broadcast_state("SPEAKING")
        self.speaker.say(reply)
        self.observer.on_stop_driving()
        self._pending_clarify = None
        self.wake_gate.end_conversation()
        self.intent_classifier.reset_history()
        self._broadcast_state("LISTENING")

    def _handle_command(self, command: str, full_transcript: str) -> None:
        # ---- Homophone correction ----
        # Fix the common Vosk verb swaps (wake/break -> take, road -> go,
        # police -> please) before any matcher sees the command, so a single
        # misheard word does not lose an otherwise clear instruction.
        corrected = _normalise_homophones(command)
        if corrected != command:
            print(f"[Homophone] '{command}' -> '{corrected}'")
            command = corrected

        # ---- Emergency motion stop (highest priority, no LLM) ----
        # Checked before everything else, including a pending "Did you mean
        # X?" question, so a spoken stop always halts the chair no matter what
        # state Jarvis is in.  This covers "stop", "stop driving", "stop the
        # chair", "halt", and the like (see _STOP_ONLY_RE).
        if _STOP_ONLY_RE.match(command):
            print("[FastStop] stop driving (LLM skipped)")
            self._trigger_stop_driving()
            return

        # ---- Explicit sleep command ("go to sleep" / "stop listening") ----
        # Spoken twin of the wake word: the user dismisses Jarvis on demand
        # and he returns to idle until "Jarvis" is heard again.  Checked
        # before everything else so it also cancels a pending "Did you mean
        # X?" question.  It does NOT move the chair — only stops listening.
        if _SLEEP_RE.match(command):
            print("[WakeGate] sleep command → returning to idle.")
            self._pending_clarify = None
            reply = f"Going to sleep, {self._address}. Say Jarvis when you need me."
            self.observer.on_reply(reply)
            self._broadcast_state("SPEAKING")
            self.speaker.say(reply)
            self.wake_gate.end_conversation()
            self.intent_classifier.reset_history()
            self._broadcast_state("STANDING BY")
            return

        # ---- Pending clarification ("Did you mean X?") ----
        # If the previous turn asked the user to confirm a destination,
        # treat a plain yes/no here as the answer so they don't have to
        # say the destination again.  Anything that is not a yes/no clears
        # the pending question and is processed as a fresh command below.
        if self._pending_clarify is not None:
            verdict = _classify_confirmation(command)
            dest_phrase = self._pending_clarify
            self._pending_clarify = None
            if verdict == "yes":
                self._dispatch_navigate(dest_phrase, full_transcript)
                return
            if verdict == "no":
                remainder = _command_after_no(command)
                if remainder:
                    print(f"[Clarify] 'no' + follow-up command: '{remainder}'")
                    ack = f"Of course, {self._address}."
                    self.observer.on_reply(ack)
                    self._broadcast_state("SPEAKING")
                    self.speaker.say(ack, drain=False)
                    self._handle_command(remainder, remainder)
                else:
                    reply = f"No problem, {self._address}. Where would you like to go?"
                    self.observer.on_reply(reply)
                    self._broadcast_state("SPEAKING")
                    self.speaker.say(reply)
                    self._broadcast_state("LISTENING")
                return

        # ---- Fast path: map show / hide (no LLM) ----
        # The map command is the one the visitors use most, so it is resolved
        # locally.  Routing it through the LLM meant a single misheard word
        # ("show me them up") came back as chatter and dropped the
        # conversation; the local match never does that.
        if _MAP_SHOW_RE.search(command):
            print("[FastMap] show_map  (LLM skipped)")
            reply = f"Of course, {self._address}."
            self.observer.on_reply(reply)
            self._broadcast_state("SPEAKING")
            self.speaker.say(reply)
            self.observer.on_show_map()
            self._broadcast_state("LISTENING")
            return
        if _MAP_HIDE_RE.search(command):
            print("[FastMap] hide_map  (LLM skipped)")
            reply = f"Closing the map, {self._address}."
            self.observer.on_reply(reply)
            self._broadcast_state("SPEAKING")
            self.speaker.say(reply)
            self.observer.on_hide_map()
            self._broadcast_state("LISTENING")
            return

        # ---- Fast path: operator / developer commands (no LLM) ----
        # Start / stop mapping, localization, navigation, and the all-in-one
        # commands resolve locally so they no longer depend on a Groq round
        # trip or the model classifying a clipped transcript correctly.
        dev_cmd = _match_dev_command(command)
        if dev_cmd is not None:
            print(f"[FastDev] {dev_cmd}  (LLM skipped)")
            reply = _DEV_CMD_REPLIES.get(
                dev_cmd, f"Done, {self._address}."
            ).format(address=self._address)
            self.observer.on_reply(reply)
            self._broadcast_state("SPEAKING")
            self.speaker.say(reply)
            self.observer.on_dev_command(dev_cmd)
            self._broadcast_state("LISTENING")
            return

        # ---- Parallel LLM + FastNav race ----
        # Submit the LLM call to the background thread immediately so the
        # Groq HTTP request is in flight while FastNav does its < 1 ms check.
        # On a clean FastNav match the future is cancelled / ignored.
        # On a miss (garbled verb, free-form phrasing) the LLM result is
        # already arriving instead of starting only after FastNav gives up.
        llm_future: Optional[concurrent.futures.Future] = None
        if (self.fast_navigator is not None
                and getattr(self.intent_classifier, "_available", False)):
            llm_future = self._llm_pool.submit(
                self.intent_classifier.classify, command, self._address
            )

        # ---- Fast path: keyword router (no LLM, ~0 ms) ----
        if self.fast_navigator is not None:
            fast_match = self.fast_navigator.match(command, address=self._address)
            if fast_match:
                dest_key, loc_id, reply = fast_match
                # FastNav won — discard the LLM future.  cancel() is a no-op
                # if the HTTP request is already in flight, which is fine: the
                # result comes back to the pool thread and is simply never read.
                if llm_future is not None:
                    llm_future.cancel()
                print(f"[FastNav] '{dest_key}' → {loc_id}  (LLM skipped)")
                hit = Hit(phrase=dest_key, location_id=loc_id,
                          confidence=1.0, raw_text=full_transcript)
                self.observer.on_reply(reply)

                if loc_id == "EMERGENCY_STOP":
                    # A stop is a halt, not a destination.  End the drive by
                    # resending the current pose as the goal (the plain nav-goal
                    # path rejects it as "not a map marker").
                    self._trigger_stop_driving()
                    return

                # Voice confirmation before moving
                dest_display = dest_key.replace("_", " ").title()
                confirmed = self._voice_confirm(dest_display)
                if confirmed:
                    self.emitter.emit(hit)
                    self.wake_gate.end_conversation()
                    self.intent_classifier.reset_history()
                self._broadcast_state("LISTENING")
                return

        # ---- LLM path: intent classification ----
        self._broadcast_state("THINKING")
        try:
            if llm_future is not None:
                # Collect the parallel result — already in flight since before
                # the FastNav check, so wait time ≈ max(0, llm_time - fastnav_time).
                intent = llm_future.result(timeout=30)
            else:
                intent = self.intent_classifier.classify(command, address=self._address)
        except Exception as exc:
            print(f"[Intent ] classify error: {exc}")
            self._broadcast_state("LISTENING")
            return

        print(f"\n[Intent ] {intent.intent_type!r}  dest={intent.destination!r}")

        if intent.intent_type == "navigate":
            self._handle_navigate(intent, command, full_transcript)
            return

        reply = intent.reply or ""
        if intent.intent_type == "show_map":
            self.observer.on_reply(reply)
            self._broadcast_state("SPEAKING")
            self.speaker.say(reply)
            self.observer.on_show_map()
        elif intent.intent_type == "hide_map":
            self.observer.on_reply(reply)
            self._broadcast_state("SPEAKING")
            self.speaker.say(reply)
            self.observer.on_hide_map()
        elif intent.intent_type == "create_point":
            self.observer.on_reply(reply)
            self._broadcast_state("SPEAKING")
            self.speaker.say(reply)
            self.observer.on_create_point()
        elif intent.intent_type == "dev_command":
            # A stop that only the LLM caught still ends the drive the same way
            # the fast path does, by resending the current pose as the goal.
            if intent.destination in ("emergency_stop", "stop_driving"):
                self._trigger_stop_driving()
                return
            self.observer.on_reply(reply)
            self._broadcast_state("SPEAKING")
            self.speaker.say(reply)
            if intent.destination:
                self.observer.on_dev_command(intent.destination)
        elif intent.intent_type == "clarify":
            # Ask the user to confirm the guessed destination and park it so
            # a plain yes/no on the next turn resolves it (see _handle_command).
            self._pending_clarify = (intent.destination or "").lower().strip() or None
            self.observer.on_reply(reply)
            self._broadcast_state("SPEAKING")
            self.speaker.say(reply)
        elif intent.intent_type == "goodbye":
            self.observer.on_reply(reply)
            self._broadcast_state("SPEAKING")
            self.speaker.say(reply)
            self.wake_gate.end_conversation()
            self.intent_classifier.reset_history()
        elif intent.intent_type == "question":
            self.observer.on_reply(reply)
            self._broadcast_state("SPEAKING")
            self.speaker.say(reply)
        else:  # chatter — false alarm or speech aimed at someone else.
            # The follow-up after the wake word was not a question or a
            # command for Jarvis, so treat the wake as a false trigger and
            # drop straight back to idle instead of holding the conversation
            # window open for the full timeout. Jarvis only listens again
            # after the wake word is spoken once more.
            print("[WakeGate] chatter after wake → returning to idle.")
            self.wake_gate.end_conversation()
            self.intent_classifier.reset_history()
            self._broadcast_state("STANDING BY")
            return

        self._broadcast_state("LISTENING")

    def _dispatch_navigate(self, dest_phrase: str, full_transcript: str) -> None:
        """Emit a navigation goal for an already confirmed destination.

        Used when the user has confirmed a "Did you mean X?" clarification
        with a yes — the question already served as the confirmation, so the
        goal is sent straight to the pathfinding team without asking again.
        """
        dest_phrase = (dest_phrase or "").lower().strip()
        # Only map markers are valid destinations — reject anything else.
        loc_id = self._resolve_map_destination(dest_phrase)
        if loc_id is None:
            self._reject_unknown_destination(dest_phrase)
            return
        hit = Hit(
            phrase=dest_phrase,
            location_id=loc_id,
            confidence=0.85,
            raw_text=full_transcript,
        )
        reply = f"On my way to {dest_phrase.title()}, {self._address}."
        self.observer.on_reply(reply)
        self._broadcast_state("SPEAKING")
        self.speaker.say(reply, drain=False)
        self.emitter.emit(hit)
        self.wake_gate.end_conversation()
        self.intent_classifier.reset_history()
        self._broadcast_state("LISTENING")

    def _handle_navigate(
        self,
        intent: "Intent",
        command: str,
        full_transcript: str,
    ) -> None:
        """LLM-classified navigate fallback (used when FastNavigator didn't match)."""
        dest_phrase = (intent.destination or "").lower().strip()
        if not dest_phrase:
            reply = (
                f"I didn't catch a destination, {self._address}. "
                f"Where would you like to go?"
            )
            self.observer.on_reply(reply)
            self._broadcast_state("SPEAKING")
            self.speaker.say(reply)
            self._broadcast_state("LISTENING")
            return
        # Only allow destinations that exist as map markers. The LLM can
        # hallucinate a place that is not on the map, so reject anything not
        # in the live destinations table instead of synthesising a LOC_ id.
        loc_id = self._resolve_map_destination(dest_phrase)
        if loc_id is None:
            self._reject_unknown_destination(dest_phrase)
            return
        hit = Hit(
            phrase=dest_phrase,
            location_id=loc_id,
            confidence=0.85,
            raw_text=full_transcript,
        )
        reply = intent.reply or f"On my way to {dest_phrase.title()}, {self._address}."
        self.observer.on_reply(reply)
        confirmed = self._voice_confirm(dest_phrase.title())
        if confirmed:
            self.emitter.emit(hit)
            self.wake_gate.end_conversation()
            self.intent_classifier.reset_history()
        self._broadcast_state("LISTENING")


if __name__ == "__main__":
    cfg  = VoiceConfig()
    ctrl = VoiceController(config=cfg)
    dests = ", ".join(cfg.destinations.keys())
    print(
        f"\n[VoiceController] Say '{cfg.wake_word}' to wake the chair. "
        f"Valid destinations: {dests}"
    )
    print("Press Ctrl+C to quit.\n")
    try:
        ctrl.run()
    except KeyboardInterrupt:
        print(f"\n[VoiceController] Shutting down (Ctrl+C).")
        ctrl.stop()
