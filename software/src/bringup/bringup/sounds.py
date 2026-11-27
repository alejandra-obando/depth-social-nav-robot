"""Alert sounds for the USB speaker, generated as WAV files (no extra libraries).

The speaker tells the people in the room what the robot is doing:

* ``lost``   : the robot no longer sees the person (phase 1)       -> two short low beeps, repeated
* ``wait``   : the person fell behind or is not seen (phase 2)     -> one short beep, repeated
* ``corner`` : the far corner is recognised (end of phase 1)       -> rising three-note signal
* ``done``   : the start corner is reached (end of phase 2)        -> long high tone
"""

import struct
import wave
from pathlib import Path

import numpy as np

RATE = 22050

# Each sound is a list of (frequency [Hz], duration [s]); frequency 0 is silence.
SOUNDS = {
    "lost": [(440, 0.15), (0, 0.1), (440, 0.15)],
    "wait": [(660, 0.2)],
    "corner": [(523, 0.25), (659, 0.25), (784, 0.5)],
    "done": [(880, 1.0)],
}


def samples(notes, volume=0.6):
    out = []
    for f, d in notes:
        t = np.arange(int(RATE * d)) / RATE
        tone = np.sin(2 * np.pi * f * t) if f > 0 else np.zeros_like(t)
        # 10 ms fade in and out, so the beeps do not click.
        n = min(len(t) // 2, int(0.01 * RATE))
        if n:
            ramp = np.linspace(0, 1, n)
            tone[:n] *= ramp
            tone[-n:] *= ramp[::-1]
        out.append(tone)
    return (np.concatenate(out) * volume * 32767).astype(np.int16)


def write_wav(path, notes):
    s = samples(notes)
    with wave.open(str(path), "wb") as w:
        w.setnchannels(1)
        w.setsampwidth(2)
        w.setframerate(RATE)
        w.writeframes(struct.pack(f"<{len(s)}h", *s))
    return Path(path)


def write_all(folder):
    """Write one WAV per sound into ``folder``; returns {name: path}."""
    folder = Path(folder)
    folder.mkdir(parents=True, exist_ok=True)
    return {name: write_wav(folder / f"{name}.wav", notes) for name, notes in SOUNDS.items()}
