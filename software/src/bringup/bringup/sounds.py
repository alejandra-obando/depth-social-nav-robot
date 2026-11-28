"""What the speaker says, and the beeps used when a phrase has no recording.

One alert per event of the exam plan.  The phrases are spoken from WAV files in
``voice_dir`` (speaker_node parameter, default ~/.config/social_nav/voice): make them
with espeak-ng (``ros2 run bringup make_voice``) or record your own with the same
names (``aplay``-compatible WAV).  A missing file falls back to the beep below.
"""

import struct
import wave
from pathlib import Path

import numpy as np

RATE = 22050

# alert -> phrase (Spanish, as in the exam plan).
PHRASES = {
    "start": "Sígueme",                                          # phase 1 starts (back tag first seen)
    "obstacle": "Ten cuidado con los obstáculos",                # VFH slows the robot down
    "lost": "No te veo, espérame",                               # LOST (repeated)
    "found": "Te encontré",                                      # LOST -> FOLLOW, WAIT_PERSON -> GUIDE
    "corner": "Fase uno completa. Ahora te guío de regreso",     # far corner: end-of-phase signal
    "guide": "Sígueme",                                          # phase 2 starts (chest tag seen)
    "wait": "Te espero",                                         # WAIT_PERSON (repeated)
    "done": "Llegamos. Fin del recorrido",                       # start corner reached
}

# Fallback beeps: each a list of (frequency [Hz], duration [s]); frequency 0 is silence.
SOUNDS = {
    "start": [(523, 0.15), (784, 0.15)],
    "obstacle": [(330, 0.3)],
    "lost": [(440, 0.15), (0, 0.1), (440, 0.15)],
    "found": [(784, 0.15), (1047, 0.15)],
    "corner": [(523, 0.25), (659, 0.25), (784, 0.5)],
    "guide": [(523, 0.15), (784, 0.15)],
    "wait": [(660, 0.2)],
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


def make_voice(folder, espeak="espeak-ng", voice="es-419", speed=140):
    """Write one spoken WAV per phrase with espeak-ng; returns {name: path}."""
    import subprocess
    folder = Path(folder).expanduser()
    folder.mkdir(parents=True, exist_ok=True)
    out = {}
    for name, text in PHRASES.items():
        out[name] = folder / f"{name}.wav"
        subprocess.run([espeak, "-v", voice, "-s", str(speed), "-w", str(out[name]), text], check=True)
    return out


def main():
    import argparse
    ap = argparse.ArgumentParser(description="Make the spoken phrases with espeak-ng.")
    ap.add_argument("folder", nargs="?", default="~/.config/social_nav/voice")
    ap.add_argument("--voice", default="es-419", help="espeak-ng voice (es-419: Latin American Spanish)")
    a = ap.parse_args()
    for name, path in make_voice(a.folder, voice=a.voice).items():
        print(f"{name:9s} {path}  \"{PHRASES[name]}\"")


if __name__ == "__main__":
    main()
