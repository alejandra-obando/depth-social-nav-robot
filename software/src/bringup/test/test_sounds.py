import wave

from bringup.sounds import SOUNDS, write_all


def test_every_alert_has_a_wav(tmp_path):
    files = write_all(tmp_path)
    assert set(files) == set(SOUNDS) == {"lost", "wait", "corner", "done"}
    for name, path in files.items():
        with wave.open(str(path)) as w:
            seconds = w.getnframes() / w.getframerate()
        assert abs(seconds - sum(d for _, d in SOUNDS[name])) < 0.01
