import wave

from bringup.sounds import PHRASES, SOUNDS, write_all
from navigation.state_machine import ALERTS, TRANSITION_ALERTS


def test_every_alert_has_a_phrase_and_a_beep(tmp_path):
    alerts = set(ALERTS.values()) | set(TRANSITION_ALERTS.values()) | {"start", "obstacle"}
    assert alerts == set(PHRASES) == set(SOUNDS)
    files = write_all(tmp_path)
    for name, path in files.items():
        with wave.open(str(path)) as w:
            seconds = w.getnframes() / w.getframerate()
        assert abs(seconds - sum(d for _, d in SOUNDS[name])) < 0.01
