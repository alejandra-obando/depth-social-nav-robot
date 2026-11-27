# Experiments

Standalone tests and plots of each neural model. Each experiment prints PASS/FAIL per check and saves a plot
in `results/`.

```bash
.venv/bin/python -m neural.experiments.run_all
```

| Experiment | What it checks | Status |
|------------|----------------|--------|
| `exp_leaky_integrator.py` | Occlusions ≤ 2 s keep the link, ≥ 4 s break it; recovery < 0.5 s | 5/5 |
| `exp_competitive_network.py` | One winner; a 10 % larger challenger does not switch, a 50 % larger one does; no flicker with noise | 5/5 |
| `exp_neural_field.py` | Peak at the sign; memory > 3 s; forgotten < 8 s; one peak with two signs; a stronger distractor does not steal the peak; tracking lag < 8° | 6/6 |
| `exp_attractor_dynamics.py` | Reaches the goal; avoids one obstacle; passes a 1.0 m gap; goes around a 0.3 m gap (bifurcation) | 9/9 |
| `exp_motion_detector.py` | Responds to a crossing person and codes its direction; silent for a static scene; ego-motion compensation | 3/4 (ego-motion fails, known) |
| `exp_corner_detector.py` | Fires at the far corner after travelling; not in the middle, not before travelling, not at one wall, not between two briefcases | 5/5 |
| `exp_behaviour_selector.py` | Sequence FOLLOW → SEARCH → FOLLOW → SIGNAL → WAIT → GUIDE → WAIT → GUIDE → DONE; ignores a 0.1 s glitch; phase stays 2 | 3/3 |
