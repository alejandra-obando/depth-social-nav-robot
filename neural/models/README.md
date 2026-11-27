# Models

Neural structures used by the robot. Each file is a standalone NumPy model with its equations in the docstring.

| File | Model | Used for | Based on |
|------|-------|----------|----------|
| `leaky_integrator.py` | Leaky integrator neuron with fast rise, slow decay and hysteresis | Link with the person (front and rear): survives short occlusions | Usher & McClelland 2001 |
| `competitive_network.py` | Winner-take-all of leaky competing accumulators | Behaviour selection | Usher & McClelland 2001; Redgrave, Prescott & Gurney 1999 |
| `neural_field.py` | 1-D dynamic neural field (Amari) on a ring of directions | Direction of the person: selection among several signs, memory while hidden, forgetting | Amari 1977; Bicho, Mallet & Schöner 2000 |
| `attractor_dynamics.py` | Heading attractor dynamics (target attractor, obstacle repellors) and a speed neuron | Steering towards the person / start corner around obstacles | Schöner, Dose & Engels 1995; Bicho et al. 2000 |
| `motion_detector.py` | Array of Hassenstein–Reichardt detectors on the LiDAR scan | Moving people and briefcases: stronger repulsion, yield | Hassenstein & Reichardt 1956; Borst & Euler 2011 |
| `corner_detector.py` | Template "wall" neurons + AND + leaky integrator | Recognising the far corner (end of phase 1) and the start corner (end of phase 2) | Engineered |
| `behaviour_selector.py` | Competing behaviour units + bistable phase neuron | FOLLOW / SEARCH / SIGNAL / GUIDE / WAIT / DONE | Redgrave et al. 1999; Prescott et al. 2006 |
| `social_nav_net.py` | The complete architecture | Connects all the blocks: sensors in, `(v, omega, signal)` out | — |
