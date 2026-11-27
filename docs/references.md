# References

Bibliographic data (authors, journal, volume, pages, DOI) checked on 27 November 2026. "Open copy" links
point to the authors' or their university's repository. Read the papers themselves before quoting them in the
presentation: the notes below summarise their abstracts.

## Direct basis of the neural blocks

1. **Amari, S. (1977).** Dynamics of pattern formation in lateral-inhibition type neural fields.
   *Biological Cybernetics* 27(2):77–87. [DOI](https://doi.org/10.1007/BF00337259) ·
   [Open copy](https://bsi-ni.brain.riken.jp/database/file/64/050_2.pdf)
   – The field equation used in `neural_field.py`.
2. **Schöner, G., Dose, M. & Engels, C. (1995).** Dynamics of behavior: theory and applications for
   autonomous robot architectures. *Robotics and Autonomous Systems* 16:213–245.
   [DOI](https://doi.org/10.1016/0921-8890(95)00049-6) ·
   [Lecture notes on the approach](https://www.ini.rub.de/upload/file/1556191117_49af19cb12cefbcace71/3_attractor_dynamics_approach.pdf)
   – Heading as a dynamical variable: targets are attractors, obstacles repellors (`attractor_dynamics.py`).
3. **Bicho, E., Mallet, P. & Schöner, G. (2000).** Target representation on an autonomous vehicle with
   low-level sensors. *International Journal of Robotics Research* 19(5):424–447.
   [DOI](https://doi.org/10.1177/02783640022066950) ·
   [Open copy](https://ini.rub.de/upload/file/1682930519_50e311d868badc7cb8c1/BichoMalletSchoner98.pdf) ·
   [UMinho repository](https://repositorium.uminho.pt/handle/1822/2289)
   – **Closest to this project**: a neural field that detects targets, selects one, keeps it in memory when
   it is no longer sensed and forgets it after a delay, integrated with obstacle avoidance.
4. **Erlhagen, W. & Bicho, E. (2006).** The dynamic neural field approach to cognitive robotics.
   *Journal of Neural Engineering* 3:R36–R54. [DOI](https://doi.org/10.1088/1741-2560/3/3/R02) ·
   [Open copy](https://core.ac.uk/download/55611061.pdf)
   – Tutorial on neural fields in robots and the maths needed to implement them.
5. **Monteiro, S. & Bicho, E. (2010).** Attractor dynamics approach to formation control: theory and
   application. *Autonomous Robots* 29:331–355. [DOI](https://doi.org/10.1007/s10514-010-9198-8) ·
   [Open copy](https://core.ac.uk/download/55611628.pdf)
   – Robots keep a distance to a leader while avoiding static and moving obstacles (like following the person).
6. **Usher, M. & McClelland, J. L. (2001).** The time course of perceptual choice: the leaky, competing
   accumulator model. *Psychological Review* 108(3):550–592. [PubMed](https://pubmed.ncbi.nlm.nih.gov/11488378/)
   – Leaky integration and competition (`leaky_integrator.py`, `competitive_network.py`).
7. **Redgrave, P., Prescott, T. J. & Gurney, K. (1999).** The basal ganglia: a vertebrate solution to the
   selection problem? *Neuroscience* 89(4):1009–1023. [Open copy](https://eprints.whiterose.ac.uk/107033/)
8. **Gurney, K., Prescott, T. J. & Redgrave, P. (2001).** A computational model of action selection in the
   basal ganglia. I. A new functional anatomy. *Biological Cybernetics* 84(6):401–410.
   [DOI](https://doi.org/10.1007/PL00007984) · [PubMed](https://pubmed.ncbi.nlm.nih.gov/11417052/)
9. **Prescott, T. J., Montes González, F. M., Gurney, K., Humphries, M. D. & Redgrave, P. (2006).** A robot
   model of the basal ganglia: behavior and intrinsic processing. *Neural Networks* 19(1):31–61.
   [Open copy](https://eprints.whiterose.ac.uk/107037/1/Prescott%20Neural%20Networks%202006%20Preprint1.pdf) ·
   [dblp](https://dblp.org/rec/journals/nn/PrescottGGHR06)
   – 7–9: behaviour selection by competition (`behaviour_selector.py`).
10. **Hassenstein, B. & Reichardt, W. (1956).** *Zeitschrift für Naturforschung* 11b:513–524 (German, no
    open link). Readable account: **Borst, A. & Euler, T. (2011).** Seeing things in motion: models,
    circuits, and mechanisms. *Neuron* 71:974–994. [DOI](https://doi.org/10.1016/j.neuron.2011.08.031)
    – The elementary motion detector (`motion_detector.py`).

## Context: social navigation and person following

11. **Kruse, T., Pandey, A. K., Alami, R. & Kirsch, A. (2013).** Human-aware robot navigation: a survey.
    *Robotics and Autonomous Systems* 61(12):1726–1743. [DOI](https://doi.org/10.1016/j.robot.2013.05.007) ·
    [Open copy](https://publikationen.uni-tuebingen.de/xmlui/handle/10900/41386)
12. **Islam, M. J., Hong, J. & Sattar, J. (2019).** Person-following by autonomous robots: a categorical
    overview. *International Journal of Robotics Research* 38(14):1581–1618. [arXiv](https://arxiv.org/abs/1803.08202)
13. **Helbing, D. & Molnár, P. (1995).** Social force model for pedestrian dynamics. *Physical Review E*
    51:4282. [DOI](https://doi.org/10.1103/PhysRevE.51.4282) · [arXiv](https://arxiv.org/abs/cond-mat/9805244)
    – Used to move the simulated walkers (`simulation/people.py`).
14. RPF-Search: field-based search for robot person following in unknown dynamic environments (2025).
    [arXiv](https://arxiv.org/abs/2503.02188) – A recent, non-neural approach to finding the person again
    after an occlusion; useful for comparison. Check the authors on the arXiv page.
15. **Fu, Q., Hu, C., Peng, J. & Yue, S. (2018).** Collision selective visual neural network inspired by
    LGMD2 neurons in juvenile locusts. [arXiv](https://arxiv.org/abs/1801.06452)
    – A possible alternative to the Reichardt detector for obstacles that approach the robot.

No paper was found that uses neural fields for following *and* guiding a person; the contribution of this
project is combining the models above for social navigation.
