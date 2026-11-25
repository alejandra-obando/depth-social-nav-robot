# Exam rules (final exam, third term – Neurocontrol 2)

Source: the professor's rubric (12 November 2026), the class of 11 November and his answers of 14 November.
Rules agreed in class on 18 November should be added at the end of this file.

## Date and time

- **Monday 30 November 2026.**
- Each group has **30 minutes**: maximum **20 minutes** for the physical task and **10 minutes** for the presentation (with slides).

## The room

- About 7 m × 7 m. The robot travels along the **diagonal**, from one corner to the opposite one.
- **Obstacles:** the 36 tablets of the maze (25 × 25 cm, 2.5 cm thick) in stacks, probably 9 stacks of 4 tablets = **10 cm high**.
  There are also briefcases, which may be taller than the robot and **may be moved during the test**.
- **2 or 3 people** walk around the room while the robot works.

## Phase 1 – Follow

1. The robot starts at the start corner.
2. It follows the designated person along the diagonal to the opposite corner.
3. It avoids static and moving obstacles and people.
4. When it arrives, the **robot itself must recognise the corner** and give a **visual signal** that phase 1 is over.

## Phase 2 – Guide

1. The robot guides the person back to the start corner. The person walks **behind** the robot.
2. The robot must keep checking that the person is still following it.
3. The position of the start corner **may be stored** (for example with wheel odometry and the LiDAR map).

## Rules for both phases

- The distance between the robot and the person must be **at most 1.50 m at all times**.
- The link with the person **must be made through an image** (a pattern or a QR code).
  Each group chooses the type, size and position of the sign, but it **must be worn on the front and on the back** of the person.
- If the robot loses the person, the person will be asked to stop.
- **Using neural structures is rewarded.**

## Faults

| Type | Penalty | Examples |
|------|---------|----------|
| Serious | −0.5 | Crash without an avoidance manoeuvre; losing the link with the person; getting lost in phase 2. |
| Minor | −0.25 | Bumping into something during an avoidance manoeuvre; a mechanical failure that is fixed; each extra minute over the limit. |

- **One restart** of the task is allowed.

## Grading

- **70 %** physical task, **30 %** presentation (complete, clear, concise, coherent and precise).
- **Peer evaluation:** each group grades another group with the rubric, by human judgement (no precise measurements).
