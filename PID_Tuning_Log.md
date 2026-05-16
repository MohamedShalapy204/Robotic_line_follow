# PID Tuning Log

| Trial | Kp | Ki | Kd | Base Speed | Lap Time (s) | Max Lateral Deviation | Cross-Track RMS Error | Notes / Behavior Observed |
| :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :--- |
| 1 | 1.0 | 0.0 | 0.0 | 0.1 | | | | Initial test with Proportional only. |
| 2 | | | | | | | | |
| 3 | | | | | | | | |
| 4 | | | | | | | | |
| 5 | | | | | | | | |

## Guidelines for Tuning
1. **Start with Kp:** Increase `Kp` until the robot oscillates around the line.
2. **Add Kd:** Increase `Kd` to dampen the oscillation and smooth out the turns.
3. **Add Ki (Optional):** Increase `Ki` slightly if the robot consistently stays slightly off-center on straightaways. Be careful, as too much `Ki` can cause instability.
4. **Base Speed:** As you increase speed, you will likely need to re-tune `Kp` and `Kd` (usually increasing both).
