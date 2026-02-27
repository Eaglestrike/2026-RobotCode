# 2026 Robot Code

## Shooter Turret

<img src="media/Turret_Angles.JPG" alt="Turret Angles" style="max-width:400px; width:100%; height:auto;" />
<p><em>Figure: Turret coordinate system.</em></p>

***Default Value for Harmonic Drive (from Jerry Robotics Arm)*** <br>

  KS = 0.19 (or 0.2095)   V <br>
  KV = 3.0                V·s/rad <br>
  KA = 0.01 (or 0.0)      V·s^2/rad <br>

  KP = 40.0               V/rad <br>
  KI = 100.0              V/(rad·s) <br>
  KD = 1.2 (or 0.7)       V·s/rad <br>

  IZONE_DEG = 0.2 (or 0.1)          degree <br>
  I_MAX_VOLTS = 1.0 (or whatever)   V <br>


kP 40
kI 100
kD 0.7
izone_deg 0.2
i_max_volts 1

kS 0.19
kV 3
kA 0.01
kGCos 0.05

max vel 6
max acc 12