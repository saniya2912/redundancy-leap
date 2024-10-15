from main_redundancy import LeapNode_Taucontrol

leap=LeapNode_Taucontrol()

while True:
    leap.set_desired_torque([-0.00286614,  0.0168139,   0.01201024,  0.00665909,  0.,          0.,
  0.,          0.,          0.,          0.,          0.,          0.,
  0, 0,  0.0099706,   0.00432057])