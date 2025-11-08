void initKalmanPosVel(void)
{
  current_prob.m11 = 1.0f;
  current_prob.m21 = 0.0f;
  current_prob.m12 = 0.0f;
  current_prob.m22 = 1.0f;

  quadprops.height = 0.0f;
  quadprops.kalmanvel_z = 0.0f;
  quadprops.baro_height = 0.0f;
}

#define timeslice 0.007f // 140 Hz
#define var_acc 1.0f

void KalmanPosVel()
{
  const float Q11 = var_acc * 0.25f * (timeslice * timeslice * timeslice * timeslice);
  const float Q12 = var_acc * 0.5f * (timeslice * timeslice * timeslice);
  const float Q21 = var_acc * 0.5f * (timeslice * timeslice * timeslice);
  const float Q22 = var_acc * (timeslice * timeslice);
  const float R11 = 0.008f;

  float ps1 = quadprops.height + timeslice * quadprops.kalmanvel_z;
  float ps2 = quadprops.kalmanvel_z;
  float opt = timeslice * current_prob.m22;
  float pp12 = current_prob.m12 + opt + Q12;

  float pp21 = current_prob.m21 + opt;
  float pp11 = current_prob.m11 + timeslice * (current_prob.m12 + pp21) + Q11;
  pp21 += Q21;
  float pp22 = current_prob.m22 + Q22;

  float inn = quadprops.baro_height - ps1;
  float ic = pp11 + R11;

  float kg1 = pp11 / ic;
  float kg2 = pp21 / ic;

  quadprops.height = ps1 + kg1 * inn;
  quadprops.kalmanvel_z = ps2 + kg2 * inn;

  opt = 1.0f - kg1;
  current_prob.m11 = pp11 * opt;
  current_prob.m12 = pp12 * opt;
  current_prob.m21 = pp21 - pp11 * kg2;
  current_prob.m22 = pp22 - pp12 * kg2;
}
