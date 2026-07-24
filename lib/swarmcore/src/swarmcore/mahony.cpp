#include "mahony.h"

namespace sc {

void Mahony::update(float gx, float gy, float gz,
                    float ax, float ay, float az, float dt) {
  // Feedback only when the accelerometer measurement is usable.
  const float anorm = sqrtf(ax * ax + ay * ay + az * az);
  if (anorm > 1e-6f) {
    ax /= anorm; ay /= anorm; az /= anorm;

    // Estimated gravity direction from the quaternion.
    const float vx = 2.0f * (q1_ * q3_ - q0_ * q2_);
    const float vy = 2.0f * (q0_ * q1_ + q2_ * q3_);
    const float vz = q0_ * q0_ - q1_ * q1_ - q2_ * q2_ + q3_ * q3_;

    // Error = cross(measured, estimated).
    const float ex = ay * vz - az * vy;
    const float ey = az * vx - ax * vz;
    const float ez = ax * vy - ay * vx;

    if (p_.twoKi > 0.0f) {
      ix_ += p_.twoKi * ex * dt;
      iy_ += p_.twoKi * ey * dt;
      iz_ += p_.twoKi * ez * dt;
      gx += ix_; gy += iy_; gz += iz_;
    }
    gx += p_.twoKp * ex;
    gy += p_.twoKp * ey;
    gz += p_.twoKp * ez;
  }

  // Integrate quaternion rate.
  const float halfDt = 0.5f * dt;
  const float qa = q0_, qb = q1_, qc = q2_;
  q0_ += (-qb * gx - qc * gy - q3_ * gz) * halfDt;
  q1_ += (qa * gx + qc * gz - q3_ * gy) * halfDt;
  q2_ += (qa * gy - qb * gz + q3_ * gx) * halfDt;
  q3_ += (qa * gz + qb * gy - qc * gx) * halfDt;

  // Normalize.
  const float n = sqrtf(q0_ * q0_ + q1_ * q1_ + q2_ * q2_ + q3_ * q3_);
  if (n > 1e-9f) { q0_ /= n; q1_ /= n; q2_ /= n; q3_ /= n; }
}

}  // namespace sc
