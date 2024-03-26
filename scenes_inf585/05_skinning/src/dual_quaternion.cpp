#include "dual_quaternion.hpp"

// dual_quaternion::dual_quaternion(const cgp::quaternion& realPart, const cgp::quaternion& dualPart)
//     : q0(realPart), qe(dualPart) {}

// dual_quaternion dual_quaternion::operator+(const dual_quaternion& other) const {
//     return dual_quaternion(q0 + other.q0, qe + other.qe);
// }

// dual_quaternion dual_quaternion::operator-(const dual_quaternion& other) const {
//     return dual_quaternion(q0 - other.q0, qe - other.qe);
// }

// dual_quaternion dual_quaternion::operator*(const dual_quaternion& other) const {
//     return dual_quaternion(q0 * other.q0, q0 * other.qe + qe * other.q0);
// }

// dual_quaternion dual_quaternion::operator/(const dual_quaternion& other) const {
//     return dual_quaternion(q0 / other.q0, (q0 * other.qe - qe * other.q0) / (other.q0 * other.q0));
// }

// dual_quaternion dual_quaternion::operator*(const float prod) const {
//     return dual_quaternion(q0 * prod, qe * prod);
// }

// dual_quaternion dual_quaternion::normalize(const dual_quaternion& dual_quat) {
//     return dual_quaternion(dual_quat.q0 / norm(dual_quat.q0), dual_quat.qe / norm(dual_quat.q0));
// }