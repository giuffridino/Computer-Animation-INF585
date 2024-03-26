#pragma once

#include "cgp/cgp.hpp"

struct dual_quaternion {
    cgp::quaternion q0;
    cgp::quaternion qe;

    dual_quaternion(const cgp::quaternion& realPart, const cgp::quaternion& dualPart);

    dual_quaternion operator+(const dual_quaternion& other) const;

    dual_quaternion operator-(const dual_quaternion& other) const;

    dual_quaternion operator*(const dual_quaternion& other) const;

    dual_quaternion operator/(const dual_quaternion& other) const;

    dual_quaternion operator*(const float prod) const;

    static dual_quaternion normalize(const dual_quaternion& dual_quat);
};