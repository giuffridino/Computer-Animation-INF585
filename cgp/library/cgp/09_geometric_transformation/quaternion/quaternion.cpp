#include "cgp/01_base/base.hpp"
#include "quaternion.hpp"
#include <iostream>

namespace cgp
{
	std::string type_str(quaternion const&)
	{
		return "quaternion";
	}

    quaternion conjugate(quaternion const& q)
    {
        return quaternion{ -q.xyz(), q.w };
    }
    quaternion inverse(quaternion const& q)
    {
        return conjugate(q) / dot(q, q);
    }


    quaternion& operator*=(quaternion& a, quaternion const& b)
    {
        a = a * b;
        return a;
    }

    quaternion  operator*(quaternion const& a, quaternion const& b)
    {
        return quaternion{
            a.x * b.w + a.w * b.x + a.y * b.z - a.z * b.y,
            a.y * b.w + a.w * b.y + a.z * b.x - a.x * b.z,
            a.z * b.w + a.w * b.z + a.x * b.y - a.y * b.x,
            a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z
        };
    }

    quaternion operator*(quaternion const& a, float b)
    {
        quaternion res = a;
        res *= b;
        return res;
    }
    quaternion operator*(float a, quaternion const& b)
    {
        quaternion res = b;
        res *= a;
        return res;
    }


    quaternion& operator/=(quaternion& a, quaternion const& b)
    {
        a = a / b;
        return a;
    }
    quaternion  operator/(quaternion const& a, quaternion const& b)
    {
        return a * inverse(b);
    }
    quaternion operator/(quaternion const& a, float b)
    {
        quaternion res = a;
        res /= b;
        return res;
    }
    quaternion operator/(float a, quaternion const& b)
    {
        quaternion res = inverse(b);
        return a * res;
    }

    quaternion& operator+=(quaternion& a, quaternion const& b)
    {
        static_cast<vec4&>(a) += b;
        return a;
    }
    quaternion operator+(quaternion const& a, quaternion const& b)
    {
        quaternion res = a;
        res += b;
        return res;
    }
    quaternion& operator-=(quaternion& a, quaternion const& b)
    {
        static_cast<vec4&>(a) -= b;
        return a;
    }
    quaternion operator-(quaternion const& a, quaternion const& b)
    {
        quaternion res = a;
        res -= b;
        return res;
    }
    quaternion normalize(quaternion const& q)
    {
        return q/norm(q);
    }
    quaternion negate(quaternion const& q)
    {
        return {-q.x, -q.y, -q.z, -q.w};
    }
    quaternion slerp(quaternion const& q1, quaternion const& q2, float t)
    {
        quaternion quat1 = normalize(q1);
        quaternion quat2 = normalize(q2);

        float dotProduct = quat1.x * quat2.x + quat1.y * quat2.y + quat1.z * quat2.z + quat1.w * quat2.w;
        dotProduct = std::fmax(-1.0f, std::fmin(1.0f, dotProduct));
        if (std::abs(dotProduct - 1.0f) < std::numeric_limits<float>::epsilon())
        {
            return quat1;
        }

        if (dotProduct < 0.0f)
        {
            quat2 = negate(quat2);
            dotProduct = -dotProduct;
        }
        dotProduct = std::fmax(-1.0f, std::fmin(1.0f, dotProduct));
        float theta = std::acos(dotProduct);
        float sinTheta = std::sin(theta);
        if (std::abs(sinTheta) < std::numeric_limits<float>::epsilon())
        {
            return quat1;
        }

        float weight1 = std::sin((1.0f - t) * theta) / sinTheta;
        float weight2 = std::sin(t * theta) / sinTheta;

        quaternion result;
        result.x = weight1 * quat1.x + weight2 * quat2.x;
        result.y = weight1 * quat1.y + weight2 * quat2.y;
        result.z = weight1 * quat1.z + weight2 * quat2.z;
        result.w = weight1 * quat1.w + weight2 * quat2.w;
        // std::cout << "result:" << result << "\n";
        return normalize(result);
    }
    quaternion lerp(const quaternion& q1, const quaternion& q2, float t)
    {
        t = std::fmax(0.0f, std::fmin(1.0f, t));
        return normalize((1 - t) * q1 + t * q2);
    }

    std::istream& operator>>(std::istream& stream, quaternion& data)
    {
        stream >> data.x;
        stream >> data.y;
        stream >> data.z;
        stream >> data.w;

        return stream;
    }
}