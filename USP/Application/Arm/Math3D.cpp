#include "Math3D.h"

#include <math.h>

namespace Math3D
{

namespace
{
constexpr float kEps = 1e-5f;
constexpr float kPi = 3.14159265358979323846f;

Vec6 getColumn6(const Mat6 &A, uint8_t c)
{
    Vec6 out = {};
    for (uint8_t r = 0; r < 6; ++r)
    {
        out.v[r] = A.m[r][c];
    }
    return out;
}

void setColumn6(Mat6 *A, uint8_t c, const Vec6 &x)
{
    if (A == 0)
    {
        return;
    }
    for (uint8_t r = 0; r < 6; ++r)
    {
        A->m[r][c] = x.v[r];
    }
}

float clampf(float x, float lo, float hi)
{
    if (x < lo)
    {
        return lo;
    }
    if (x > hi)
    {
        return hi;
    }
    return x;
}

} // namespace

bool nearZero(float value)
{
    return fabsf(value) < kEps;
}

Vec3 makeVec3(float x, float y, float z)
{
    Vec3 out = {{x, y, z}};
    return out;
}

Vec6 makeVec6(float v0, float v1, float v2, float v3, float v4, float v5)
{
    Vec6 out = {{v0, v1, v2, v3, v4, v5}};
    return out;
}

Mat3 eye3()
{
    Mat3 out = {};
    out.m[0][0] = 1.0f;
    out.m[1][1] = 1.0f;
    out.m[2][2] = 1.0f;
    return out;
}

Mat4 eye4()
{
    Mat4 out = {};
    out.m[0][0] = 1.0f;
    out.m[1][1] = 1.0f;
    out.m[2][2] = 1.0f;
    out.m[3][3] = 1.0f;
    return out;
}

Mat6 eye6()
{
    Mat6 out = {};
    for (uint8_t i = 0; i < 6; ++i)
    {
        out.m[i][i] = 1.0f;
    }
    return out;
}

Mat3 zeros3()
{
    Mat3 out = {};
    return out;
}

Mat4 zeros4()
{
    Mat4 out = {};
    return out;
}

Mat6 zeros6()
{
    Mat6 out = {};
    return out;
}

Vec3 vec3Add(const Vec3 &a, const Vec3 &b)
{
    Vec3 out = {};
    for (uint8_t i = 0; i < 3; ++i)
    {
        out.v[i] = a.v[i] + b.v[i];
    }
    return out;
}

Vec3 vec3Sub(const Vec3 &a, const Vec3 &b)
{
    Vec3 out = {};
    for (uint8_t i = 0; i < 3; ++i)
    {
        out.v[i] = a.v[i] - b.v[i];
    }
    return out;
}

Vec3 vec3Scale(const Vec3 &a, float s)
{
    Vec3 out = {};
    for (uint8_t i = 0; i < 3; ++i)
    {
        out.v[i] = a.v[i] * s;
    }
    return out;
}

float vec3Norm(const Vec3 &a)
{
    float sum = 0.0f;
    for (uint8_t i = 0; i < 3; ++i)
    {
        sum += a.v[i] * a.v[i];
    }
    return sqrtf(sum);
}

Vec6 vec6Add(const Vec6 &a, const Vec6 &b)
{
    Vec6 out = {};
    for (uint8_t i = 0; i < 6; ++i)
    {
        out.v[i] = a.v[i] + b.v[i];
    }
    return out;
}

Vec6 vec6Sub(const Vec6 &a, const Vec6 &b)
{
    Vec6 out = {};
    for (uint8_t i = 0; i < 6; ++i)
    {
        out.v[i] = a.v[i] - b.v[i];
    }
    return out;
}

Vec6 vec6Scale(const Vec6 &a, float s)
{
    Vec6 out = {};
    for (uint8_t i = 0; i < 6; ++i)
    {
        out.v[i] = a.v[i] * s;
    }
    return out;
}

float vec6Norm(const Vec6 &a)
{
    float sum = 0.0f;
    for (uint8_t i = 0; i < 6; ++i)
    {
        sum += a.v[i] * a.v[i];
    }
    return sqrtf(sum);
}

float vec6Norm3(const Vec6 &a)
{
    float sum = 0.0f;
    for (uint8_t i = 0; i < 3; ++i)
    {
        sum += a.v[i] * a.v[i];
    }
    return sqrtf(sum);
}

float vec6NormLinear(const Vec6 &a)
{
    float sum = 0.0f;
    for (uint8_t i = 3; i < 6; ++i)
    {
        sum += a.v[i] * a.v[i];
    }
    return sqrtf(sum);
}

Mat3 mat3Add(const Mat3 &a, const Mat3 &b)
{
    Mat3 out = {};
    for (uint8_t r = 0; r < 3; ++r)
    {
        for (uint8_t c = 0; c < 3; ++c)
        {
            out.m[r][c] = a.m[r][c] + b.m[r][c];
        }
    }
    return out;
}

Mat3 mat3Sub(const Mat3 &a, const Mat3 &b)
{
    Mat3 out = {};
    for (uint8_t r = 0; r < 3; ++r)
    {
        for (uint8_t c = 0; c < 3; ++c)
        {
            out.m[r][c] = a.m[r][c] - b.m[r][c];
        }
    }
    return out;
}

Mat3 mat3Scale(const Mat3 &a, float s)
{
    Mat3 out = {};
    for (uint8_t r = 0; r < 3; ++r)
    {
        for (uint8_t c = 0; c < 3; ++c)
        {
            out.m[r][c] = a.m[r][c] * s;
        }
    }
    return out;
}

Mat3 mat3Mul(const Mat3 &a, const Mat3 &b)
{
    Mat3 out = {};
    for (uint8_t r = 0; r < 3; ++r)
    {
        for (uint8_t c = 0; c < 3; ++c)
        {
            float sum = 0.0f;
            for (uint8_t k = 0; k < 3; ++k)
            {
                sum += a.m[r][k] * b.m[k][c];
            }
            out.m[r][c] = sum;
        }
    }
    return out;
}

Mat3 mat3Transpose(const Mat3 &a)
{
    Mat3 out = {};
    for (uint8_t r = 0; r < 3; ++r)
    {
        for (uint8_t c = 0; c < 3; ++c)
        {
            out.m[r][c] = a.m[c][r];
        }
    }
    return out;
}

Vec3 mat3MulVec3(const Mat3 &a, const Vec3 &x)
{
    Vec3 out = {};
    for (uint8_t r = 0; r < 3; ++r)
    {
        float sum = 0.0f;
        for (uint8_t c = 0; c < 3; ++c)
        {
            sum += a.m[r][c] * x.v[c];
        }
        out.v[r] = sum;
    }
    return out;
}

float mat3Trace(const Mat3 &a)
{
    return a.m[0][0] + a.m[1][1] + a.m[2][2];
}

Mat4 mat4Mul(const Mat4 &a, const Mat4 &b)
{
    Mat4 out = {};
    for (uint8_t r = 0; r < 4; ++r)
    {
        for (uint8_t c = 0; c < 4; ++c)
        {
            float sum = 0.0f;
            for (uint8_t k = 0; k < 4; ++k)
            {
                sum += a.m[r][k] * b.m[k][c];
            }
            out.m[r][c] = sum;
        }
    }
    return out;
}

Mat4 mat4Transpose(const Mat4 &a)
{
    Mat4 out = {};
    for (uint8_t r = 0; r < 4; ++r)
    {
        for (uint8_t c = 0; c < 4; ++c)
        {
            out.m[r][c] = a.m[c][r];
        }
    }
    return out;
}

Mat6 mat6Add(const Mat6 &a, const Mat6 &b)
{
    Mat6 out = {};
    for (uint8_t r = 0; r < 6; ++r)
    {
        for (uint8_t c = 0; c < 6; ++c)
        {
            out.m[r][c] = a.m[r][c] + b.m[r][c];
        }
    }
    return out;
}

Mat6 mat6Scale(const Mat6 &a, float s)
{
    Mat6 out = {};
    for (uint8_t r = 0; r < 6; ++r)
    {
        for (uint8_t c = 0; c < 6; ++c)
        {
            out.m[r][c] = a.m[r][c] * s;
        }
    }
    return out;
}

Mat6 mat6Mul(const Mat6 &a, const Mat6 &b)
{
    Mat6 out = {};
    for (uint8_t r = 0; r < 6; ++r)
    {
        for (uint8_t c = 0; c < 6; ++c)
        {
            float sum = 0.0f;
            for (uint8_t k = 0; k < 6; ++k)
            {
                sum += a.m[r][k] * b.m[k][c];
            }
            out.m[r][c] = sum;
        }
    }
    return out;
}

Mat6 mat6Transpose(const Mat6 &a)
{
    Mat6 out = {};
    for (uint8_t r = 0; r < 6; ++r)
    {
        for (uint8_t c = 0; c < 6; ++c)
        {
            out.m[r][c] = a.m[c][r];
        }
    }
    return out;
}

Vec6 mat6MulVec6(const Mat6 &a, const Vec6 &x)
{
    Vec6 out = {};
    for (uint8_t r = 0; r < 6; ++r)
    {
        float sum = 0.0f;
        for (uint8_t c = 0; c < 6; ++c)
        {
            sum += a.m[r][c] * x.v[c];
        }
        out.v[r] = sum;
    }
    return out;
}

bool solveLinear6x6(const Mat6 &A, const Vec6 &b, Vec6 *x)
{
    if (x == 0)
    {
        return false;
    }

    // 高斯消元增广矩阵 [A|b]。
    float aug[6][7] = {};
    for (uint8_t r = 0; r < 6; ++r)
    {
        for (uint8_t c = 0; c < 6; ++c)
        {
            aug[r][c] = A.m[r][c];
        }
        aug[r][6] = b.v[r];
    }

    for (uint8_t pivot = 0; pivot < 6; ++pivot)
    {
        // 选主元，降低数值误差。
        uint8_t best = pivot;
        float best_val = fabsf(aug[pivot][pivot]);
        for (uint8_t r = static_cast<uint8_t>(pivot + 1); r < 6; ++r)
        {
            const float v = fabsf(aug[r][pivot]);
            if (v > best_val)
            {
                best = r;
                best_val = v;
            }
        }

        if (best_val < kEps)
        {
            return false;
        }

        if (best != pivot)
        {
            for (uint8_t c = 0; c < 7; ++c)
            {
                const float t = aug[pivot][c];
                aug[pivot][c] = aug[best][c];
                aug[best][c] = t;
            }
        }

        const float diag = aug[pivot][pivot];
        for (uint8_t c = pivot; c < 7; ++c)
        {
            aug[pivot][c] /= diag;
        }

        for (uint8_t r = 0; r < 6; ++r)
        {
            if (r == pivot)
            {
                continue;
            }
            const float factor = aug[r][pivot];
            for (uint8_t c = pivot; c < 7; ++c)
            {
                aug[r][c] -= factor * aug[pivot][c];
            }
        }
    }

    for (uint8_t i = 0; i < 6; ++i)
    {
        x->v[i] = aug[i][6];
    }
    return true;
}

void trans_to_Rp(const Mat4 &T, Mat3 *R, Vec3 *p)
{
    if (R != 0)
    {
        for (uint8_t r = 0; r < 3; ++r)
        {
            for (uint8_t c = 0; c < 3; ++c)
            {
                R->m[r][c] = T.m[r][c];
            }
        }
    }

    if (p != 0)
    {
        p->v[0] = T.m[0][3];
        p->v[1] = T.m[1][3];
        p->v[2] = T.m[2][3];
    }
}

Mat4 trans_inv(const Mat4 &T)
{
    Mat3 R = {};
    Vec3 p = {};
    trans_to_Rp(T, &R, &p);

    Mat3 Rt = mat3Transpose(R);
    Vec3 p_inv = mat3MulVec3(Rt, vec3Scale(p, -1.0f));

    Mat4 out = eye4();
    for (uint8_t r = 0; r < 3; ++r)
    {
        for (uint8_t c = 0; c < 3; ++c)
        {
            out.m[r][c] = Rt.m[r][c];
        }
    }
    out.m[0][3] = p_inv.v[0];
    out.m[1][3] = p_inv.v[1];
    out.m[2][3] = p_inv.v[2];
    return out;
}

Mat3 vec_to_so3(const Vec3 &omega)
{
    Mat3 so3 = {};
    so3.m[0][1] = -omega.v[2];
    so3.m[0][2] = omega.v[1];
    so3.m[1][0] = omega.v[2];
    so3.m[1][2] = -omega.v[0];
    so3.m[2][0] = -omega.v[1];
    so3.m[2][1] = omega.v[0];
    return so3;
}

Vec3 so3_to_vec(const Mat3 &so3)
{
    return makeVec3(so3.m[2][1], so3.m[0][2], so3.m[1][0]);
}

Mat4 vec_to_se3(const Vec6 &V)
{
    Mat4 se3 = {};
    const Vec3 omg = makeVec3(V.v[0], V.v[1], V.v[2]);
    const Mat3 so3 = vec_to_so3(omg);

    for (uint8_t r = 0; r < 3; ++r)
    {
        for (uint8_t c = 0; c < 3; ++c)
        {
            se3.m[r][c] = so3.m[r][c];
        }
    }

    se3.m[0][3] = V.v[3];
    se3.m[1][3] = V.v[4];
    se3.m[2][3] = V.v[5];
    return se3;
}

Vec6 se3_to_vec(const Mat4 &se3)
{
    return makeVec6(se3.m[2][1], se3.m[0][2], se3.m[1][0],
                    se3.m[0][3], se3.m[1][3], se3.m[2][3]);
}

void axis_angle(const Vec3 &vec, Vec3 *axis, float *angle)
{
    const float theta = vec3Norm(vec);
    if (angle != 0)
    {
        *angle = theta;
    }

    if (axis != 0)
    {
        if (nearZero(theta))
        {
            *axis = makeVec3(0.0f, 0.0f, 0.0f);
        }
        else
        {
            *axis = vec3Scale(vec, 1.0f / theta);
        }
    }
}

Mat3 matrix_exp3(const Mat3 &so3)
{
    const Vec3 omgtheta = so3_to_vec(so3);
    const float n = vec3Norm(omgtheta);

    if (nearZero(n))
    {
        return eye3();
    }

    float theta = 0.0f;
    axis_angle(omgtheta, 0, &theta);
    const Mat3 omgmat = mat3Scale(so3, 1.0f / theta);

    // Rodrigues 公式。
    Mat3 out = eye3();
    out = mat3Add(out, mat3Scale(omgmat, sinf(theta)));
    out = mat3Add(out, mat3Scale(mat3Mul(omgmat, omgmat), 1.0f - cosf(theta)));
    return out;
}

Mat4 matrix_exp6(const Mat4 &se3)
{
    Mat3 so3 = {};
    for (uint8_t r = 0; r < 3; ++r)
    {
        for (uint8_t c = 0; c < 3; ++c)
        {
            so3.m[r][c] = se3.m[r][c];
        }
    }

    const Vec3 omgtheta = so3_to_vec(so3);
    const float omg_norm = vec3Norm(omgtheta);

    Mat4 T = eye4();
    if (nearZero(omg_norm))
    {
        // 纯平移情形。
        T.m[0][3] = se3.m[0][3];
        T.m[1][3] = se3.m[1][3];
        T.m[2][3] = se3.m[2][3];
        return T;
    }

    float theta = 0.0f;
    axis_angle(omgtheta, 0, &theta);
    const Mat3 omgmat = mat3Scale(so3, 1.0f / theta);
    const Mat3 R = matrix_exp3(so3);

    // SE(3) 指数中的 V(theta) 项。
    Mat3 V = eye3();
    V = mat3Scale(V, theta);
    V = mat3Add(V, mat3Scale(omgmat, 1.0f - cosf(theta)));
    V = mat3Add(V, mat3Scale(mat3Mul(omgmat, omgmat), theta - sinf(theta)));

    const Vec3 v = makeVec3(se3.m[0][3], se3.m[1][3], se3.m[2][3]);
    const Vec3 p = mat3MulVec3(V, vec3Scale(v, 1.0f / theta));

    for (uint8_t r = 0; r < 3; ++r)
    {
        for (uint8_t c = 0; c < 3; ++c)
        {
            T.m[r][c] = R.m[r][c];
        }
    }
    T.m[0][3] = p.v[0];
    T.m[1][3] = p.v[1];
    T.m[2][3] = p.v[2];
    return T;
}

Mat3 matrix_log3(const Mat3 &R)
{
    const float acosinput = clampf((mat3Trace(R) - 1.0f) * 0.5f, -1.0f, 1.0f);

    if (acosinput >= 1.0f - kEps)
    {
        return zeros3();
    }

    if (acosinput <= -1.0f + kEps)
    {
        // theta 接近 pi 时走特例分支，避免除零。
        Vec3 omg = makeVec3(0.0f, 0.0f, 0.0f);

        if (!nearZero(1.0f + R.m[2][2]))
        {
            const float s = 1.0f / sqrtf(2.0f * (1.0f + R.m[2][2]));
            omg = makeVec3(R.m[0][2] * s, R.m[1][2] * s, (1.0f + R.m[2][2]) * s);
        }
        else if (!nearZero(1.0f + R.m[1][1]))
        {
            const float s = 1.0f / sqrtf(2.0f * (1.0f + R.m[1][1]));
            omg = makeVec3(R.m[0][1] * s, (1.0f + R.m[1][1]) * s, R.m[2][1] * s);
        }
        else
        {
            const float s = 1.0f / sqrtf(2.0f * (1.0f + R.m[0][0]));
            omg = makeVec3((1.0f + R.m[0][0]) * s, R.m[1][0] * s, R.m[2][0] * s);
        }

        return vec_to_so3(vec3Scale(omg, kPi));
    }

    const float theta = acosf(acosinput);
    const Mat3 Rt = mat3Transpose(R);
    const Mat3 skew = mat3Sub(R, Rt);
    return mat3Scale(skew, theta / (2.0f * sinf(theta)));
}

Mat4 matrix_log6(const Mat4 &T)
{
    Mat3 R = {};
    Vec3 p = {};
    trans_to_Rp(T, &R, &p);

    const Mat3 omgmat = matrix_log3(R);

    if (nearZero(vec3Norm(so3_to_vec(omgmat))))
    {
        // 纯平移情形。
        Mat4 out = zeros4();
        out.m[0][3] = p.v[0];
        out.m[1][3] = p.v[1];
        out.m[2][3] = p.v[2];
        return out;
    }

    const float theta = acosf(clampf((mat3Trace(R) - 1.0f) * 0.5f, -1.0f, 1.0f));
    const Mat3 I = eye3();
    const Mat3 term1 = mat3Sub(I, mat3Scale(omgmat, 0.5f));

    const float half_theta = theta * 0.5f;
    const float cot_half = cosf(half_theta) / sinf(half_theta);
    const float coeff = (1.0f / theta) - 0.5f * cot_half;

    Mat3 omg2 = mat3Mul(omgmat, omgmat);
    omg2 = mat3Scale(omg2, coeff / theta);

    const Mat3 Ginv = mat3Add(term1, omg2);
    const Vec3 v = mat3MulVec3(Ginv, p);

    Mat4 out = zeros4();
    for (uint8_t r = 0; r < 3; ++r)
    {
        for (uint8_t c = 0; c < 3; ++c)
        {
            out.m[r][c] = omgmat.m[r][c];
        }
    }
    out.m[0][3] = v.v[0];
    out.m[1][3] = v.v[1];
    out.m[2][3] = v.v[2];
    return out;
}

Mat6 adjoint(const Mat4 &T)
{
    Mat3 R = {};
    Vec3 p = {};
    trans_to_Rp(T, &R, &p);

    Mat6 out = zeros6();
    for (uint8_t r = 0; r < 3; ++r)
    {
        for (uint8_t c = 0; c < 3; ++c)
        {
            out.m[r][c] = R.m[r][c];
            out.m[r + 3][c + 3] = R.m[r][c];
        }
    }

    // Ad_T = [R, 0; p^R, R]
    const Mat3 p_hat = vec_to_so3(p);
    const Mat3 p_hat_R = mat3Mul(p_hat, R);
    for (uint8_t r = 0; r < 3; ++r)
    {
        for (uint8_t c = 0; c < 3; ++c)
        {
            out.m[r + 3][c] = p_hat_R.m[r][c];
        }
    }

    return out;
}

Mat4 fkine(const Mat4 &M, const Mat6 &Slist, const float q[kDoF])
{
    // PoE: T = exp([S1]q1)...exp([Sn]qn)M
    Mat4 T = M;

    for (int i = static_cast<int>(kDoF) - 1; i >= 0; --i)
    {
        const Vec6 Si = getColumn6(Slist, static_cast<uint8_t>(i));
        const Vec6 Si_q = vec6Scale(Si, q[i]);
        const Mat4 exp6 = matrix_exp6(vec_to_se3(Si_q));
        T = mat4Mul(exp6, T);
    }

    return T;
}

Mat6 cal_Jacobian(const Mat6 &Slist, const float q[kDoF])
{
    // 空间雅可比递推：Ji = Ad(exp([S1]q1)...exp([S(i-1)]q(i-1))) * Si
    Mat6 J = Slist;
    Mat4 T = eye4();

    for (uint8_t i = 1; i < kDoF; ++i)
    {
        const Vec6 Si_prev = getColumn6(Slist, static_cast<uint8_t>(i - 1));
        const Vec6 Si_prev_q = vec6Scale(Si_prev, q[i - 1]);
        T = mat4Mul(T, matrix_exp6(vec_to_se3(Si_prev_q)));

        const Mat6 AdT = adjoint(T);
        const Vec6 Si = getColumn6(Slist, i);
        const Vec6 Ji = mat6MulVec6(AdT, Si);
        setColumn6(&J, i, Ji);
    }

    return J;
}

bool ikine(const Mat6 &Slist,
           const Mat4 &M,
           const Mat4 &T,
           const float q0[kDoF],
           float eomg,
           float ev,
           float q_out[kDoF],
           float lambda,
           float max_step,
           int max_iterations)
{
    if (q_out == 0 || q0 == 0)
    {
        return false;
    }

    float q[kDoF] = {};
    for (uint8_t i = 0; i < kDoF; ++i)
    {
        q[i] = q0[i];
    }

    bool success = false;

    for (int iter = 0; iter < max_iterations; ++iter)
    {
        const Mat4 Tsb = fkine(M, Slist, q);
        const Mat4 T_err = mat4Mul(trans_inv(Tsb), T);
        const Vec6 Vb = se3_to_vec(matrix_log6(T_err));
        const Vec6 Vs = mat6MulVec6(adjoint(Tsb), Vb);

        const float omg_err = vec6Norm3(Vs);
        const float lin_err = vec6NormLinear(Vs);
        if (omg_err <= eomg && lin_err <= ev)
        {
            success = true;
            break;
        }

        const Mat6 J = cal_Jacobian(Slist, q);
        const Mat6 Jt = mat6Transpose(J);

        // DLS: Δq = J^T (JJ^T + λ²I)^(-1) Vs
        Mat6 A = mat6Mul(J, Jt);
        const Mat6 I6 = eye6();
        A = mat6Add(A, mat6Scale(I6, lambda * lambda));

        Vec6 y = {};
        if (!solveLinear6x6(A, Vs, &y))
        {
            break;
        }

        Vec6 delta_q = mat6MulVec6(Jt, y);
        // 迭代步长限幅，抑制大步震荡。
        const float dq_norm = vec6Norm(delta_q);
        if (dq_norm > max_step && !nearZero(dq_norm))
        {
            delta_q = vec6Scale(delta_q, max_step / dq_norm);
        }

        for (uint8_t i = 0; i < kDoF; ++i)
        {
            q[i] += delta_q.v[i];
        }
    }

    for (uint8_t i = 0; i < kDoF; ++i)
    {
        q_out[i] = q[i];
    }

    return success;
}

Mat4 transformation_mdh(float alpha,
                        float a,
                        float d,
                        float theta,
                        bool sigma,
                        float q)
{
    // sigma=true: 转动关节（theta 变量）；sigma=false: 移动关节（d 变量）。
    if (sigma)
    {
        theta = theta + q;
    }
    else
    {
        d = d + q;
    }

    const float ct = cosf(theta);
    const float st = sinf(theta);
    const float ca = cosf(alpha);
    const float sa = sinf(alpha);

    Mat4 T = eye4();

    T.m[0][0] = ct;
    T.m[0][1] = -st;
    T.m[0][2] = 0.0f;
    T.m[0][3] = a;

    T.m[1][0] = st * ca;
    T.m[1][1] = ct * ca;
    T.m[1][2] = -sa;
    T.m[1][3] = -d * sa;

    T.m[2][0] = st * sa;
    T.m[2][1] = ct * sa;
    T.m[2][2] = ca;
    T.m[2][3] = d * ca;

    return T;
}

} // namespace Math3D
