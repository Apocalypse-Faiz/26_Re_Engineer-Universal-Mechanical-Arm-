#ifndef __MATH3D_H__
#define __MATH3D_H__

#include <stdint.h>

namespace Math3D
{

/**
 * @brief 机械臂自由度常量（固定 6 轴）。
 */
static const uint8_t kDoF = 6;

/**
 * @brief 角度单位。
 */
enum class AngleUnit : uint8_t
{
    Radians = 0,
    Degrees
};

/**
 * @brief 长度单位。
 */
enum class LengthUnit : uint8_t
{
    Meters = 0,
    Millimeters
};

/**
 * @brief 3 维向量。
 */
struct Vec3
{
    float v[3];
};

/**
 * @brief 6 维向量（twist/space velocity）。
 */
struct Vec6
{
    float v[6];
};

/**
 * @brief 3x3 矩阵。
 */
struct Mat3
{
    float m[3][3];
};

/**
 * @brief 4x4 齐次变换矩阵。
 */
struct Mat4
{
    float m[4][4];
};

/**
 * @brief 6x6 矩阵（雅可比/伴随矩阵等）。
 */
struct Mat6
{
    float m[6][6];
};

/**
 * @brief 判断标量是否接近零。
 */
bool nearZero(float value);

/**
 * @brief 按指定角度单位转换到弧度。
 */
float angleToRad(float value, AngleUnit unit);

/**
 * @brief 将弧度值按指定单位输出。
 */
float radToAngle(float value_rad, AngleUnit unit);

/**
 * @brief 按指定长度单位转换到米。
 */
float lengthToMeter(float value, LengthUnit unit);

Vec3 makeVec3(float x, float y, float z);
Vec6 makeVec6(float v0, float v1, float v2, float v3, float v4, float v5);

Mat3 eye3();
Mat4 eye4();
Mat6 eye6();
Mat3 zeros3();
Mat4 zeros4();
Mat6 zeros6();

Vec3 vec3Add(const Vec3 &a, const Vec3 &b);
Vec3 vec3Sub(const Vec3 &a, const Vec3 &b);
Vec3 vec3Scale(const Vec3 &a, float s);
float vec3Norm(const Vec3 &a);

Vec6 vec6Add(const Vec6 &a, const Vec6 &b);
Vec6 vec6Sub(const Vec6 &a, const Vec6 &b);
Vec6 vec6Scale(const Vec6 &a, float s);
float vec6Norm(const Vec6 &a);
float vec6Norm3(const Vec6 &a);
float vec6NormLinear(const Vec6 &a);

Mat3 mat3Add(const Mat3 &a, const Mat3 &b);
Mat3 mat3Sub(const Mat3 &a, const Mat3 &b);
Mat3 mat3Scale(const Mat3 &a, float s);
Mat3 mat3Mul(const Mat3 &a, const Mat3 &b);
Mat3 mat3Transpose(const Mat3 &a);
Vec3 mat3MulVec3(const Mat3 &a, const Vec3 &x);
float mat3Trace(const Mat3 &a);

Mat4 mat4Mul(const Mat4 &a, const Mat4 &b);
Mat4 mat4Transpose(const Mat4 &a);

Mat6 mat6Add(const Mat6 &a, const Mat6 &b);
Mat6 mat6Scale(const Mat6 &a, float s);
Mat6 mat6Mul(const Mat6 &a, const Mat6 &b);
Mat6 mat6Transpose(const Mat6 &a);
Vec6 mat6MulVec6(const Mat6 &a, const Vec6 &x);

bool solveLinear6x6(const Mat6 &A, const Vec6 &b, Vec6 *x);

/**
 * @brief 齐次变换拆分为旋转矩阵和平移向量。
 */
void trans_to_Rp(const Mat4 &T, Mat3 *R, Vec3 *p);

/**
 * @brief 齐次变换求逆。
 */
Mat4 trans_inv(const Mat4 &T);

/**
 * @brief 向量到 so(3) 反对称矩阵（hat）。
 */
Mat3 vec_to_so3(const Vec3 &omega);

/**
 * @brief so(3) 反对称矩阵到向量（vee）。
 */
Vec3 so3_to_vec(const Mat3 &so3);

/**
 * @brief 6 维 twist 向量转 se(3) 矩阵。
 */
Mat4 vec_to_se3(const Vec6 &V);

/**
 * @brief se(3) 矩阵转 6 维 twist 向量。
 */
Vec6 se3_to_vec(const Mat4 &se3);

/**
 * @brief 提取轴角表示。
 */
void axis_angle(const Vec3 &vec, Vec3 *axis, float *angle);

/**
 * @brief SO(3) 指数映射。
 */
Mat3 matrix_exp3(const Mat3 &so3);

/**
 * @brief SE(3) 指数映射。
 */
Mat4 matrix_exp6(const Mat4 &se3);

/**
 * @brief SO(3) 对数映射。
 */
Mat3 matrix_log3(const Mat3 &R);

/**
 * @brief SE(3) 对数映射。
 */
Mat4 matrix_log6(const Mat4 &T);

/**
 * @brief 计算变换矩阵的伴随矩阵 Ad_T。
 */
Mat6 adjoint(const Mat4 &T);

/**
 * @brief PoE 前向运动学。
 */
Mat4 fkine(const Mat4 &M, const Mat6 &Slist, const float q[kDoF]);

/**
 * @brief 空间雅可比计算。
 */
Mat6 cal_Jacobian(const Mat6 &Slist, const float q[kDoF]);

/**
 * @brief 基于阻尼最小二乘（DLS）的数值逆运动学。
 * @param eomg 角速度误差阈值
 * @param ev 线速度误差阈值
 * @param lambda 阻尼因子
 * @param max_step 每次迭代最大关节步长
 */
bool ikine(const Mat6 &Slist,
           const Mat4 &M,
           const Mat4 &T,
           const float q0[kDoF],
           float eomg,
           float ev,
           float q_out[kDoF],
           float lambda,
           float max_step,
           int max_iterations);

/**
 * @brief MDH 变换矩阵构造。
 * @param sigma true 表示转动关节，false 表示移动关节
 */
Mat4 transformation_mdh(float alpha,
                        float a,
                        float d,
                        float theta,
                        bool sigma,
                        float q);

/**
 * @brief 由 MDH 参数构建 PoE 模型（默认全部为转动关节）。
 * @param alpha MDH alpha 数组（rad）
 * @param a MDH a 数组
 * @param d MDH d 数组
 * @param theta_offset MDH theta 偏置数组（rad）
 * @param M_out 输出零位齐次变换矩阵 M
 * @param Slist_out 输出空间旋量轴矩阵 Slist（列向量）
 */
bool buildModelFromMDH(const float alpha[kDoF],
                       const float a[kDoF],
                       const float d[kDoF],
                       const float theta_offset[kDoF],
                       Mat4 *M_out,
                       Mat6 *Slist_out);

/**
 * @brief 由 MDH 参数构建 PoE 模型（默认全部为转动关节，支持单位转换）。
 */
bool buildModelFromMDH(const float alpha[kDoF],
                       const float a[kDoF],
                       const float d[kDoF],
                       const float theta_offset[kDoF],
                       AngleUnit angle_unit,
                       LengthUnit length_unit,
                       Mat4 *M_out,
                       Mat6 *Slist_out);

/**
 * @brief 由 MDH 参数构建 PoE 模型（可指定转动/移动关节）。
 * @param sigma true=转动关节，false=移动关节
 */
bool buildModelFromMDH(const float alpha[kDoF],
                       const float a[kDoF],
                       const float d[kDoF],
                       const float theta_offset[kDoF],
                       const bool sigma[kDoF],
                       Mat4 *M_out,
                       Mat6 *Slist_out);

/**
 * @brief 由 MDH 参数构建 PoE 模型（可指定转动/移动关节，支持单位转换）。
 */
bool buildModelFromMDH(const float alpha[kDoF],
                       const float a[kDoF],
                       const float d[kDoF],
                       const float theta_offset[kDoF],
                       const bool sigma[kDoF],
                       AngleUnit angle_unit,
                       LengthUnit length_unit,
                       Mat4 *M_out,
                       Mat6 *Slist_out);

} // namespace Math3D

#endif
