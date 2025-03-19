#include "KINEMATICS.h"

MatrixXd getTransformI0(void)
{
    MatrixXd htmI0 = MatrixXd::Identity(4, 4);
    return htmI0;
}

MatrixXd jointToTransform01(VectorXd q)
{
    double q1 = q(0);
    double s1 = sin(q1);
    double c1 = cos(q1);

    MatrixXd htm01(4, 4);
    htm01 << c1, -s1, 0, 0,
             s1, c1, 0, 0,
             0, 0, 1, 0.016, // z축을 기준으로 회전, z축 방향으로 0.016 이동
             0, 0, 0, 1;

    return htm01;
}

MatrixXd jointToTransform12(VectorXd q)
{
    double q2 = q(1);
    double s2 = sin(q2);
    double c2 = cos(q2);

    MatrixXd htm12(4, 4);
    htm12 << c2, 0, s2, 0,
             0, 1, 0, 0, // y축을 기준으로 회전
            -s2, 0, c2, 0.06, // z축 방향으로 0.06 이동
             0, 0, 0, 1;

    return htm12;
}

MatrixXd jointToTransform23(VectorXd q)
{
    double q3 = q(2);
    double s3 = sin(q3);
    double c3 = cos(q3);

    MatrixXd htm23(4, 4);
    htm23 << 1, 0, 0, 0, // x축을 기준으로 회전
             0, c3, -s3, 0,
             0, s3, c3, 0.35, // z축 방향으로 0.35 이동
             0, 0, 0, 1;

    return htm23;
}

MatrixXd jointToTransform34(VectorXd q)
{
    double q4 = q(3);
    double s4 = sin(q4);
    double c4 = cos(q4);

    MatrixXd htm34(4, 4);
    htm34 << c4, -s4, 0, -0.083472,  // x축으로 -0.083472 이동
             s4, c4, 0, 0,
             0, 0, 1, 0.275647, // z축을 기준으로 회전, z축 방향으로 0.275647 이동
             0, 0, 0, 1;

    return htm34;
}

MatrixXd jointToTransform45(VectorXd q)
{
    double q5 = q(4);
    double s5 = sin(q5);
    double c5 = cos(q5);

    MatrixXd htm45(4, 4);
    htm45 << 1, 0, 0, 0, // x축을 기준으로 회전
             0, c5, -s5, 0,
             0, s5, c5, 0.036, // z축 방향으로 0.036 이동
             0, 0, 0, 1;

    return htm45;
}

MatrixXd jointToTransform56(VectorXd q)
{
    double q6 = q(5);
    double s6 = sin(q6);
    double c6 = cos(q6);

    MatrixXd htm56(4, 4);
    htm56 << c6, -s6, 0, 0,
             s6, c6, 0, 0,
             0, 0, 1, 0.079, // z축을 기준으로 회전, z축 방향으로 0.079 이동
             0, 0, 0, 1;

    return htm56;
}

MatrixXd getTransform6E(void)
{ // End-effector까지의 변환 행렬
    MatrixXd htm6E(4, 4);
    htm6E << 1, 0, 0, 0,
             0, 1, 0, 0,
             0, 0, 1, 0.05, // z축 방향으로 0.05 이동
             0, 0, 0, 1;

    return htm6E;
}


VectorXd jointToPosition(VectorXd q) // Forward Kinematics
{
    MatrixXd TI0(4, 4), T01(4, 4), T12(4, 4), T23(4, 4), T34(4, 4), T45(4, 4), T56(4, 4), T6E(4, 4), TIE(4, 4);
    TI0 = getTransformI0();
    T01 = jointToTransform01(q);
    T12 = jointToTransform12(q);
    T23 = jointToTransform23(q);
    T34 = jointToTransform34(q);
    T45 = jointToTransform45(q);
    T56 = jointToTransform56(q);
    T6E = getTransform6E();

    TIE = TI0 * T01 * T12 * T23 * T34 * T45 * T56 * T6E;

    Vector3d position = TIE.block(0, 3, 3, 1);

    return position;
}

MatrixXd jointToRotMat(VectorXd q)
{
    MatrixXd TI0(4, 4), T01(4, 4), T12(4, 4), T23(4, 4), T34(4, 4), T45(4, 4), T56(4, 4), T6E(4, 4), TIE(4, 4);

    TI0 = getTransformI0();
    T01 = jointToTransform01(q);
    T12 = jointToTransform12(q);
    T23 = jointToTransform23(q);
    T34 = jointToTransform34(q);
    T45 = jointToTransform45(q);
    T56 = jointToTransform56(q);
    T6E = getTransform6E();

    TIE = TI0 * T01 * T12 * T23 * T34 * T45 * T56 * T6E;

    MatrixXd rot_m(3, 3);
    rot_m = TIE.block(0, 0, 3, 3);

    return rot_m; // return C_IE or C_des or C_init
}

VectorXd rotMatToRotVec(MatrixXd C)
{
    // Input: a rotation matrix C
    // Output: the rotational vector which describes the rotation C (Roe Angle)
    VectorXd phi(3), n(3);
    double th;

    th = acos((C(0, 0) + C(1, 1) + C(2, 2) - 1) / 2); // Roe Angle
    if (fabs(th) < 0.001)
    {
        n << 0, 0, 0;
    }
    else
    {
        n << (C(2, 1) - C(1, 2)), // Equivalant Angle-Axis
            (C(0, 2) - C(2, 0)),
            (C(1, 0) - C(0, 1));
        n = (1 / (2 * sin(th))) * n; // Length == 1 (2.82)
    }

    phi = th * n; // can be decomposited by it's amplitude

    return phi;
}

MatrixXd jointToPosJac(VectorXd q)
{
    MatrixXd J_P = MatrixXd::Zero(3, 6);
    MatrixXd T_I0(4, 4), T_01(4, 4), T_12(4, 4), T_23(4, 4), T_34(4, 4), T_45(4, 4), T_56(4, 4), T_6E(4, 4);
    MatrixXd T_I1(4, 4), T_I2(4, 4), T_I3(4, 4), T_I4(4, 4), T_I5(4, 4), T_I6(4, 4);
    MatrixXd R_I1(3, 3), R_I2(3, 3), R_I3(3, 3), R_I4(3, 3), R_I5(3, 3), R_I6(3, 3);
    Vector3d r_I_I1, r_I_I2, r_I_I3, r_I_I4, r_I_I5, r_I_I6;
    Vector3d n_1, n_2, n_3, n_4, n_5, n_6;
    Vector3d n_I_1, n_I_2, n_I_3, n_I_4, n_I_5, n_I_6;
    Vector3d r_I_IE;

    T_I0 = getTransformI0();
    T_01 = jointToTransform01(q);
    T_12 = jointToTransform12(q);
    T_23 = jointToTransform23(q);
    T_34 = jointToTransform34(q);
    T_45 = jointToTransform45(q);
    T_56 = jointToTransform56(q);
    T_6E = getTransform6E();

    T_I1 = T_I0 * T_01;
    T_I2 = T_I0 * T_01 * T_12;
    T_I3 = T_I0 * T_01 * T_12 * T_23;
    T_I4 = T_I0 * T_01 * T_12 * T_23 * T_34;
    T_I5 = T_I0 * T_01 * T_12 * T_23 * T_34 * T_45;
    T_I6 = T_I0 * T_01 * T_12 * T_23 * T_34 * T_45 * T_56;

    R_I1 = T_I1.block(0, 0, 3, 3);
    R_I2 = T_I2.block(0, 0, 3, 3);
    R_I3 = T_I3.block(0, 0, 3, 3);
    R_I4 = T_I4.block(0, 0, 3, 3);
    R_I5 = T_I5.block(0, 0, 3, 3);
    R_I6 = T_I6.block(0, 0, 3, 3);

    r_I_I1 = T_I1.block(0, 3, 3, 1);
    r_I_I2 = T_I2.block(0, 3, 3, 1);
    r_I_I3 = T_I3.block(0, 3, 3, 1);
    r_I_I4 = T_I4.block(0, 3, 3, 1);
    r_I_I5 = T_I5.block(0, 3, 3, 1);
    r_I_I6 = T_I6.block(0, 3, 3, 1);

    n_1 << 0, 0, 1;
    n_2 << 1, 0, 0;
    n_3 << 1, 0, 0;
    n_4 << 0, 1, 0;
    n_5 << 1, 0, 0;
    n_6 << 0, 1, 0;

    n_I_1 = R_I1 * n_1;
    n_I_2 = R_I2 * n_2;
    n_I_3 = R_I3 * n_3;
    n_I_4 = R_I4 * n_4;
    n_I_5 = R_I5 * n_5;
    n_I_6 = R_I6 * n_6;

    r_I_IE = (T_I6 * T_6E).block(0, 3, 3, 1);

    J_P.col(0) << n_I_1.cross(r_I_IE - r_I_I1);
    J_P.col(1) << n_I_2.cross(r_I_IE - r_I_I2);
    J_P.col(2) << n_I_3.cross(r_I_IE - r_I_I3);
    J_P.col(3) << n_I_4.cross(r_I_IE - r_I_I4);
    J_P.col(4) << n_I_5.cross(r_I_IE - r_I_I5);
    J_P.col(5) << n_I_6.cross(r_I_IE - r_I_I6);

    return J_P;
}

MatrixXd jointToRotJac(VectorXd q)
{
    MatrixXd J_R(3, 6);
    MatrixXd T_I0(4, 4), T_01(4, 4), T_12(4, 4), T_23(4, 4), T_34(4, 4), T_45(4, 4), T_56(4, 4), T_6E(4, 4);
    MatrixXd T_I1(4, 4), T_I2(4, 4), T_I3(4, 4), T_I4(4, 4), T_I5(4, 4), T_I6(4, 4);
    MatrixXd R_I1(3, 3), R_I2(3, 3), R_I3(3, 3), R_I4(3, 3), R_I5(3, 3), R_I6(3, 3);
    Vector3d n_1, n_2, n_3, n_4, n_5, n_6;

    T_I0 = getTransformI0();
    T_01 = jointToTransform01(q);
    T_12 = jointToTransform12(q);
    T_23 = jointToTransform23(q);
    T_34 = jointToTransform34(q);
    T_45 = jointToTransform45(q);
    T_56 = jointToTransform56(q);
    T_6E = getTransform6E();

    T_I1 = T_I0 * T_01;
    T_I2 = T_I0 * T_01 * T_12;
    T_I3 = T_I0 * T_01 * T_12 * T_23;
    T_I4 = T_I0 * T_01 * T_12 * T_23 * T_34;
    T_I5 = T_I0 * T_01 * T_12 * T_23 * T_34 * T_45;
    T_I6 = T_I0 * T_01 * T_12 * T_23 * T_34 * T_45 * T_56;

    R_I1 = T_I1.block(0, 0, 3, 3);
    R_I2 = T_I2.block(0, 0, 3, 3);
    R_I3 = T_I3.block(0, 0, 3, 3);
    R_I4 = T_I4.block(0, 0, 3, 3);
    R_I5 = T_I5.block(0, 0, 3, 3);
    R_I6 = T_I6.block(0, 0, 3, 3);

    n_1 << 0, 0, 1;
    n_2 << 1, 0, 0;
    n_3 << 1, 0, 0;
    n_4 << 0, 1, 0;
    n_5 << 1, 0, 0;
    n_6 << 0, 1, 0;

    J_R.col(0) << R_I1 * n_1;
    J_R.col(1) << R_I2 * n_2;
    J_R.col(2) << R_I3 * n_3;
    J_R.col(3) << R_I4 * n_4;
    J_R.col(4) << R_I5 * n_5;
    J_R.col(5) << R_I6 * n_6;

    return J_R;
}

VectorXd inverseKinematics(Vector3d r_des, MatrixXd C_des, VectorXd q0, double tol)
{
    double num_it = 0;
    MatrixXd J_P(3, 6), J_R(3, 6), J(6, 6), pinvJ(6, 6), C_err(3, 3), C_IE(3, 3);
    VectorXd q(6), dq(6), dXe(6);
    Vector3d dr, dph;
    double lambda;

    double max_it = 100; // 200;
    q = q0;
    C_IE = jointToRotMat(q); // q0로 구한 end-effector의 orientation
    C_err = C_des * C_IE.transpose();
    lambda = 0.001;
    dr = r_des - jointToPosition(q);
    dph = rotMatToRotVec(C_err);
    dXe << dr(0), dr(1), dr(2), dph(0), dph(1), dph(2);

    while (num_it < max_it && dXe.norm() > tol)
    {
        J_P = jointToPosJac(q);
        J_R = jointToRotJac(q);

        J.block(0, 0, 3, 6) = J_P;
        J.block(3, 0, 3, 6) = J_R; // Geometric Jacobian

        dq = pseudoInverseMat(J, lambda) * dXe;

        q += 0.5 * dq; // scaling factor k = 0.5

        C_IE = jointToRotMat(q);
        C_err = C_des * C_IE.transpose(); // orientation err

        dr = r_des - jointToPosition(q); // position err
        dph = rotMatToRotVec(C_err);
        dXe << dr(0), dr(1), dr(2), dph(0), dph(1), dph(2);

        num_it++;
        for (int i = 0; i < q.size(); i++)
        {
            if (q(i) < -M_PI / 2 || q(i) > M_PI / 2) // -90도에서 90도로 제한
            {
                q(i) = std::clamp(q(i), -M_PI / 2, M_PI / 2); // 각도를 범위 내로 강제로 설정
            }
            /*if (q(i) < -90.0 || q(i) > 90.0)
            {
                q(i) = 0.0;
            }*/
        }
    }
    return q;
}

MatrixXd pseudoInverseMat(const MatrixXd &A, double /*lambda*/)
{
    Eigen::CompleteOrthogonalDecomposition<MatrixXd> cod(A);
    return cod.pseudoInverse();
}
