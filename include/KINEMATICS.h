#ifndef KINEMATICS_H
#define KINEMATICS_H

#include <Eigen/Dense>

using namespace Eigen;

MatrixXd getTransformI0(void);
MatrixXd jointToTransform01(VectorXd q);
MatrixXd jointToTransform12(VectorXd q);
MatrixXd jointToTransform23(VectorXd q);
MatrixXd jointToTransform34(VectorXd q);
MatrixXd jointToTransform45(VectorXd q);
MatrixXd jointToTransform56(VectorXd q);
MatrixXd getTransform6E(void);
VectorXd jointToPosition(VectorXd q);
MatrixXd jointToPosJac(VectorXd q);
MatrixXd jointToRotJac(VectorXd q);
MatrixXd jointToRotMat(VectorXd q);
VectorXd rotMatToRotVec(MatrixXd rotMat);
MatrixXd pseudoInverseMat(const MatrixXd& A, double lambda);
VectorXd inverseKinematics(Vector3d r_des, MatrixXd C_des, VectorXd q0, double tol);

#endif // KINEMATICS_H

