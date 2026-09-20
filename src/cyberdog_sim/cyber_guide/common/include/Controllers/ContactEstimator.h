/*! @file ContactEstimator.h
 *  @brief All Contact Estimation Algorithms
 *
 *  This file will contain all contact detection algorithms. For now, it just
 * has a pass-through algorithm which passes the phase estimation to the state
 * estimator.  This will need to change once we move contact detection to C++
 *
 *  We also still need to establish conventions for "phase" and "contact".
 */

#ifndef PROJECT_CONTACTESTIMATOR_H
#define PROJECT_CONTACTESTIMATOR_H

#include "Controllers/StateEstimatorContainer.h"
#include "../Dynamics/FloatingBaseModel.h"
#include "../Dynamics/Quadruped.h"
/*!
 * A "passthrough" contact estimator which returns the expected contact state
 */

#define LEGNUM  4
#define ACTJOINTNUM 12
#define DIMCONFIG 18
#define JOINTNUM 3
#define DIMFLOAT 6

// 存储动力学参数
template <typename T>
struct DynamicsParameters {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    DMat<T> M;
    DMat<T> Minv;
    DMat<T> C;
    DVec<T> grav;
    DVec<T> coriolis;
};

// 存储腿部参数
template <typename T>
struct LegParameters {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Matrix<T, JOINTNUM, DIMCONFIG> Sl;
    Eigen::Matrix<T, JOINTNUM, DIMCONFIG> Jc;
    Eigen::Matrix<T, JOINTNUM, 1> fi;
};

template <typename T>
class ContactEstimator : public GenericEstimator<T>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    /*!
     * Set the estimated contact by copying the exptected contact state into the
     * estimated contact state
     */
    virtual void run()
    {
      this->_stateEstimatorData.result->contactEstimate = *this->_stateEstimatorData.contactPhase;
      this->_stateEstimatorData.result->swingEstimate = *this->_stateEstimatorData.swingPhase;
      // 只在MPC下运行
      if(this->_stateEstimatorData.parameters->control_mode == 4)
      {
        run_with_kalmanfilter();
      }

    }

    /*!
     * Set up the contact estimator
     */
    virtual void setup();

private:
    void run_with_kalmanfilter();
    void _UpdateModel(const StateEstimate<T> & state_est, const LegControllerData<T> * leg_data);
    void _ComputeForce();
    /* 卡尔曼滤波参数 */
    Eigen::Matrix<T, LEGNUM, 1> _Xest;
    Eigen::Matrix<T, LEGNUM, 1> _Xpre;
    Eigen::Matrix<T, LEGNUM, 1> _U;
    Eigen::Matrix<T, LEGNUM * 2, 1> _Zmeas;
    Eigen::Matrix<T, LEGNUM, LEGNUM> _Pest;
    Eigen::Matrix<T, LEGNUM, LEGNUM> _Ppre;
    Eigen::Matrix<T, LEGNUM, LEGNUM> _A;
    Eigen::Matrix<T, LEGNUM, LEGNUM> _B;
    Eigen::Matrix<T, LEGNUM * 2, LEGNUM> _H;
    Eigen::Matrix<T, LEGNUM, LEGNUM> _Q;
    Eigen::Matrix<T, LEGNUM * 2, LEGNUM * 2> _R;
    Eigen::Matrix<T, LEGNUM, LEGNUM * 2> _K;
    Eigen::Matrix<T, LEGNUM, LEGNUM> _I;

    Vec4<T> _Scontact, _Sswing;
    Vec4<T> _gaitphase;
    Vec4<T> _Fz;

    Vec4<T> _Pz;
    float _Pc = 0.5;

    /* 高斯变量参数 */
    float _muc0, _muc1, _mus0, _mus1, _muhz;
    float _sigmac0, _sigmac1, _sigmas0, _sigmas1, _sigmahz, _sigmafz;
    Vec4<T> _mufz;
    Vec4<T> _Pcontact;
    Vec4<T> _Pposition;
    Vec4<T> _Pforce;

    /* 动力学参数 */
    FloatingBaseModel<T> _model;
    DynamicsParameters<T> _dynamicsparameters;
    FBModelState<T> _state;

    /* 扰动力计算参数 */
    Eigen::Matrix<T, ACTJOINTNUM, DIMCONFIG> _S;
    Eigen::Matrix<T, DIMCONFIG, ACTJOINTNUM> _ST;
    Eigen::Matrix<T, DIMCONFIG, 1> _rd;
    Eigen::Matrix<T, ACTJOINTNUM, 1> _r;
    Eigen::Matrix<T, DIMCONFIG, 1> _Pk;
    Eigen::Matrix<T, DIMCONFIG, 1> _Xn;
    Eigen::Matrix<T, DIMCONFIG, 1> _Yn_1;
    Eigen::Matrix<T, DIMCONFIG, 1> _Yn;
    T _gamma;
    T _lambda;
    T _deltT;
    LegParameters<T> _legparameters[4];
};
#endif  // PROJECT_CONTACTESTIMATOR_H