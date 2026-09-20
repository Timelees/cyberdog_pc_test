/*! @file ContactEstimator.cpp
 *  @brief All State Estimation Algorithms
 *
 *  This file will contain all state estimation algorithms.
 *  ContactEstimator should compute:
 *  - contact
 */

#include "Controllers/ContactEstimator.h"
#include <iostream>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <algorithm>
using namespace std;

#define PUB_INFO

namespace {
// 以更直观的方式打印 Eigen 矩阵/向量（带尺寸、固定精度、对齐、可选截断）
template <typename Derived>
std::string formatEigen(const Eigen::MatrixBase<Derived>& m,
                        const std::string& name,
                        int precision = 4,
                        int width = 10,
                        int maxRows = 0,
                        int maxCols = 0) {
  std::ostringstream oss;
  const int rows = static_cast<int>(m.rows());
  const int cols = static_cast<int>(m.cols());

  const int r = (maxRows > 0) ? std::min(rows, maxRows) : rows;
  const int c = (maxCols > 0) ? std::min(cols, maxCols) : cols;

  oss << "[ContactEstimator] " << name << " (" << rows << "x" << cols << ")\n";
  oss << std::fixed << std::setprecision(precision);

  for (int i = 0; i < r; ++i) {
    for (int j = 0; j < c; ++j) {
      oss << std::setw(width) << m(i, j);
      if (j + 1 < c) oss << ' ';
    }
    if (c < cols) oss << "  ...";
    oss << "\n";
  }
  if (r < rows) oss << "...\n";
  return oss.str();
}

inline void printSection(std::ostream& os, const std::string& title) {
  os << "\n==================== " << title << " ====================\n";
}

template <typename T>
inline void printContactKFDebug(const T* p0, const T* p1, const T* p2, const T* p3,
                                const T* v1, const T* v2, const T* v3,
                                const T* kf, size_t legNum) {
  std::cout << "[ContactEstimator] leg | KF |   p0(Xest) | Pcontact | Pposition |    Pforce | gaitPhase |   Pz(m) |    Fz\n";
  std::cout << "[ContactEstimator] ----+----+-----------+----------+-----------+----------+----------+--------+--------\n";
  std::cout << std::fixed << std::setprecision(4);
  for (size_t i = 0; i < legNum; ++i) {
    std::cout << "[ContactEstimator] "
              << std::setw(3) << i << " | "
              << std::setw(2) << static_cast<int>(kf[i]) << " | "
              << std::setw(9) << p0[i] << " | "
              << std::setw(8) << p1[i] << " | "
              << std::setw(9) << p2[i] << " | "
              << std::setw(8) << p3[i] << " | "
              << std::setw(8) << v1[i] << " | "
              << std::setw(6) << v2[i] << " | "
              << std::setw(6) << v3[i] << "\n";
  }
}

// 适配 Vec4 / Eigen 向量类型（调用底层 .data()）
template <typename DerivedP0, typename DerivedP1, typename DerivedP2, typename DerivedP3,
          typename DerivedV1, typename DerivedV2, typename DerivedV3, typename DerivedKF>
inline void printContactKFDebug(const Eigen::MatrixBase<DerivedP0>& p0,
                                const Eigen::MatrixBase<DerivedP1>& p1,
                                const Eigen::MatrixBase<DerivedP2>& p2,
                                const Eigen::MatrixBase<DerivedP3>& p3,
                                const Eigen::MatrixBase<DerivedV1>& v1,
                                const Eigen::MatrixBase<DerivedV2>& v2,
                                const Eigen::MatrixBase<DerivedV3>& v3,
                                const Eigen::MatrixBase<DerivedKF>& kf) {
  using Scalar = typename DerivedP0::Scalar;
  static_assert(std::is_same_v<Scalar, typename DerivedP1::Scalar> &&
                    std::is_same_v<Scalar, typename DerivedP2::Scalar> &&
                    std::is_same_v<Scalar, typename DerivedP3::Scalar> &&
                    std::is_same_v<Scalar, typename DerivedV1::Scalar> &&
                    std::is_same_v<Scalar, typename DerivedV2::Scalar> &&
                    std::is_same_v<Scalar, typename DerivedV3::Scalar> &&
                    std::is_same_v<Scalar, typename DerivedKF::Scalar>,
                "printContactKFDebug: input scalar types must match");

  const size_t n = static_cast<size_t>(p0.size());
  printContactKFDebug<Scalar>(p0.derived().data(), p1.derived().data(), p2.derived().data(), p3.derived().data(),
                              v1.derived().data(), v2.derived().data(), v3.derived().data(), kf.derived().data(),
                              n);
}
}  // namespace

template <typename T>
void ContactEstimator<T>::setup()
{
  /* 卡尔曼参数初始化 */
  //状态方程
  _A.setZero();
  _B.setIdentity(LEGNUM, LEGNUM);
  _H.block(0, 0, LEGNUM, LEGNUM) = Eigen::Matrix<T, LEGNUM, LEGNUM>::Identity();
  _H.block(LEGNUM, 0, LEGNUM, LEGNUM) = Eigen::Matrix<T, LEGNUM, LEGNUM>::Identity();
  _I.setIdentity(LEGNUM, LEGNUM);
  //过程噪声计算，协方差值 Wk  降低过程噪声协方差会减小最优估计的协方差值
  _Q.setZero();
  _Q.block(0, 0, LEGNUM, LEGNUM) = T(0.998) * Eigen::Matrix<T, LEGNUM, LEGNUM>::Identity();
  //测量噪声计算，协方差值 Vk
  _R.setZero();
  _R.block(0, 0, LEGNUM, LEGNUM) = T(0.841) * Eigen::Matrix<T, LEGNUM, LEGNUM>::Identity();
  _R.block(LEGNUM, LEGNUM, LEGNUM, LEGNUM) = T(0.930) * Eigen::Matrix<T, LEGNUM, LEGNUM>::Identity();
  _Xest.setZero();
  _Pest.setIdentity();
  _I.setIdentity();
  _muc0 = T(0); _sigmac0 = T(0.01);
  //支撑相后期 不准，随时进入摆动相
  _muc1 = T(1); _sigmac1 = T(0.01);
  _mus0 = T(0); _sigmas0 = T(0.01);
  _mus1 = T(1); _sigmas1 = T(0.01);
  //摆动相后期 不准，随时进入支撑相
  _muhz = T(0.00); 
  _sigmahz = T(0.025);

  _mufz[0] = T(0); 
  _mufz[1] = T(0);
  _mufz[2] = T(0);
  _mufz[3] = T(0);

  _sigmafz = T(0.1);

  /* 动力学模型参数初始化 */
  _model = this->_stateEstimatorData.quadruped->buildModel();
  _state.q = DVec<T>::Zero(ACTJOINTNUM);
  _state.qd = DVec<T>::Zero(ACTJOINTNUM);

  /* 扰动力公式参数初始化 */
  _S.setZero();
  _ST.setZero();
  _ST.block(DIMCONFIG - ACTJOINTNUM, 0, ACTJOINTNUM, ACTJOINTNUM) = Eigen::Matrix<T, ACTJOINTNUM, ACTJOINTNUM>::Identity();
  _S = _ST.transpose();
  _rd.setZero();
  _deltT = this->_stateEstimatorData.parameters->controller_dt;
  //截至频率
  _lambda = T(200);
  _gamma = exp( - _lambda * _deltT);
  _r.setZero();
  _Pk.setZero();
  _Xn.setZero();
  _Yn_1.setZero();
  _Yn.setZero();
  for(long i(0); i < LEGNUM; i++)
  {
    int index = DIMFLOAT + i * JOINTNUM;
    _legparameters[i].Sl.block(0, index, JOINTNUM, JOINTNUM) = Eigen::Matrix<T, JOINTNUM, JOINTNUM>::Identity();
    _legparameters[i].Jc.setZero();
    _legparameters[i].fi.setZero();
  }

  //显示配置信息
  {
    cout <<"ContactEstimator Init Complete! \n"<< endl;
    cout <<"***************************************************\n"<< endl;
  }
}

template <typename T>
void ContactEstimator<T>::run_with_kalmanfilter()
{
  float P_contact_temp[4][4];

  Vec4<T> contactphase = *this->_stateEstimatorData.contactPhase;
  Vec4<T> swingphase = *this->_stateEstimatorData.swingPhase;

  _Q.block(0, 0, LEGNUM, LEGNUM) = T(0.998) * Eigen::Matrix<T, LEGNUM, LEGNUM>::Identity();
  _R.block(0, 0, LEGNUM, LEGNUM) = T(0.841) * Eigen::Matrix<T, LEGNUM, LEGNUM>::Identity();
  _R.block(LEGNUM, LEGNUM, LEGNUM, LEGNUM) = T(0.930) * Eigen::Matrix<T, LEGNUM, LEGNUM>::Identity();

  /* 计算扰动力 */
  _UpdateModel(*this->_stateEstimatorData.result, this->_stateEstimatorData.legControllerData);
  _ComputeForce();

  /* 计算输入值、观测值概率 */
  for (int i = 0; i < LEGNUM; i++)
  {
    Vec3<T> pfoot = this->_stateEstimatorData.result->position +
            this->_stateEstimatorData.result->rBody.transpose() *
            (this->_stateEstimatorData.quadruped->getHipLocation(i) +
            this->_stateEstimatorData.legControllerData[i].p);

    Vec3<T> footforceActual = - _legparameters[i].fi;

    _Fz[i] = -footforceActual[2];
    _Pz[i] = pfoot[2];
    // std::cout << "==============leg " << i << " =========="<< std::endl;
    // std::cout << "the " << i << " th leg foot position is = " << pfoot.transpose() << std::endl;
    // std::cout << "the " << i << " th leg foot force is = " << footforceActual.transpose() << std::endl;
    
    
    if(swingphase(i) == T(0))
    {
      _gaitphase[i] = contactphase[i];
      _Scontact[i] = T(1);
      _Sswing[i] = T(0);
    }
    else
    {
      _gaitphase[i] = swingphase[i];
      _Scontact[i] = T(0);
      _Sswing[i] = T(1);
    }
    
    P_contact_temp[i][0] = erf((_gaitphase[i] - _muc0) / (_sigmac0 * T(sqrt(2))));
    P_contact_temp[i][1] = erf((_muc1 - _gaitphase[i] ) / (_sigmac1 * T(sqrt(2))));
    P_contact_temp[i][2] = erf((_mus0 - _gaitphase[i]) / (_sigmas0 * T(sqrt(2))));
    P_contact_temp[i][3] = erf((_gaitphase[i] - _mus1) / (_sigmas1 * T(sqrt(2))));

    _Pcontact[i] = (_Scontact[i] * (P_contact_temp[i][0] + P_contact_temp[i][1]) + _Sswing[i] * (T(2) + P_contact_temp[i][2] + P_contact_temp[i][3])) * T(0.5);
    // std::cout << "用作接触估计的步态相位：" << _gaitphase[i] << ", 接触相：" << _Scontact[i] << std::endl;
    // std::cout << "计算得到的相位接触概率：" << _Pcontact[i] << std::endl;
    
    _Pposition[i] = (T(1) + erf(( _muhz - _Pz[i]) / (_sigmahz * T(sqrt(2)))))* T(0.5); //
    // std::cout << "用作接触估计的足端高度：" << _Pz[i] << std::endl;
    // std::cout << "计算得到的高度接触概率：" << _Pposition[i] << std::endl;

    _Pforce[i] = 0.5*(T(1) + erf((_Fz[i] - _mufz[i]) / (_sigmafz * T(sqrt(2))))) * T(0.5)+0.5*_Pforce[i];//
    // std::cout << "用作接触估计的足端垂向力：" << _Fz[i] << std::endl;
    // std::cout << "计算得到的力接触概率：" << _Pforce[i] << std::endl;

    if(isnan(_Pcontact[i])) _Pcontact[i] = T(0);
    if(isnan(_Pposition[i])) _Pposition[i] = T(0);
    if(isnan(_Pforce[i])) _Pforce[i] = T(0);
  }

  _U << _Pcontact[0], _Pcontact[1], _Pcontact[2], _Pcontact[3];
  _Zmeas << _Pposition[0], _Pposition[1], _Pposition[2], _Pposition[3],
          _Pforce[0], _Pforce[1], _Pforce[2], _Pforce[3];

  // std::cout << "输入向量 U (contact probabilities): " << _U.transpose() << std::endl;
  // std::cout << "观测向量 Z (position & force probabilities): " << _Zmeas.transpose() << std::endl;
  
  /* 卡尔曼滤波 */
  _Xpre = _A * _Xest + _B * _U;
  _Ppre = _A * _Pest * _A.transpose() + _Q;

  // method 1
  Eigen::Matrix<T, LEGNUM * 2, 1> zModel = _H * _Xpre;
  Eigen::Matrix<T, LEGNUM * 2, 1> ey = _Zmeas - zModel;
  Eigen::Matrix<T, LEGNUM * 2, LEGNUM * 2> S = _H * _Ppre * _H.transpose() + _R;
  // todo compute LU only once
  Eigen::Matrix<T, LEGNUM * 2, 1> S_ey = S.lu().template solve(ey);
  _Xest = _Xpre + _Ppre * _H.transpose() * S_ey;
  // todo compute LU only once
  Eigen::Matrix<T, LEGNUM * 2, LEGNUM> S_C = S.lu().template solve(_H);
  _Pest = (_I - _Ppre * _H.transpose() * S_C) * _Ppre;

  // std::cout << "卡尔曼滤波后估计的接触状态概率 (Xest): " << _Xest.transpose() << std::endl;
  for(int i = 0; i < LEGNUM; i++)
  {
    this->_stateEstimatorData.result->contactEstimate_p0[i] = _Xest[i];
    this->_stateEstimatorData.result->contactEstimate_p1[i] = _Pcontact[i];
    this->_stateEstimatorData.result->contactEstimate_p2[i] = _Pposition[i];
    this->_stateEstimatorData.result->contactEstimate_p3[i] = _Pforce[i];
    this->_stateEstimatorData.result->contactEstimate_v1[i] = _gaitphase[i];
    this->_stateEstimatorData.result->contactEstimate_v2[i] = _Pz[i];
    this->_stateEstimatorData.result->contactEstimate_v3[i] = _Fz[i];
    if (_Xest[i] > _Pc)
    {
      this->_stateEstimatorData.result->contactEstimate_KF[i] = T(1);

    }
    else
    {
      this->_stateEstimatorData.result->contactEstimate_KF[i] = T(0);
    }

    //std::cout<< "the " << i << " th leg contact is = " << this->_stateEstimatorData.result->contactEstimate_KF[i] << std::endl;
  }

// #ifdef PUB_INFO
//   // 限频打印（避免每个控制周期刷屏）
//   static int _dbg_print_cnt = 0;
//   constexpr int kPrintEveryN = 100;  // 可按需调整
//   if ((_dbg_print_cnt++ % kPrintEveryN) == 0) {
//     printSection(std::cout, "Contact KF Estimates");
//     printContactKFDebug(this->_stateEstimatorData.result->contactEstimate_p0,
//                         this->_stateEstimatorData.result->contactEstimate_p1,
//                         this->_stateEstimatorData.result->contactEstimate_p2,
//                         this->_stateEstimatorData.result->contactEstimate_p3,
//                         this->_stateEstimatorData.result->contactEstimate_v1,
//                         this->_stateEstimatorData.result->contactEstimate_v2,
//                         this->_stateEstimatorData.result->contactEstimate_v3,
//                         this->_stateEstimatorData.result->contactEstimate_KF);
//   }
// #endif
}

// 计算动力学矩阵
template<typename T>
void ContactEstimator<T>::_UpdateModel(const StateEstimate<T> & state_est,
                                       const LegControllerData<T> * leg_data){
  _state.bodyOrientation = state_est.orientation;
  _state.bodyPosition = state_est.position;
  for(long i(0); i < JOINTNUM; ++i){
    _state.bodyVelocity[i] = state_est.omegaBody[i];
    _state.bodyVelocity[i + JOINTNUM] = state_est.vBody[i];

    for(long leg(0); leg < LEGNUM; ++leg){
      _state.q[JOINTNUM * leg + i] = leg_data[leg].q[i];
      _state.qd[JOINTNUM * leg + i] = leg_data[leg].qd[i];
      // todo need change
      _r[JOINTNUM * leg + i] = leg_data[leg].tauActuatual[i];
    }
  }
  _model.setState(_state);
  _model.contactJacobians();
  _model.massMatrix();
  _model.generalizedGravityForce();
  _model.generalizedCoriolisForce();

  _dynamicsparameters.M = _model.getMassMatrix();
  _dynamicsparameters.grav = _model.getGravityForce();
  _dynamicsparameters.coriolis = _model.getCoriolisForce();
  _dynamicsparameters.Minv = _dynamicsparameters.M.inverse();
  _dynamicsparameters.C = _model._C;

  // 统一格式打印（可按需限制 maxRows/maxCols，避免刷屏）
  // printSection(std::cout, "Dynamics Parameters");
  // std::cout << formatEigen(_dynamicsparameters.M, "M (mass matrix)", /*precision*/4, /*width*/10, /*maxRows*/0, /*maxCols*/0);
  // std::cout << formatEigen(_dynamicsparameters.grav, "grav (generalized gravity)", 4, 10);
  // std::cout << formatEigen(_dynamicsparameters.coriolis, "coriolis (generalized coriolis)", 4, 10);
  // std::cout << formatEigen(_dynamicsparameters.C, "C (C matrix)", 4, 10);
}

// 计算扰动力
template<typename T>
void ContactEstimator<T>::_ComputeForce()
{
  T beta = (T(1) - _gamma) / (_gamma * _deltT);
  Eigen::Matrix<T, DIMCONFIG, 1> qdot;
  qdot.setZero();
  //qdot.block(0, 0, JOINTNUM, 1) = this->_stateEstimatorData.result->vBody;
  //qdot.block(JOINTNUM, 0, JOINTNUM, 1) = this->_stateEstimatorData.result->omegaBody;
  qdot.block(0, 0, JOINTNUM, 1) = this->_stateEstimatorData.result->omegaBody;
  qdot.block(JOINTNUM, 0, JOINTNUM, 1) = this->_stateEstimatorData.result->vBody;
  qdot.block(DIMCONFIG - ACTJOINTNUM, 0, ACTJOINTNUM, 1) = _state.qd;
  Eigen::Matrix<T, DIMCONFIG, 1> r;
  r.setZero();
  r.block(DIMCONFIG - ACTJOINTNUM, 0, ACTJOINTNUM, 1) = _r;
  _Pk = _dynamicsparameters.M * qdot;
  _Xn = beta * _Pk + r + _dynamicsparameters.C.transpose() * qdot - _dynamicsparameters.grav;
  //_Xn = beta * _Pk + r - _dynamicsparameters.grav;
  _Yn_1 =  _Yn;
  _Yn = (T(1) - _gamma) * _Xn + _gamma * _Yn_1;
  _rd = beta * _Pk - _Yn;
  _legparameters[0].Jc = _model._Jc[linkID::FR];
  _legparameters[1].Jc = _model._Jc[linkID::FL];
  _legparameters[2].Jc = _model._Jc[linkID::HR];
  _legparameters[3].Jc = _model._Jc[linkID::HL];

#ifdef PUB_INFO
  // 限频打印 Jc，避免刷屏
  // static int _dbg_print_jc_cnt = 0;
  // constexpr int kPrintJcEveryN = 200;  // 可按需调整
  // if ((_dbg_print_jc_cnt++ % kPrintJcEveryN) == 0) {
  //   printSection(std::cout, "Contact Jacobians Jc (per leg)");
  //   std::cout << formatEigen(_legparameters[0].Jc, "Jc[FR]", 4, 10);
  //   std::cout << formatEigen(_legparameters[1].Jc, "Jc[FL]", 4, 10);
  //   std::cout << formatEigen(_legparameters[2].Jc, "Jc[HR]", 4, 10);
  //   std::cout << formatEigen(_legparameters[3].Jc, "Jc[HL]", 4, 10);
  // }
#endif

  for(long i(0); i < LEGNUM; i++)
  {
    auto temp = _legparameters[i].Sl * _legparameters[i].Jc.transpose();
    _legparameters[i].fi = temp.inverse() * _legparameters[i].Sl * _rd;
    // std::cout<< "the " << i << " th leg force is = " << _legparameters[i].fi[2]  << std::endl;
  }
}

template class ContactEstimator<double>;
template class ContactEstimator<float>;

