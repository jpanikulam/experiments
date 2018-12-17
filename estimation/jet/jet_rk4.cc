#include "estimation/jet/jet_rk4.hh"

#include "eigen.hh"

namespace estimation {
namespace jet {
VecNd<18> to_vector(const StateDelta &in_grp) {
  const VecNd<18> out =
      (VecNd<18>() << ((in_grp.eps_dot_error)[0]), ((in_grp.eps_dot_error)[1]),
       ((in_grp.eps_dot_error)[2]), ((in_grp.eps_dot_error)[3]),
       ((in_grp.eps_dot_error)[4]), ((in_grp.eps_dot_error)[5]),
       ((in_grp.eps_ddot_error)[0]), ((in_grp.eps_ddot_error)[1]),
       ((in_grp.eps_ddot_error)[2]), ((in_grp.eps_ddot_error)[3]),
       ((in_grp.eps_ddot_error)[4]), ((in_grp.eps_ddot_error)[5]),
       ((in_grp.T_world_from_body_error_log)[0]),
       ((in_grp.T_world_from_body_error_log)[1]),
       ((in_grp.T_world_from_body_error_log)[2]),
       ((in_grp.T_world_from_body_error_log)[3]),
       ((in_grp.T_world_from_body_error_log)[4]),
       ((in_grp.T_world_from_body_error_log)[5]))
          .finished();
  return out;
}
State operator-(const State &a, const State &b) {
  const State difference =
      State{((a.eps_dot) - (b.eps_dot)), ((a.eps_ddot) - (b.eps_ddot)),
            ((a.T_world_from_body) * ((b.T_world_from_body).inverse()))};
  return difference;
}
VecNd<18> compute_delta(const State &a, const State &b) {
  const State difference = a - b;
  const VecNd<6> eps_dot_error = difference.eps_dot;
  const VecNd<6> eps_ddot_error = difference.eps_ddot;
  const SE3 T_world_from_body_error = difference.T_world_from_body;
  const VecNd<6> T_world_from_body_error_log =
      SE3::log(T_world_from_body_error);
  const StateDelta delta =
      StateDelta{eps_dot_error, eps_ddot_error, T_world_from_body_error_log};
  const VecNd<18> out_vec = to_vector(delta);
  return out_vec;
}
StateDot compute_qdot(const State &Q, const Parameters &Z) {
  const VecNd<6> eps_dddot = Z.eps_dddot;
  const VecNd<6> eps_dot = Q.eps_dot;
  const VecNd<6> eps_ddot = Q.eps_ddot;
  const StateDot Qdot = StateDot{eps_ddot, eps_dddot, eps_dot};
  return Qdot;
}
StateDot operator*(const double half_h, const StateDot &K1) {
  const StateDot anon_01bfb4 =
      StateDot{(half_h * (K1.eps_ddot)), (half_h * (K1.eps_dddot)),
               (half_h * (K1.eps_dot))};
  return anon_01bfb4;
}
State operator+(const State &Q, const StateDot &anon_01bfb4) {
  const State Q2 =
      State{((Q.eps_dot) + (anon_01bfb4.eps_ddot)),
            ((Q.eps_ddot) + (anon_01bfb4.eps_dddot)),
            ((SE3::exp((anon_01bfb4.eps_dot))) * (Q.T_world_from_body))};
  return Q2;
}
StateDot operator+(const StateDot &K1, const StateDot &K4) {
  const StateDot anon_ca70ad = StateDot{((K1.eps_ddot) + (K4.eps_ddot)),
                                        ((K1.eps_dddot) + (K4.eps_dddot)),
                                        ((K1.eps_dot) + (K4.eps_dot))};
  return anon_ca70ad;
}
State rk4_integrate(const State &Q, const Parameters &Z, const double h) {
  const double half = 0.5;
  const double half_h = h * half;
  const StateDot K1 = compute_qdot(Q, Z);
  const State Q2 = Q + (half_h * K1);
  const StateDot K2 = compute_qdot(Q2, Z);
  const State Q3 = Q + (half_h * K2);
  const StateDot K3 = compute_qdot(Q3, Z);
  const State Q4 = Q + (h * K3);
  const StateDot K4 = compute_qdot(Q4, Z);
  const double two = 2.0;
  const double sixth = 0.166666666667;
  const State Qn = Q + (sixth * ((K1 + K4) + (two * (K2 + K3))));
  return Qn;
}
State operator+(const State &a, const StateDelta &grp_b) {
  const State out = State{((a.eps_dot) + (grp_b.eps_dot_error)),
                          ((a.eps_ddot) + (grp_b.eps_ddot_error)),
                          ((SE3::exp((grp_b.T_world_from_body_error_log))) *
                           (a.T_world_from_body))};
  return out;
}
StateDelta from_vector(const VecNd<18> &in_vec) {
  const VecNd<6> anon_b67cd1 =
      (VecNd<6>() << (in_vec[0]), (in_vec[1]), (in_vec[2]), (in_vec[3]),
       (in_vec[4]), (in_vec[5]))
          .finished();
  const VecNd<6> anon_c6df6a =
      (VecNd<6>() << (in_vec[6]), (in_vec[7]), (in_vec[8]), (in_vec[9]),
       (in_vec[10]), (in_vec[11]))
          .finished();
  const VecNd<6> anon_7c9f34 =
      (VecNd<6>() << (in_vec[12]), (in_vec[13]), (in_vec[14]), (in_vec[15]),
       (in_vec[16]), (in_vec[17]))
          .finished();
  const StateDelta out = StateDelta{anon_b67cd1, anon_c6df6a, anon_7c9f34};
  return out;
}
State apply_delta(const State &a, const VecNd<18> &delta) {
  const StateDelta grp_b = from_vector(delta);
  const State out = a + grp_b;
  return out;
}
} // namespace jet
} // namespace estimation