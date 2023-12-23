#include <nav_msgs/Odometry.h>
#include <trajectory_msgs//PolyTraj.h>
#include <optimizer/poly_traj_utils.hpp>
#include <quadrotor_msgs/PositionCommand.h>
#include <std_msgs/Empty.h>
#include <visualization_msgs/Marker.h>
#include <ros/ros.h>
#include <acados_export_diffdrive/acados_solver_diffdrive.h>
#include <acados_export_diffdrive/acados_sim_solver_diffdrive.h>
#include <acados_c/ocp_nlp_interface.h>

using namespace Eigen;

ros::Publisher pos_cmd_pub;

quadrotor_msgs::PositionCommand cmd;
// double pos_gain[3] = {0, 0, 0};
// double vel_gain[3] = {0, 0, 0};

#define FLIP_YAW_AT_END 0
#define TURN_YAW_TO_CENTER_AT_END 0

bool receive_traj_ = false;
boost::shared_ptr<poly_traj::Trajectory> traj_;
double traj_duration_;
ros::Time start_time_;
int traj_id_;
ros::Time heartbeat_time_(0);
Eigen::Vector3d last_pos_;

// yaw control
double last_yaw_, last_yawdot_, slowly_flip_yaw_target_, slowly_turn_to_center_target_;
double time_forward_;

int drone_type_;
double cmd_vel_thresh_;
double init_x_, init_y_, init_z_;
int dd_use_mpc_; // only 1 for yes

struct ocp {
    diffdrive_solver_capsule *capsule;
    double time_horizon;
    ocp_nlp_config *cfg;
    ocp_nlp_dims *dims;
    ocp_nlp_solver *solver;
    ocp_nlp_in *in;
    ocp_nlp_out *out;
    double p[DIFFDRIVE_NP]; // reference for [px, py, vx, vx, theta]
} dd_ocp;

struct sim {
    diffdrive_sim_solver_capsule *capsule;
    sim_config *cfg;
    void *dims;
    sim_solver *solver;
    sim_in *in;
    sim_out *out;
    double x[DIFFDRIVE_NX]; // [px, py, vx, vy, theta, vb, omega]
    double u[DIFFDRIVE_NU]; // [vb_dot, omega_dot]
} dd_sim;

void heartbeatCallback(std_msgs::EmptyPtr msg) {
    heartbeat_time_ = ros::Time::now();
}

void polyTrajCallback(trajectory_msgs::PolyTrajPtr msg) {
    if (msg->order != 5) {
        ROS_ERROR("[traj_server] Only support trajectory order equals 5 now!");
        return;
    }
    if (msg->duration.size() * (msg->order + 1) != msg->coef_x.size()) {
        ROS_ERROR("[traj_server] WRONG trajectory parameters, ");
        return;
    }

    int piece_nums = msg->duration.size();
    std::vector<double> dura(piece_nums);
    std::vector<poly_traj::CoefficientMat> cMats(piece_nums);
    for (int i = 0; i < piece_nums; ++i) {
        int i6 = i * 6;
        cMats[i].row(0) << msg->coef_x[i6 + 0], msg->coef_x[i6 + 1], msg->coef_x[i6 + 2],
                msg->coef_x[i6 + 3], msg->coef_x[i6 + 4], msg->coef_x[i6 + 5];
        cMats[i].row(1) << msg->coef_y[i6 + 0], msg->coef_y[i6 + 1], msg->coef_y[i6 + 2],
                msg->coef_y[i6 + 3], msg->coef_y[i6 + 4], msg->coef_y[i6 + 5];
        cMats[i].row(2) << msg->coef_z[i6 + 0], msg->coef_z[i6 + 1], msg->coef_z[i6 + 2],
                msg->coef_z[i6 + 3], msg->coef_z[i6 + 4], msg->coef_z[i6 + 5];

        dura[i] = msg->duration[i];
    }

    traj_.reset(new poly_traj::Trajectory(dura, cMats));

    start_time_ = msg->start_time;
    traj_duration_ = traj_->getTotalDuration();
    traj_id_ = msg->traj_id;

    receive_traj_ = true;
}

std::pair<double, double> calculate_yaw(double t_cur, Eigen::Vector3d &pos, double dt) {
    constexpr double YAW_DOT_MAX_PER_SEC = 2 * M_PI;
    constexpr double YAW_DOT_DOT_MAX_PER_SEC = 5 * M_PI;
    std::pair<double, double> yaw_yawdot(0, 0);

    Eigen::Vector3d dir = t_cur + time_forward_ <= traj_duration_
                          ? traj_->getPos(t_cur + time_forward_) - pos
                          : traj_->getPos(traj_duration_) - pos;
    double yaw_temp = dir.norm() > 0.1
                      ? atan2(dir(1), dir(0))
                      : last_yaw_;

    double yawdot = 0;
    double d_yaw = yaw_temp - last_yaw_;
    if (d_yaw >= M_PI) {
        d_yaw -= 2 * M_PI;
    }
    if (d_yaw <= -M_PI) {
        d_yaw += 2 * M_PI;
    }

    const double YDM = d_yaw >= 0 ? YAW_DOT_MAX_PER_SEC : -YAW_DOT_MAX_PER_SEC;
    const double YDDM = d_yaw >= 0 ? YAW_DOT_DOT_MAX_PER_SEC : -YAW_DOT_DOT_MAX_PER_SEC;
    double d_yaw_max;
    if (fabs(last_yawdot_ + dt * YDDM) <= fabs(YDM)) {
        // yawdot = last_yawdot_ + dt * YDDM;
        d_yaw_max = last_yawdot_ * dt + 0.5 * YDDM * dt * dt;
    } else {
        // yawdot = YDM;
        double t1 = (YDM - last_yawdot_) / YDDM;
        d_yaw_max = ((dt - t1) + dt) * (YDM - last_yawdot_) / 2.0;
    }

    if (fabs(d_yaw) > fabs(d_yaw_max)) {
        d_yaw = d_yaw_max;
    }
    yawdot = d_yaw / dt;

    double yaw = last_yaw_ + d_yaw;
    if (yaw > M_PI)
        yaw -= 2 * M_PI;
    if (yaw < -M_PI)
        yaw += 2 * M_PI;
    yaw_yawdot.first = yaw;
    yaw_yawdot.second = yawdot;

    last_yaw_ = yaw_yawdot.first;
    last_yawdot_ = yaw_yawdot.second;

    yaw_yawdot.second = yaw_temp;

    return yaw_yawdot;
}

void publish_cmd(Vector3d p, Vector3d v, Vector3d a, Vector3d j, double y, double yd, double ydd = 0.0) {

    cmd.header.stamp = ros::Time::now();
    cmd.header.frame_id = "world";
    cmd.trajectory_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY;
    cmd.trajectory_id = traj_id_;

    cmd.position.x = p(0);
    cmd.position.y = p(1);
    cmd.position.z = p(2);
    cmd.velocity.x = v(0);
    cmd.velocity.y = v(1);
    cmd.velocity.z = v(2);
    cmd.acceleration.x = a(0);
    cmd.acceleration.y = a(1);
    cmd.acceleration.z = a(2);
    cmd.jerk.x = j(0);
    cmd.jerk.y = j(1);
    cmd.jerk.z = j(2);
    cmd.yaw = y;
    cmd.yaw_dot = yd;
    cmd.yaw_acc = ydd;

    pos_cmd_pub.publish(cmd);

    last_pos_ = p;
}

void cmdCallback(const ros::TimerEvent &e) {
    /* no publishing before receive traj_ and have heartbeat */
    if (heartbeat_time_.toSec() <= 1e-5) {
        // ROS_ERROR_ONCE("[traj_server] No heartbeat from the planner received");
        return;
    }
    if (!receive_traj_)
        return;

    ros::Time time_now = ros::Time::now();

    if ((time_now - heartbeat_time_).toSec() > 0.5) {
        ROS_ERROR("[traj_server] Lost heartbeat from the planner, is it dead?");

        receive_traj_ = false;
        publish_cmd(last_pos_, Vector3d::Zero(), Vector3d::Zero(), Vector3d::Zero(), last_yaw_, 0);
    }

    double t_cur = (time_now - start_time_).toSec();

    Eigen::Vector3d pos(Eigen::Vector3d::Zero()), vel(Eigen::Vector3d::Zero()), acc(Eigen::Vector3d::Zero()), jer(
            Eigen::Vector3d::Zero());
    std::pair<double, double> yaw_yawdot(0, 0);

    static ros::Time time_last = ros::Time::now();
#if FLIP_YAW_AT_END or TURN_YAW_TO_CENTER_AT_END
    static bool finished = false;
#endif
    if (t_cur < traj_duration_ && t_cur >= 0.0) {
        pos = traj_->getPos(t_cur);
        vel = traj_->getVel(t_cur);
        acc = traj_->getAcc(t_cur);
        jer = traj_->getJer(t_cur);

        // calculate yaw, yaw_dot, yaw_acc (or mpc)
        double yaw_acc = 0;
        if (drone_type_ != 2) // copter and omni-drive
            yaw_yawdot = calculate_yaw(t_cur, pos, (time_now - time_last).toSec());
        else { // diff-drive
            if (dd_use_mpc_ == 1) {
                // use mpc
                // set reference
                double t_sampling;
                for (int i = 0; i <= dd_ocp.dims->N; i++) {
                    t_sampling = t_cur + i * dd_ocp.time_horizon / dd_ocp.dims->N;
                    if (t_sampling > traj_duration_)
                        t_sampling = traj_duration_ - dd_ocp.time_horizon / dd_ocp.dims->N;
                    Eigen::Vector3d pos_ref = traj_->getPos(t_sampling);
                    Eigen::Vector3d vel_ref = traj_->getVel(t_sampling);
                    dd_ocp.p[0] = pos_ref(0);
                    dd_ocp.p[1] = pos_ref(1);
                    dd_ocp.p[2] = vel_ref(0);
                    dd_ocp.p[3] = vel_ref(1);
                    dd_ocp.p[4] = atan2(vel_ref(1), vel_ref(0));
                    diffdrive_acados_update_params(dd_ocp.capsule, i, dd_ocp.p, DIFFDRIVE_NP);
                }

                // set init states
                ocp_nlp_constraints_model_set(dd_ocp.cfg, dd_ocp.dims, dd_ocp.in, 0, "lbx", dd_sim.x);
                ocp_nlp_constraints_model_set(dd_ocp.cfg, dd_ocp.dims, dd_ocp.in, 0, "ubx", dd_sim.x);

                // solve
                int ocp_status = diffdrive_acados_solve(dd_ocp.capsule);
                if (ocp_status == ACADOS_SUCCESS) {
                    // print info
                    double elapsed_time;
                    ocp_nlp_get(dd_ocp.cfg, dd_ocp.solver, "time_tot", &elapsed_time);
                    ocp_nlp_out_get(dd_ocp.cfg, dd_ocp.dims, dd_ocp.out, 0, "u", dd_sim.u);

                    // update sim
                    sim_in_set(dd_sim.cfg, dd_sim.dims, dd_sim.in, "x", dd_sim.x);
                    sim_in_set(dd_sim.cfg, dd_sim.dims, dd_sim.in, "u", dd_sim.u);
                    int sim_status = diffdrive_acados_sim_solve(dd_sim.capsule);
                    if (sim_status == ACADOS_SUCCESS) {
                        sim_out_get(dd_sim.cfg, dd_sim.dims, dd_sim.out, "x", dd_sim.x);
                    } else {
                        ROS_ERROR("SOLVE SIM ERROR");
                    }
                } else {
                    ROS_ERROR("SOLVE OCP ERROR");
                }
            } else {
                // no mpc, yaw is set directly according to vel
                if (vel.norm() == 0) {
                    yaw_yawdot.first = last_yaw_;
                    yaw_yawdot.second = 0;
                    yaw_acc = 0;
                } else {
                    yaw_yawdot.first = atan2(vel(1), vel(0));
                    yaw_yawdot.second = (vel(0) * acc(1) - vel(1) * acc(0)) / vel.squaredNorm();
                    auto temp0 = vel(0) * jer(1) - vel(1) * jer(0);
                    auto temp1 = 2 * yaw_yawdot.second * (vel(0) * acc(1) + vel(1) * acc(0));
                    yaw_acc = (temp0 - temp1) / vel.squaredNorm();
                }
            }
        }

        time_last = time_now;
//        last_yaw_ = yaw_yawdot.first;
//        last_pos_ = pos;

        slowly_flip_yaw_target_ = yaw_yawdot.first + M_PI;
        if (slowly_flip_yaw_target_ > M_PI)
            slowly_flip_yaw_target_ -= 2 * M_PI;
        if (slowly_flip_yaw_target_ < -M_PI)
            slowly_flip_yaw_target_ += 2 * M_PI;
        constexpr double CENTER[2] = {0.0, 0.0};
        slowly_turn_to_center_target_ = atan2(CENTER[1] - pos(1), CENTER[0] - pos(0));

        // publish
        if (drone_type_ != 2) {
            publish_cmd(pos, vel, acc, jer, yaw_yawdot.first, yaw_yawdot.second, yaw_acc);
            last_yaw_ = yaw_yawdot.first;
            last_pos_ = pos;
        } else {
            if (dd_use_mpc_ == 1) {
                double v_dot = dd_sim.u[0], v = dd_sim.x[5], theta = dd_sim.x[4], omega = dd_sim.x[6];
                Eigen::Vector3d dd_sim_pos, dd_sim_vel, dd_sim_acc;
                dd_sim_pos << dd_sim.x[0], dd_sim.x[1], 0;
                dd_sim_vel << dd_sim.x[2], dd_sim.x[3], 0;
                dd_sim_acc << v_dot * cos(theta) - v * omega * sin(theta), v_dot * sin(theta) +
                                                                           v * omega * cos(theta), 0;
                publish_cmd(dd_sim_pos, dd_sim_vel, dd_sim_acc, jer, theta, omega, dd_sim.u[1]);
            } else if (vel.norm() > cmd_vel_thresh_) {
                publish_cmd(pos, vel, acc, jer, yaw_yawdot.first, yaw_yawdot.second, yaw_acc);
                last_yaw_ = yaw_yawdot.first;
                last_pos_ = pos;
            } else {
                publish_cmd(last_pos_, Vector3d::Zero(), Vector3d::Zero(), Vector3d::Zero(), last_yaw_, 0, 0);
            }
        }
#if FLIP_YAW_AT_END or TURN_YAW_TO_CENTER_AT_END
        finished = false;
#endif
    }

#if FLIP_YAW_AT_END
    else if (t_cur >= traj_duration_)
    {
      if (finished)
        return;

      /* hover when finished traj_ */
      pos = traj_->getPos(traj_duration_);
      vel.setZero();
      acc.setZero();
      jer.setZero();

      if (slowly_flip_yaw_target_ > 0)
      {
        last_yaw_ += (time_now - time_last).toSec() * M_PI / 2;
        yaw_yawdot.second = M_PI / 2;
        if (last_yaw_ >= slowly_flip_yaw_target_)
        {
          finished = true;
        }
      }
      else
      {
        last_yaw_ -= (time_now - time_last).toSec() * M_PI / 2;
        yaw_yawdot.second = -M_PI / 2;
        if (last_yaw_ <= slowly_flip_yaw_target_)
        {
          finished = true;
        }
      }

      yaw_yawdot.first = last_yaw_;
      time_last = time_now;

      publish_cmd(pos, vel, acc, jer, yaw_yawdot.first, yaw_yawdot.second);
    }
#endif

#if TURN_YAW_TO_CENTER_AT_END
    else if (t_cur >= traj_duration_)
    {
      if (finished)
        return;

      /* hover when finished traj_ */
      pos = traj_->getPos(traj_duration_);
      vel.setZero();
      acc.setZero();
      jer.setZero();

      double d_yaw = last_yaw_ - slowly_turn_to_center_target_;
      if (d_yaw >= M_PI)
      {
        last_yaw_ += (time_now - time_last).toSec() * M_PI / 2;
        yaw_yawdot.second = M_PI / 2;
        if (last_yaw_ > M_PI)
          last_yaw_ -= 2 * M_PI;
      }
      else if (d_yaw <= -M_PI)
      {
        last_yaw_ -= (time_now - time_last).toSec() * M_PI / 2;
        yaw_yawdot.second = -M_PI / 2;
        if (last_yaw_ < -M_PI)
          last_yaw_ += 2 * M_PI;
      }
      else if (d_yaw >= 0)
      {
        last_yaw_ -= (time_now - time_last).toSec() * M_PI / 2;
        yaw_yawdot.second = -M_PI / 2;
        if (last_yaw_ <= slowly_turn_to_center_target_)
          finished = true;
      }
      else
      {
        last_yaw_ += (time_now - time_last).toSec() * M_PI / 2;
        yaw_yawdot.second = M_PI / 2;
        if (last_yaw_ >= slowly_turn_to_center_target_)
          finished = true;
      }

      yaw_yawdot.first = last_yaw_;
      time_last = time_now;

      publish_cmd(pos, vel, acc, jer, yaw_yawdot.first, yaw_yawdot.second);
    }
#endif
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "traj_server");
    // ros::NodeHandle node;
    ros::NodeHandle nh("~");

    nh.param("traj_server/time_forward", time_forward_, -1.0);
    nh.param("traj_server/drone_type", drone_type_, 0);
    nh.param("traj_server/cmd_vel_thresh", cmd_vel_thresh_, 0.0);
    nh.param("traj_server/init_x", init_x_, 0.0);
    nh.param("traj_server/init_y", init_y_, 0.0);
    nh.param("traj_server/init_z", init_z_, 0.0);
    nh.param("traj_server/diffdrive_use_mpc", dd_use_mpc_, 1);


    // mpc for diffdrive type robots
    if (drone_type_ == 2 && dd_use_mpc_ == 1) {
        dd_ocp.capsule = diffdrive_acados_create_capsule();
        diffdrive_acados_create(dd_ocp.capsule);

        dd_ocp.time_horizon = 1.0;
        dd_ocp.cfg = diffdrive_acados_get_nlp_config(dd_ocp.capsule);
        dd_ocp.dims = diffdrive_acados_get_nlp_dims(dd_ocp.capsule);
        dd_ocp.solver = diffdrive_acados_get_nlp_solver(dd_ocp.capsule);
        dd_ocp.in = diffdrive_acados_get_nlp_in(dd_ocp.capsule);
        dd_ocp.out = diffdrive_acados_get_nlp_out(dd_ocp.capsule);

        dd_sim.capsule = diffdrive_acados_sim_solver_create_capsule();
        diffdrive_acados_sim_create(dd_sim.capsule);
        dd_sim.cfg = diffdrive_acados_get_sim_config(dd_sim.capsule);
        dd_sim.dims = diffdrive_acados_get_sim_dims(dd_sim.capsule);
        dd_sim.solver = diffdrive_acados_get_sim_solver(dd_sim.capsule);
        dd_sim.in = diffdrive_acados_get_sim_in(dd_sim.capsule);
        dd_sim.out = diffdrive_acados_get_sim_out(dd_sim.capsule);
        dd_sim.x[0] = init_x_;
        dd_sim.x[1] = init_y_;
        dd_sim.x[2] = dd_sim.x[3] = dd_sim.x[4] = dd_sim.x[5] = dd_sim.x[6] = 0.0;
        dd_sim.u[0] = dd_sim.u[1] = 0;
    }

    ros::Subscriber poly_traj_sub = nh.subscribe("planning/trajectory", 10, polyTrajCallback);
    ros::Subscriber heartbeat_sub = nh.subscribe("heartbeat", 10, heartbeatCallback);

    pos_cmd_pub = nh.advertise<quadrotor_msgs::PositionCommand>("/position_cmd", 50);
    Eigen::Vector3d init_pos(init_x_, init_y_, init_z_);
    publish_cmd(init_pos, Vector3d::Zero(), Vector3d::Zero(), Vector3d::Zero(), 0, 0, 0);

    ros::Timer cmd_timer = nh.createTimer(ros::Duration(0.01), cmdCallback);

    last_yaw_ = 0.0;
    last_yawdot_ = 0.0;

    ros::Duration(1.0).sleep();

    ROS_INFO("[Traj server]: ready.");

    ros::spin();

    return 0;
}