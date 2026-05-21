#ifndef GIMBAL_AXIS_HPP
#define GIMBAL_AXIS_HPP

#pragma once

#include "ladrc_improved.hpp"
#include "pid.hpp"
#include "Feedforward.hpp"
#include <cmath>
#include <cstdint>
#include <algorithm>

enum class GimbalMode { ANGLE, VELOCITY, VISION };

struct AxisTuneParams
{
    float vel_kp, vel_kd, vel_wc, vel_b0, vel_max;
    float ang_kp, ang_kd, ang_wc, ang_b0, ang_max;
    float pid_kp, pid_ki, pid_kd, pid_max, pid_ilimit, pid_isep;
    float td_r;
    float gravity_k, gravity_phi;
    float vel_limit;
    uint8_t runaway_thresh;
    float torque_limit;
    float torque_sign;
    float angle_max_deg;
    float angle_min_deg;
};

class GimbalAxis
{
public:
    struct Config
    {
        AxisTuneParams tune;
        float vel_r = 200.0f, vel_h = 0.001f;
        float ang_r = 200.0f, ang_h = 0.001f;
        float td_h = 0.001f;
        bool use_td_filter = false;
        bool use_gravity_ff = false;
    };

    GimbalAxis() = default;

    void init(const Config &cfg)
    {
        cfg_ = cfg;
        vel_adrc_ = ALG::LADRC::LADRC(cfg_.vel_r, cfg_.tune.vel_kp, cfg_.tune.vel_kd,
                                       cfg_.tune.vel_wc, cfg_.tune.vel_b0, cfg_.vel_h, cfg_.tune.vel_max);
        angle_adrc_ = ALG::LADRC::LADRC(cfg_.ang_r, cfg_.tune.ang_kp, cfg_.tune.ang_kd,
                                         cfg_.tune.ang_wc, cfg_.tune.ang_b0, cfg_.ang_h, cfg_.tune.ang_max);
        angle_pid_ = ALG::PID::PID(cfg_.tune.pid_kp, cfg_.tune.pid_ki, cfg_.tune.pid_kd,
                                    cfg_.tune.pid_max, cfg_.tune.pid_ilimit, cfg_.tune.pid_isep);
        tar_vel_td_ = ALG::LADRC::TD(cfg_.tune.td_r, cfg_.td_h);
        if (cfg_.use_gravity_ff)
        {
            gravity_ff_ = Alg::Feedforward::Gravity(cfg_.tune.gravity_k, cfg_.tune.gravity_phi);
        }
    }

    float computeAnglePID(float angle_err)
    {
        return angle_pid_.UpDate(angle_err, 0.0f);
    }

    float filterVelTarget(float raw_vel)
    {
        if (cfg_.use_td_filter)
        {
            tar_vel_td_.calc(raw_vel);
            return tar_vel_td_.getX1();
        }
        return raw_vel;
    }

    float applyAngleSafetyLimit(float vel_target, float angle_deg) const
    {
        if (angle_deg >= cfg_.tune.angle_max_deg && vel_target > 0.0f)
            return 0.0f;
        if (angle_deg <= cfg_.tune.angle_min_deg && vel_target < 0.0f)
            return 0.0f;
        return vel_target;
    }

    float computeTorque(GimbalMode mode, float vel_target, float vel_feedback, float angle_deg = 0.0f)
    {
        if (runaway_flag_)
        {
            vel_adrc_.reset();
            angle_adrc_.reset();
            return 0.0f;
        }

        float torque = 0.0f;
        if (mode == GimbalMode::ANGLE || mode == GimbalMode::VISION)
        {
            angle_adrc_.setTarget(vel_target);
            torque = angle_adrc_.update(vel_feedback);
        }
        else
        {
            vel_adrc_.setTarget(vel_target);
            torque = vel_adrc_.update(vel_feedback);
        }

        if (cfg_.use_gravity_ff)
        {
            gravity_ff_.GravityFeedforward(angle_deg);
            torque += gravity_ff_.getFeedforward();
        }

        torque *= cfg_.tune.torque_sign;
        torque = std::clamp(torque, -cfg_.tune.torque_limit, cfg_.tune.torque_limit);
        return torque;
    }

    bool checkRunaway(float vel_feedback)
    {
        if (std::fabs(vel_feedback) > cfg_.tune.vel_limit)
        {
            runaway_counter_++;
            if (runaway_counter_ >= cfg_.tune.runaway_thresh)
                runaway_flag_ = true;
        }
        else
        {
            if (runaway_counter_ > 0)
                runaway_counter_--;
            if (std::fabs(vel_feedback) < cfg_.tune.vel_limit * 0.5f)
                runaway_flag_ = false;
        }
        return runaway_flag_;
    }

    void resetControllers()
    {
        vel_adrc_.reset();
        angle_adrc_.reset();
        angle_pid_.reset();
        tar_vel_td_.reset();
    }

    bool isRunaway() const { return runaway_flag_; }
    float getVelFiltered(GimbalMode mode) const
    {
        return (mode == GimbalMode::ANGLE || mode == GimbalMode::VISION)
                   ? angle_adrc_.getTD_X1() : vel_adrc_.getTD_X1();
    }
    float getGravityFF() { return gravity_ff_.getFeedforward(); }
    ALG::PID::PID &anglePid() { return angle_pid_; }
    ALG::LADRC::LADRC &velAdrc() { return vel_adrc_; }
    ALG::LADRC::LADRC &angleAdrc() { return angle_adrc_; }

    void applyTuneParams(const AxisTuneParams &p)
    {
        cfg_.tune = p;
        vel_adrc_.setParams(p.vel_kp, p.vel_kd, p.vel_wc, p.vel_b0);
        vel_adrc_.setMax(p.vel_max);
        angle_adrc_.setParams(p.ang_kp, p.ang_kd, p.ang_wc, p.ang_b0);
        angle_adrc_.setMax(p.ang_max);
        angle_pid_.setK(p.pid_kp, p.pid_ki, p.pid_kd);
        angle_pid_.setMax(p.pid_max);
        angle_pid_.setIntegralLimit(p.pid_ilimit);
        angle_pid_.setIntegralSeparation(p.pid_isep);
        tar_vel_td_.setR(p.td_r);
        gravity_ff_.setK(p.gravity_k);
        gravity_ff_.setPhi(p.gravity_phi);
    }

private:
    ALG::LADRC::LADRC vel_adrc_;
    ALG::LADRC::LADRC angle_adrc_;
    ALG::PID::PID angle_pid_;
    ALG::LADRC::TD tar_vel_td_;
    Alg::Feedforward::Gravity gravity_ff_;

    uint8_t runaway_counter_ = 0;
    bool runaway_flag_ = false;
    Config cfg_{};
};

#endif
