#ifndef GIMBAL_TARGET_HPP
#define GIMBAL_TARGET_HPP

#pragma once

#include "DT7.hpp"
#include "vision.hpp"
#include "gimbal_axis.hpp"
#include "ladrc_improved.hpp"
#include <cmath>
#include <algorithm>

class GimbalTarget
{
public:
    struct Config
    {
        float yaw_vel_scale = 4.0f;//Yaw角速度的缩放
        float pitch_vel_scale = 1.0f;//Pitch角速度的缩放
        float pitch_angle_scale = 0.3f;//Pitch角度的缩放
        float pitch_max_deg = 20.0f;
        float pitch_min_deg = -20.0f;
        float vision_yaw_limit = 20.0f;
        float vision_pitch_limit = 15.0f;
        float joystick_deadzone = 0.03f;//摇杆死区
        float yaw_td_r = 200.0f;//Yaw目标角度TD跟踪因子
        float pitch_td_r = 200.0f;//Pitch目标角度TD跟踪因子
        float td_h = 0.001f;//TD采样周期
    };

    struct Output
    {
        float yaw_vel_target = 0.0f;
        float yaw_angle_err = 0.0f;
        float pitch_vel_target = 0.0f;
        float pitch_angle_err = 0.0f;
    };

    GimbalTarget() = default;

    void init(const Config &cfg)
    {
        cfg_ = cfg;
        yaw_target_td_ = ALG::LADRC::TD(cfg_.yaw_td_r, cfg_.td_h);
        pitch_target_td_ = ALG::LADRC::TD(cfg_.pitch_td_r, cfg_.td_h);
    }

    void applyScaleParams(float yaw_vel_scale, float pitch_vel_scale, float pitch_angle_scale)
    {
        cfg_.yaw_vel_scale = yaw_vel_scale;
        cfg_.pitch_vel_scale = pitch_vel_scale;
        cfg_.pitch_angle_scale = pitch_angle_scale;
    }

    void applyTdParams(float yaw_td_r, float pitch_td_r)
    {
        cfg_.yaw_td_r = yaw_td_r;
        cfg_.pitch_td_r = pitch_td_r;
        yaw_target_td_.setR(yaw_td_r);
        pitch_target_td_.setR(pitch_td_r);
    }

    Output update(GimbalMode mode,
                  float cur_yaw_rad, float cur_pitch_deg,
                  GimbalAxis &yaw_axis, GimbalAxis &pitch_axis,
                  const BSP::REMOTE_CONTROL::RemoteController &rc,
                  const Comm::Vision &vision)
    {
        Output out;
        static constexpr float DEG_TO_RAD = 0.0174532925f;

        if (mode == GimbalMode::VISION)
        {
            float yaw_offset = vision.getTargetYaw();// Yaw 偏移
            float pitch_offset_v = vision.getTargetPitch();// Pitch 偏移
            // 限制偏移范围
            yaw_offset = std::clamp(yaw_offset, -cfg_.vision_yaw_limit, cfg_.vision_yaw_limit);
            pitch_offset_v = std::clamp(pitch_offset_v, -cfg_.vision_pitch_limit, cfg_.vision_pitch_limit);

            //target_yaw_rad_ += yaw_offset * DEG_TO_RAD;
            //target_pitch_deg_ += pitch_offset_v;
            //target_pitch_deg_ = std::clamp(target_pitch_deg_, cfg_.pitch_min_deg, cfg_.pitch_max_deg);

            //out.yaw_angle_err = normalizeAngle(target_yaw_rad_ - cur_yaw_rad);
            //out.yaw_vel_target = yaw_axis.computeAnglePID(out.yaw_angle_err);

            //out.pitch_angle_err = (target_pitch_deg_ - cur_pitch_deg) * DEG_TO_RAD;
            //out.pitch_vel_target = pitch_axis.computeAnglePID(out.pitch_angle_err);
        }
        else if (mode == GimbalMode::ANGLE)
        {
            float raw_yaw_input = rc.get_right_x();
            if (std::fabs(raw_yaw_input) > cfg_.joystick_deadzone)
            {
                target_yaw_rad_ -= raw_yaw_input * cfg_.yaw_vel_scale * 0.001f;
                target_yaw_rad_ = normalizeAngle(target_yaw_rad_);
            }
            yaw_target_td_.calc(target_yaw_rad_);
            float smoothed_yaw_target = yaw_target_td_.getX1();
            out.yaw_angle_err = normalizeAngle(smoothed_yaw_target - cur_yaw_rad);
            out.yaw_vel_target = yaw_axis.computeAnglePID(out.yaw_angle_err);

            float raw_pitch_input = rc.get_right_y();
            if (std::fabs(raw_pitch_input) > cfg_.joystick_deadzone)
            {
                target_pitch_deg_ += raw_pitch_input * cfg_.pitch_angle_scale;
                target_pitch_deg_ = std::clamp(target_pitch_deg_, cfg_.pitch_min_deg, cfg_.pitch_max_deg);
            }
            pitch_target_td_.calc(target_pitch_deg_);
            float smoothed_pitch_target = pitch_target_td_.getX1();
            out.pitch_angle_err = (smoothed_pitch_target - cur_pitch_deg) * DEG_TO_RAD;
            out.pitch_vel_target = pitch_axis.computeAnglePID(out.pitch_angle_err);
        }
        else
        {
            out.yaw_vel_target = rc.get_right_x() * cfg_.yaw_vel_scale;

            float raw_pitch_vel = rc.get_right_y() * cfg_.pitch_vel_scale;
            out.pitch_vel_target = pitch_axis.filterVelTarget(raw_pitch_vel);
        }

        return out;
    }

    void resetTargets(float yaw_rad, float pitch_deg)
    {
        target_yaw_rad_ = yaw_rad;
        target_pitch_deg_ = std::clamp(pitch_deg, cfg_.pitch_min_deg, cfg_.pitch_max_deg);
        yaw_target_td_.setState(target_yaw_rad_, 0.0f);
        pitch_target_td_.setState(target_pitch_deg_, 0.0f);
    }

    float getTargetYawRad() const { return target_yaw_rad_; }
    float getTargetPitchDeg() const { return target_pitch_deg_; }

private:
    static float normalizeAngle(float angle)
    {
        static constexpr float PI_F = 3.14159265f;
        while (angle > PI_F) angle -= 2.0f * PI_F;
        while (angle < -PI_F) angle += 2.0f * PI_F;
        return angle;
    }

    Config cfg_{};
    ALG::LADRC::TD yaw_target_td_;
    ALG::LADRC::TD pitch_target_td_;
    float target_yaw_rad_ = 0.0f;
    float target_pitch_deg_ = 0.0f;
};

#endif
