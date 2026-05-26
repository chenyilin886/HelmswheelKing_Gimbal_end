#ifndef LADRC_IMPROVED_HPP
#define LADRC_IMPROVED_HPP

#include <cmath>
#include <algorithm>

namespace ALG::LADRC
{

class TD
{
public:
    TD(float r = 200.0f, float h = 0.001f) : r_(r), h_(h), x1_(0), x2_(0) {}

    void calc(float u)
    {
        float fh = -r_ * r_ * (x1_ - u) - 2.0f * r_ * x2_;
        x1_ += x2_ * h_;
        x2_ += fh * h_;
    }

    float getX1() const { return x1_; }
    float getX2() const { return x2_; }

    void reset()
    {
        x1_ = 0;
        x2_ = 0;
    }

    void setState(float x1, float x2)
    {
        x1_ = x1;
        x2_ = x2;
    }

    void setR(float r) { r_ = r; }

private:
    float r_;
    float h_;
    float x1_;
    float x2_;
};

class LADRC
{
public:
    LADRC() : td_(), kp_(0), kd_(0), wc_(0), b0_(1.0f), h_(0.001f), max_(0),
          z1_(0), z2_(0), u_(0), target_(0) {}
    LADRC(float r, float kp, float kd, float wc, float b0, float h, float max)
        : td_(r, h), kp_(kp), kd_(kd), wc_(wc), b0_(b0), h_(h), max_(max),
          z1_(0), z2_(0), u_(0), target_(0) {}

    void setTarget(float target)
    {
        target_ = target;
    }

    float update(float feedback)
    {
        td_.calc(target_);

        esoUpdate(feedback);

        float e1 = td_.getX1() - z1_;
        float de1 = td_.getX2();
        float u0 = kp_ * e1 + kd_ * de1;

        u_ = (u0 - z2_) / b0_;
        u_ = std::clamp(u_, -max_, max_);

        return u_;
    }

    float getU() const { return u_; }
    float getZ1() const { return z1_; }
    float getZ2() const { return z2_; }
    float getTD_X1() const { return td_.getX1(); }
    float getTD_X2() const { return td_.getX2(); }

    void reset()
    {
        u_ = 0;
        z1_ = 0;
        z2_ = 0;
        td_.reset();
    }

    void setParams(float kp, float kd, float wc, float b0)
    {
        kp_ = kp;
        kd_ = kd;
        wc_ = wc;
        b0_ = b0;
    }

    void setMax(float max) { max_ = max; }

private:
    void esoUpdate(float feedback)
    {
        float beta1 = 2.0f * wc_;
        float beta2 = wc_ * wc_;
        float e = feedback - z1_;

        z1_ += h_ * (z2_ + beta1 * e + b0_ * u_);
        z2_ += h_ * beta2 * e;

        z2_ *= 0.99f;
    }

    TD td_;
    float kp_;
    float kd_;
    float wc_;
    float b0_;
    float h_;
    float max_;
    float z1_;
    float z2_;
    float u_;
    float target_;
};

}

#endif
