#ifndef CJOINT
#define CJOINT

#include <iostream>
#include <Eigen/Dense>
#include <pinocchio/spatial/se3.hpp>
#include <cmath>
#include <memory>
#include <string>
#include <stdexcept>

typedef Eigen::Matrix4d Mat4;


class CJoint {
protected:
    double q_, qMin_, qMax_;

public:
    CJoint(double min, double max, double q_init = 0.0)
        : q_(q_init), qMin_(min), qMax_(max) {}

    virtual ~CJoint() {}

    virtual Mat4        getTransform() const = 0;
    virtual std::string getTypeName()  const = 0;
    virtual std::unique_ptr<CJoint> clone() const = 0;

    void setQ(double q) {
        if (q < qMin_ || q > qMax_)
            throw std::out_of_range("q hors de l'intervalle [qMin_, qMax_]");
        q_ = q;
    }

    double getQ()    const { return q_;    }
    double getQmin() const { return qMin_; }
    double getQmax() const { return qMax_; }
};


// ── Revolute ─────────────────────────────────────────────────────────────────
// Rotation autour de Z + translation dx selon X
// Signature : (min, max, q_init, dx)
class CJointRevolute : public CJoint {
private:
    double dx_;

public:
    CJointRevolute(double min, double max, double q_init, double dx)
        : CJoint(min, max, q_init), dx_(dx) {}

    Mat4 getTransform() const override {
        Mat4 M = Mat4::Identity();
        M(0, 0) =  std::cos(q_);
        M(1, 0) =  std::sin(q_);
        M(0, 1) = -std::sin(q_);
        M(1, 1) =  std::cos(q_);
        M(0, 3) =  dx_;
        return M;
    }

    void   setdx(double d) { dx_ = d; }
    double getdx()   const { return dx_; }

    std::string getTypeName() const override { return "CJointRevolute"; }

    std::unique_ptr<CJoint> clone() const override {
        return std::make_unique<CJointRevolute>(*this);
    }
};


// ── Prismatic ────────────────────────────────────────────────────────────────
// Translation selon Z
// Signature : (min, max, q_init)
class CJointPrismatic : public CJoint {
public:
    CJointPrismatic(double min, double max, double q_init = 0.0)
        : CJoint(min, max, q_init) {}

    Mat4 getTransform() const override {
        Mat4 M = Mat4::Identity();
        M(2, 3) = q_;
        return M;
    }

    std::string getTypeName() const override { return "CJointPrismatic"; }

    std::unique_ptr<CJoint> clone() const override {
        return std::make_unique<CJointPrismatic>(*this);
    }
};

#endif 