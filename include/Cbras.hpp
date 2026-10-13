#ifndef CBRAS
#define CBRAS

#include <vector>
#include <memory>
#include <stdexcept>
#include <iomanip>
#include <ostream>

#include "Cjoint.hpp"


class CBras {
private:
    std::vector<std::unique_ptr<CJoint>> joints_;

public:
    // ── Constructeurs / destructeur ──────────────────────────────────────────
    CBras() = default;

    // Copie profonde (chaque joint est cloné)
    CBras(const CBras& other) {
        for (const auto& j : other.joints_)
            joints_.push_back(j->clone());
    }

    /*CBras& operator=(const CBras& other) {
        if (this != &other) {
            joints_.clear();
            for (const auto& j : other.joints_)
                joints_.push_back(j->clone());
        }
        return *this;
    }*/

    CBras& operator=(CBras other) {   // copy and swap
        std::swap(joints_, other.joints_);
        return *this;
    }                                  

    CBras(CBras&&) = default;
    CBras& operator=(CBras&&) = default;
    ~CBras() = default;

    // ── Ajout d'un joint ────────────────────────────────────────────────────
    void addJoint(std::unique_ptr<CJoint> joint) {
        joints_.push_back(std::move(joint));
    }

    // ── Nombre de degrés de liberté ─────────────────────────────────────────
    std::size_t getNbJoints() const {
        return joints_.size();
    }

    // ── Accès à un joint (lève std::out_of_range si i >= N) ─────────────────
    /*const CJoint& getJoint(std::size_t i) const {
        if (i >= joints_.size())
            throw std::out_of_range(
                "getJoint: indice " + std::to_string(i) +
                " hors limites (N = " + std::to_string(joints_.size()) + ")");
        return *joints_[i];
    }*/

    CJoint& getJoint(std::size_t i) {
        if (i >= joints_.size())
            throw std::out_of_range(
                "getJoint: indice " + std::to_string(i) +
                " hors limites (N = " + std::to_string(joints_.size()) + ")");
        return *joints_[i];
    }

    // ── Cinématique directe : T0 * T1 * ... * T_{N-1} ───────────────────────
    Mat4 computeFK() const {
        Mat4 T = Mat4::Identity();
        for (const auto& j : joints_)
            T = T * j->getTransform();
        return T;
    }

    // ── Affichage ────────────────────────────────────────────────────────────
    friend std::ostream& operator<<(std::ostream& os, const CBras& bras) {
        os << "Bras [" << bras.getNbJoints() << " DDL]\n";

        for (std::size_t i = 0; i < bras.joints_.size(); ++i) {
            const CJoint& j = *bras.joints_[i];
            std::string type = j.getTypeName();

            std::string label, unit;
            if (type == "CJointRevolute") {
                label = "Revolute";
                unit  = "rad";
            } else {
                label = "Prismatic";
                unit  = "m";
            }

            os << std::fixed << std::setprecision(3);
            os << "[" << i << "] "
               << label
               << " q = " << j.getQ() << " " << unit
               << " bornes=["
               << std::setw(7) << j.getQmin() << ", "
               << std::setw(7) << j.getQmax() << "]\n";
        }
        return os;
    }


    Eigen::VectorXd getQ() const {
        Eigen::VectorXd q(joints_.size());
        for (std::size_t i = 0; i < joints_.size(); ++i)
            q[i] = joints_[i]->getQ();
        return q;
    }

    void setQ(const Eigen::VectorXd& q) {
        if (static_cast<std::size_t>(q.size()) != joints_.size())
            throw std::invalid_argument(
                "setQ: taille du vecteur (" + std::to_string(q.size()) +
                ") != nombre de joints (" + std::to_string(joints_.size()) + ")");
        for (std::size_t i = 0; i < joints_.size(); ++i)
            joints_[i]->setQ(q[i]);
    }

    Eigen::VectorXd randomQ() const {
        Eigen::VectorXd q(joints_.size());
        for (std::size_t i = 0; i < joints_.size(); ++i) {
            double lo = joints_[i]->getQmin();
            double hi = joints_[i]->getQmax();
            // tirage uniforme dans [lo, hi]
            q[i] = lo + (hi - lo) * (static_cast<double>(std::rand()) / RAND_MAX);
        }
        return q;
    }
};

#endif 