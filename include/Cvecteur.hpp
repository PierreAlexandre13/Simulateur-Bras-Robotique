#ifndef CVECTEUR_HPP
#define CVECTEUR_HPP

#include <array>
#include <cmath>
#include <stdexcept>
#include <ostream>
#include <Eigen/Dense>

// ────────────────────────────────────────────────────────────────────────────
//  CVecteur<T, N>
//  Vecteur mathématique de taille fixe N, stocké dans un std::array<T,N>.
// ────────────────────────────────────────────────────────────────────────────
template <typename T, std::size_t N>
class CVecteur {
    //static_assert(N > 0, "CVecteur : N doit être strictement positif");

private:
    std::array<T, N> data_;

public:
    // ── Constructeurs ────────────────────────────────────────────────────────

    // Constructeur par défaut : initialise tous les éléments à zéro
    CVecteur() {
        data_.fill(T{0});
    }

    ~CVecteur() {}

    // Constructeur depuis un std::array
    explicit CVecteur(const std::array<T, N>& arr) : data_(arr) {}

    // Constructeur depuis une liste d'initialisation  {1.0, 2.0, 3.0}
    CVecteur(std::initializer_list<T> init) {
        if (init.size() != N)
            throw std::invalid_argument(
                "CVecteur : taille de la liste (" +
                std::to_string(init.size()) +
                ") != N (" + std::to_string(N) + ")");
        std::copy(init.begin(), init.end(), data_.begin());
    }

    // ── Accès ────────────────────────────────────────────────────────────────

    // Lecture / écriture avec vérification de bornes
    T& operator[](std::size_t i) {
        if (i >= N)
            throw std::out_of_range(
                "CVecteur::operator[] : indice " + std::to_string(i) +
                " hors limites (N = " + std::to_string(N) + ")");
        return data_[i];
    }

    // Lecture seule avec vérification de bornes
    /*const T& operator[](std::size_t i) const {
        if (i >= N)
            throw std::out_of_range(
                "CVecteur::operator[] : indice " + std::to_string(i) +
                " hors limites (N = " + std::to_string(N) + ")");
        return data_[i];
    }*/

    // ── Opérations arithmétiques ─────────────────────────────────────────────

    // Addition élément par élément
    CVecteur operator+(const CVecteur& other) const {
        CVecteur result;
        for (std::size_t i = 0; i < N; ++i)
            result.data_[i] = data_[i] + other.data_[i];
        return result;
    }

    // Soustraction élément par élément
    CVecteur operator-(const CVecteur& other) const {
        CVecteur result;
        for (std::size_t i = 0; i < N; ++i)
            result.data_[i] = data_[i] - other.data_[i];
        return result;
    }

    // Multiplication par un scalaire (vecteur * scalaire)
    CVecteur operator*(T scalar) const {
        CVecteur result;
        for (std::size_t i = 0; i < N; ++i)
            result.data_[i] = data_[i] * scalar;
        return result;
    }

    // Multiplication par un scalaire (scalaire * vecteur) — fonction amie
    /*friend CVecteur operator*(T scalar, const CVecteur& v) {
        return v * scalar;
    }*/

    // ── Produit scalaire ─────────────────────────────────────────────────────

    // Retourne sum(data_[i] * other.data_[i])
    T dot(const CVecteur& other) const {
        T result{0};
        for (std::size_t i = 0; i < N; ++i)
            result += data_[i] * other.data_[i];
        return result;
    }

    // ── Norme euclidienne ────────────────────────────────────────────────────

    // Retourne sqrt(dot(*this))
    T norm() const {
        return std::sqrt(dot(*this));
    }

    // ── Conversion vers Eigen (zéro copie via Eigen::Map) ────────────────────
    //
    //  Eigen::Map<> crée une vue sur le buffer data_.data() sans aucune copie.
    //  On retourne ensuite une Eigen::Matrix pour avoir un objet indépendant
    //  (si l'on retournait le Map directement, il pointerait sur data_ et
    //  deviendrait invalide si le CVecteur est détruit).
    //
    Eigen::Matrix<T, N, 1> toEigen() const {
        // Map sur le buffer brut — zéro copie à cette étape
        Eigen::Map<const Eigen::Matrix<T, N, 1>> map(data_.data());
        // Copie vers une Matrix indépendante et sûre
        return Eigen::Matrix<T, N, 1>(map);
    }

    
    constexpr std::size_t size() const { return N; }

    // Affichage : [v0, v1, ..., vN-1]
    friend std::ostream& operator<<(std::ostream& os, const CVecteur& v) {
        os << "[";
        for (std::size_t i = 0; i < N; ++i) {
            os << v.data_[i];
            if (i + 1 < N) os << ", ";
        }
        os << "]";
        return os;
    }
};

#endif // CVECTEUR_HPP