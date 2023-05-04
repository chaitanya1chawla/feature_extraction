//
// Created by Andrei on 14.12.22.
//

#ifndef COPPELIASIMTUTORIAL_DUALQUATERNION_HPP
#define COPPELIASIMTUTORIAL_DUALQUATERNION_HPP

#include <Eigen/Dense>
#include <vector>

template<typename T>
inline bool equal(T const &a, T const &b, double const &tol = 1e-9) {
    if (std::is_integral<T>::value) {
        return a == b;
    } else {
        T const &diff = a - b;
        return (-tol < diff) && (diff < tol);
    }
}

// q1 == q2
template<class T>
bool qEqual(Eigen::Quaternion<T> const &q1, Eigen::Quaternion<T> const &q2, T const &tol = T(1e-9)) {
    return equal(q1.w(), q2.w(), tol) && equal(q1.x(), q2.x(), tol) && equal(q1.y(), q2.y(), tol) &&
           equal(q1.z(), q2.z(), tol);
}

// q1 + q2
template<class T>
Eigen::Quaternion<T> qAdd(Eigen::Quaternion<T> const &q1, Eigen::Quaternion<T> const &q2) {
    return Eigen::Quaternion<T>(q1.coeffs() + q2.coeffs());
}

// -q
template<class T>
Eigen::Quaternion<T> qNeg(Eigen::Quaternion<T> const &q) {
    return Eigen::Quaternion<T>(-q.coeffs());
}

// quaternion representation of a 3d vector
template<class T>
Eigen::Quaternion<T> vToQ(Eigen::Matrix<T, 3, 1> const &v) {
    Eigen::Quaternion<T> q;
    q.w() = 0;
    q.vec() = v;
    return q;
}

template<class T>
class DualQuaternion {
public:
    static DualQuaternion identity() {
        return DualQuaternion::one;
    }

    static DualQuaternion createFromCoefficients(std::vector<T> const &coefficients) {
        DualQuaternion<T> q;
        q.fromCoefficients(coefficients);
        return q;
    }

    explicit DualQuaternion(T const &q0 = T(0), T const &q1 = T(0), T const &q2 = T(0), T const &q3 = T(0),
                            T const &q4 = T(0), T const &q5 = T(0), T const &q6 = T(0), T const &q7 = T(0)) :
            r(q0, q1, q2, q3), d(q4, q5, q6, q7) {}

    DualQuaternion(Eigen::Quaternion<T> const &r, Eigen::Quaternion<T> const &d) : r(r), d(d) {}

    DualQuaternion(Eigen::Quaternion<T> const &r, Eigen::Matrix<T, 3, 1> const &t) : r(r) {
        Eigen::Quaternion<T> q;
        q.w() = 0;
        q.vec() = t;
        this->d = Eigen::Quaternion<T>((q * r).coeffs() * T(0.5));
    }

    DualQuaternion(Eigen::Matrix<T, 3, 3> const &r, Eigen::Matrix<T, 3, 1> const &t) :
            DualQuaternion(Eigen::Quaternion<T>(r), t) {}

    explicit DualQuaternion(Eigen::Matrix<T, 4, 4> const &t) :
            DualQuaternion(Eigen::Quaternion<T>(Eigen::Matrix<T, 3, 3>(t.template block<3, 3>(0, 0))),
                           t.template block<3, 1>(0, 3)) {}

    DualQuaternion(DualQuaternion const &other) : r(other.r), d(other.d) {}

    DualQuaternion(DualQuaternion &&other) noexcept: r(std::move(other.r)), d(std::move(other.d)) {}

    DualQuaternion &operator=(DualQuaternion const &other) {
        if (this != &other) {
            this->r = other.r;
            this->d = other.d;
        }
        return *this;
    }

    DualQuaternion &operator=(DualQuaternion &&other) noexcept {
        if (this != &other) {
            this->r = std::move(other.r);
            this->d = std::move(other.d);
        }
        return *this;
    }

    virtual ~DualQuaternion() = default;

    std::vector<T> coefficients() const {
        return {this->r.w(), this->r.x(), this->r.y(), this->r.z(), this->d.w(), this->d.x(), this->d.y(),
                this->d.z()};
    }

    Eigen::Matrix<T, 8, 1> coefficientsAsEigen() const {
        Eigen::Matrix<T, 8, 1> res;
        res << this->r.w(), this->r.x(), this->r.y(), this->r.z(), this->d.w(), this->d.x(), this->d.y(),
                this->d.z();
        return res;
    }

    void fromCoefficients(std::vector<T> const &coefficients) {
        if (coefficients.size() != 8) {
            throw std::runtime_error(
                    "Coefficients' size is not 8 (is " + std::to_string(coefficients.size()) + ")!");
        }
        this->r.w() = coefficients[0];
        this->r.x() = coefficients[1];
        this->r.y() = coefficients[2];
        this->r.z() = coefficients[3];
        this->d.w() = coefficients[4];
        this->d.x() = coefficients[5];
        this->d.y() = coefficients[6];
        this->d.z() = coefficients[7];
    }

    double coefficientNorm() const {
        Eigen::Matrix<double, 8, 1> coefficients;
        coefficients.topRows(4) = this->r.coeffs().template cast<double>();
        coefficients.bottomRows(4) = this->d.coeffs().template cast<double>();
        return coefficients.norm();
    }

    double coefficientSquareSum() const {
        Eigen::Matrix<double, 8, 1> coefficients;
        coefficients.topRows(4) = this->r.coeffs().template cast<double>();
        coefficients.bottomRows(4) = this->d.coeffs().template cast<double>();
        return coefficients.squaredNorm();
    }

    template<class CastType>
    DualQuaternion<CastType> cast() const {
        return {this->r.template cast<CastType>(), this->d.template cast<CastType>()};
    }

    T rotationAngle() const {
        return Eigen::AngleAxis<T>(this->r).angle();
    }

    Eigen::Matrix<T, 3, 1> rotationAxis() const {
        return Eigen::AngleAxis<T>(this->r).axis();
    }

    // inspired by DQ::norm function
    DualQuaternion norm() const {
        DualQuaternion norm = this->conjugate() * (*this);
        norm.r.w() = sqrt(norm.r.w());
        norm.d.w() /= (2 * norm.r.w());  // why???
        return norm;
    }

    void normalize() {
        *this = (*this) * (this->norm().dualQuaternionInverse());
    }

    DualQuaternion normalized() const {
        DualQuaternion res = *this;
        res.normalize();
        return res;
    }

    DualQuaternion conjugate() const {
        return DualQuaternion(this->r.conjugate(), this->d.conjugate());
    }

    DualQuaternion quaternionConjugate() const {
        return this->conjugate();
    }

    DualQuaternion dualConjugate() const {
        return DualQuaternion(this->r, qNeg(this->d));
    }

    DualQuaternion quaternionDualConjugate() const {
        return DualQuaternion(this->r.conjugate(), this->d.conjugate()).dualConjugate();
    }

    DualQuaternion dualQuaternionInverse() const {
        DualQuaternion inv;
        inv.r = this->r.conjugate();
        inv.d = qNeg(inv.r * this->d * inv.r);
        return inv;
    }

    bool equal(DualQuaternion const &other, T const &tol = 1e-9) const {
        return qEqual(this->r, other.r, tol) && qEqual(this->d, other.d, tol);
    }

    bool notEqual(DualQuaternion const &other, T const &tol = 1e-9) const {
        return !this->equal(other, tol);
    }

    bool operator==(DualQuaternion const &other) const {
        return this->equal(other);
    }

    bool operator!=(DualQuaternion const &other) const {
        return !this->equal(other);
    }

    DualQuaternion operator*(T const &s) const {
        return DualQuaternion(qMulScalar(this->r, s), qMulScalar(this->d, s));
    }

    DualQuaternion &operator*=(T const &s) {
        *this = *this * s;
        return *this;
    }

    DualQuaternion operator/(T const &s) const {
        return DualQuaternion(qDivScalar(this->r, s), qDivScalar(this->d, s));
    }

    DualQuaternion &operator/=(T const &s) {
        *this = *this / s;
        return *this;
    }

    // q_T = a * b corresponds to T_a_b = T_a_i * T_i_b; *this * other
    DualQuaternion operator*(DualQuaternion const &other) const {
        return DualQuaternion(this->r * other.r, qAdd(this->d * other.r, this->r * other.d));
    }

    DualQuaternion &operator*=(DualQuaternion const &other) {
        (*this) = (*this) * other;
        return *this;
    }

    DualQuaternion operator+(DualQuaternion const &other) const {
        return DualQuaternion(qAdd(this->r, other.r), qAdd(this->d, other.d));
    }

    DualQuaternion &operator+=(DualQuaternion const &other) {
        (*this) = (*this) + other;
        return *this;
    }

    DualQuaternion operator-() const {
        return DualQuaternion(qNeg(this->r), qNeg(this->d));
    }

    DualQuaternion operator-(DualQuaternion const &other) const {
        return (*this) + (-other);
    }

    DualQuaternion &operator-=(DualQuaternion const &other) {
        (*this) = (*this) - other;
        return (*this);
    }

    Eigen::Matrix<T, 3, 1> transform(Eigen::Matrix<T, 3, 1> const &p) const {
        return ((*this) * DualQuaternion(Eigen::Quaternion<T>(1, 0, 0, 0), vToQ(p)) *
                this->quaternionDualConjugate()).getDual().vec();
    }

    Eigen::Matrix<T, 3, 1> rotate(Eigen::Matrix<T, 3, 1> const &v) const {
        return ((*this) * DualQuaternion(vToQ(v), Eigen::Quaternion<T>(0, 0, 0, 0)) *
                this->quaternionDualConjugate()).getRotation().vec();
    }

    Eigen::Matrix<T, 3, 1> translate(Eigen::Matrix<T, 3, 1> const &t) const {
        return ((*this) * DualQuaternion(Eigen::Quaternion<T>(0, 0, 0, 0), vToQ(t)) *
                this->quaternionDualConjugate()).getTranslation();
    }

    DualQuaternion addRotation(Eigen::Quaternion<T> const &q) const {
        return DualQuaternion(q * this->r, this->getTranslation());
    }

    DualQuaternion addRotationRight(Eigen::Quaternion<T> const &q) const {
        return DualQuaternion(this->r * q, this->getTranslation());
    }

    DualQuaternion addTranslation(Eigen::Matrix<T, 3, 1> const &t) const {
        return DualQuaternion(this->r, t + this->getTranslation());
    }

    Eigen::Quaternion<T> getRotation() const {
        return this->r;
    }

    Eigen::Matrix<T, 3, 3> getRotationAsMatrix() const {
        return this->r.toRotationMatrix();
    }

    Eigen::Quaternion<T> getDual() const {
        return this->d;
    }

    Eigen::Matrix<T, 3, 1> getTranslation() const {
        return qMulScalar(this->d * this->r.conjugate(), T(2)).vec();
    }

    Eigen::Matrix<T, 4, 4> getTransformationMatrix() const {
        auto q = this->normalized();
        Eigen::Matrix<T, 4, 4> M = Eigen::Matrix<T, 4, 4>::Identity();
        M.template block<3, 3>(0, 0) = q.getRotationAsMatrix();  // Extract rotational information
        M.col(3).template topRows<3>() = q.getTranslation();  // Extract translation information
        return M;
    }

    DualQuaternion sclerp(T const &tau, DualQuaternion const &q) {
        DualQuaternion thisNormed = this->normalized(), qNormed = q.normalized();
        DualQuaternion qAsSeenFromThis = thisNormed.dualQuaternionInverse() * qNormed;
        return (tau == 0) ? thisNormed : thisNormed * qAsSeenFromThis.powScrew(tau);
    }

    friend std::ostream &operator<<(std::ostream &os, DualQuaternion const &q) {
        os << q.r << " " << q.d;
        return os;
    }

    friend std::istream &operator>>(std::istream &is, DualQuaternion &q) {
        is >> q.r >> q.d;
        return is;
    }

    static DualQuaternion<T> const zero;
    static DualQuaternion<T> const one;
    static DualQuaternion<T> const i;
    static DualQuaternion<T> const j;
    static DualQuaternion<T> const k;
    static DualQuaternion<T> const e;
    static DualQuaternion<T> const ei;
    static DualQuaternion<T> const ej;
    static DualQuaternion<T> const ek;

protected:
    Eigen::Quaternion<T> r;
    Eigen::Quaternion<T> d;
};

#endif //COPPELIASIMTUTORIAL_DUALQUATERNION_HPP
