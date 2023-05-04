//
// Created by Andrei on 09.02.22.
//

#ifndef ANDREIUTILS_DUALQUATERNION_HPP
#define ANDREIUTILS_DUALQUATERNION_HPP

#include <AndreiUtils/utils.hpp>
#include <AndreiUtils/utilsQuaternions.hpp>
#include <Eigen/Dense>

namespace AndreiUtils {
    template<class T>
    class DualQuaternion {
    public:
        static DualQuaternion identity() {
            return DualQuaternion::one;
        }

        static DualQuaternion createFromCoefficients(CR<std::vector<T>> coefficients) {
            DualQuaternion<T> q;
            q.fromCoefficients(coefficients);
            return q;
        }

        explicit DualQuaternion(CR<T> q0 = T(0), CR<T> q1 = T(0), CR<T> q2 = T(0), CR<T> q3 = T(0),
                                CR<T> q4 = T(0), CR<T> q5 = T(0), CR<T> q6 = T(0), CR<T> q7 = T(0)) :
                r(q0, q1, q2, q3), d(q4, q5, q6, q7) {}

        DualQuaternion(CR<Eigen::Quaternion<T>> r, CR<Eigen::Quaternion<T>> d) : r(r), d(d) {}

        DualQuaternion(CR<Eigen::Quaternion<T>> r, CR<Eigen::Matrix<T, 3, 1>> t) : r(r) {
            this->d = qMulScalar(vToQ(t) * r, T(0.5));
        }

        DualQuaternion(CR<Eigen::Matrix<T, 3, 3>> r, CR<Eigen::Matrix<T, 3, 1>> t) :
                DualQuaternion(qFromRotationMatrix(r), t) {}

        explicit DualQuaternion(CR<Eigen::Matrix<T, 4, 4>> t) :
                DualQuaternion(qFromRotationMatrix(Eigen::Matrix<T, 3, 3>(t.template block<3, 3>(0, 0))),
                               t.template block<3, 1>(0, 3)) {}

        DualQuaternion(CR<DualQuaternion> other) : r(other.r), d(other.d) {}

        DualQuaternion(U<DualQuaternion> other) noexcept: r(std::move(other.r)), d(std::move(other.d)) {}

        DualQuaternion &operator=(CR<DualQuaternion> other) {
            if (this != &other) {
                this->r = other.r;
                this->d = other.d;
            }
            return *this;
        }

        DualQuaternion &operator=(U<DualQuaternion> other) noexcept {
            if (this != &other) {
                this->r = std::move(other.r);
                this->d = std::move(other.d);
            }
            return *this;
        }

        virtual ~DualQuaternion() = default;

        [[nodiscard]] std::vector<T> coefficients() const {
            return {this->r.w(), this->r.x(), this->r.y(), this->r.z(), this->d.w(), this->d.x(), this->d.y(),
                    this->d.z()};
        }

        [[nodiscard]] Eigen::Matrix<T, 8, 1> coefficientsAsEigen() const {
            Eigen::Matrix<T, 8, 1> res;
            res << this->r.w(), this->r.x(), this->r.y(), this->r.z(), this->d.w(), this->d.x(), this->d.y(),
                    this->d.z();
            return res;
        }

        void fromCoefficients(CR<std::vector<T>> coefficients) {
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
            this->normalize();
        }

        [[nodiscard]] double coefficientNorm() const {
            Eigen::Matrix<double, 8, 1> coefficients;
            coefficients.topRows(4) = this->r.coeffs().template cast<double>();
            coefficients.bottomRows(4) = this->d.coeffs().template cast<double>();
            return coefficients.norm();
        }

        [[nodiscard]] double coefficientSquareSum() const {
            Eigen::Matrix<double, 8, 1> coefficients;
            coefficients.topRows(4) = this->r.coeffs().template cast<double>();
            coefficients.bottomRows(4) = this->d.coeffs().template cast<double>();
            return coefficients.squaredNorm();
        }

        template<class CastType>
        [[nodiscard]] DualQuaternion<CastType> cast() const {
            return {this->r.template cast<CastType>(), this->d.template cast<CastType>()};
        }

        [[nodiscard]] T rotationAngle() const {
            return Eigen::AngleAxis<T>(this->r).angle();
        }

        [[nodiscard]] Eigen::Matrix<T, 3, 1> rotationAxis() const {
            return Eigen::AngleAxis<T>(this->r).axis();
        }

        // inspired by Riddhiman's function
        [[nodiscard]] DualQuaternion powScrew(CR<T> a) const {
            T theta = this->rotationAngle(), n;  // Theta in radians (ALWAYS)
            DualQuaternion res;

            if (theta == 0) {
                n = theta;
            } else {
                n = 0.5 * theta;
            }

            T cos_n = cos(n);
            T cos_a_n = cos(a * n);
            T sin_n = sin(n);
            T sin_a_n = sin(a * n);

            T tmp = -(2 * this->d.w() / sin_n);
            T tmp_cos_2 = 0.5 * tmp * cos_n;

            Eigen::Matrix<T, 3, 1> u, v;
            u.x() = this->r.x();
            u.y() = this->r.y();
            u.z() = this->r.z();
            u /= sin_n;
            v.x() = this->d.x() - tmp_cos_2 * u.x();
            v.y() = this->d.y() - tmp_cos_2 * u.y();
            v.z() = this->d.z() - tmp_cos_2 * u.z();
            v /= sin_n;

            res.r.w() = cos_a_n;
            res.r.x() = u.x() * sin_a_n;
            res.r.y() = u.y() * sin_a_n;
            res.r.z() = u.z() * sin_a_n;

            T tmp_a_2 = 0.5 * tmp * a;

            res.d.w() = -tmp_a_2 * sin_a_n;
            res.d.x() = (v.x() * sin_a_n) + (tmp_a_2 * u.x() * cos_a_n);
            res.d.y() = (v.y() * sin_a_n) + (tmp_a_2 * u.y() * cos_a_n);
            res.d.z() = (v.z() * sin_a_n) + (tmp_a_2 * u.z() * cos_a_n);

            return res;
        }

        // inspired by DQ::log function
        [[nodiscard]] DualQuaternion log() const {
            // Verify if the object caller is a unit DQ
            if (this->norm() != DualQuaternion::identity()) {
                throw std::runtime_error("Bad log() call: Not a unit dual quaternion");
            }

            // log calculation: https://www.sciencedirect.com/science/article/pii/S1474667016395441
            auto angleAxis = Eigen::AngleAxis<T>(this->r);
            Eigen::Matrix<T, 3, 1> _x = angleAxis.angle() * angleAxis.axis();  // primary; axis-angle representation
            Eigen::Quaternion<T> _r(T(0), _x.x(), _x.y(), _x.z());

            _x = this->getTranslation();  // dual
            Eigen::Quaternion<T> _d(T(0), _x.x(), _x.y(), _x.z());

            return {qDivScalar(_r, T(2.)), qDivScalar(_d, T(2.))};
        }

        // inspired by DQ::exp function
        [[nodiscard]] DualQuaternion exp() const {
            if (this->r.w() != 0 && this->d.w() != 0) {
                throw std::runtime_error(
                        "Bad exp() call: Exponential operation is defined only for pure dual quaternions.");
            }

            T phi = this->r.norm();
            Eigen::Quaternion<T> prim;
            if (phi != 0.0) {
                prim = qAdd({cos(phi), 0, 0, 0}, qMulScalar(this->r, sin(phi) / phi));
            } else {
                prim = qIdentity<T>();
            }

            return {prim, this->d * prim};
        }

        // inspired by DQ::pow function
        [[nodiscard]] DualQuaternion powGeom(CR<T> a) const {
            DualQuaternion res = a * this->log();
            return res.exp();
        }

        // inspired by DQ::norm function
        [[nodiscard]] DualQuaternion norm() const {
            /*
            return DualQuaternion(this->r * this->r.conjugate(),
                                  qAdd(this->r * this->d.conjugate(), this->d * this->r.conjugate()));
            //*/

            DualQuaternion norm = this->conjugate() * (*this);
            norm.r.w() = sqrt(norm.r.w());
            norm.d.w() /= (2 * norm.r.w());  // why???
            return norm;
        }

        void normalize() {
            *this = (*this) * (this->norm().dualQuaternionInverse());
            /*
            T norm = this->r.norm();
            if (equal(norm, T(0))) {
                return;
            }
            this->r.normalize();
            //*/
        }

        [[nodiscard]] DualQuaternion normalized() const {
            DualQuaternion res = *this;
            res.normalize();
            return res;
        }

        [[nodiscard]] DualQuaternion conjugate() const {
            return DualQuaternion(this->r.conjugate(), this->d.conjugate());
        }

        [[nodiscard]] DualQuaternion quaternionConjugate() const {
            return this->conjugate();
        }

        [[nodiscard]] DualQuaternion dualConjugate() const {
            return DualQuaternion(this->r, qNeg(this->d));
        }

        [[nodiscard]] DualQuaternion quaternionDualConjugate() const {
            return DualQuaternion(this->r.conjugate(), this->d.conjugate()).dualConjugate();
        }

        [[nodiscard]] DualQuaternion dualQuaternionInverse() const {
            DualQuaternion inv;
            inv.r = this->r.conjugate();
            inv.d = qNeg(inv.r * this->d * inv.r);
            return inv;
        }

        bool equal(CR<DualQuaternion> other, CR<T> tol = 1e-9) const {
            return qEqual(this->r, other.r, tol) && qEqual(this->d, other.d, tol);
        }

        bool notEqual(CR<DualQuaternion> other, CR<T> tol = 1e-9) const {
            return !this->equal(other, tol);
        }

        bool operator==(CR<DualQuaternion> other) const {
            return this->equal(other);
        }

        bool operator!=(CR<DualQuaternion> other) const {
            return !this->equal(other);
        }

        DualQuaternion operator*(CR<T> s) const {
            return DualQuaternion(qMulScalar(this->r, s), qMulScalar(this->d, s));
        }

        DualQuaternion &operator*=(CR<T> s) {
            *this = *this * s;
            return *this;
        }

        DualQuaternion operator/(CR<T> s) const {
            return DualQuaternion(qDivScalar(this->r, s), qDivScalar(this->d, s));
        }

        DualQuaternion &operator/=(CR<T> s) {
            *this = *this / s;
            return *this;
        }

        // q_T = a * b corresponds to T_a_b = T_a_i * T_i_b; *this * other
        DualQuaternion operator*(CR<DualQuaternion> other) const {
            /*
            Eigen::Quaternion<T> real = this->r * other.r;
            Eigen::Quaternion<T> dual((this->d * other.r).coeffs() + (this->r * other.d).coeffs());
            return DualQuaternion(real, dual);
            /*/
            return DualQuaternion(this->r * other.r, qAdd(this->d * other.r, this->r * other.d));
            //*/
        }

        DualQuaternion &operator*=(CR<DualQuaternion> other) {
            (*this) = (*this) * other;
            return *this;
        }

        DualQuaternion operator+(CR<DualQuaternion> other) const {
            return DualQuaternion(qAdd(this->r, other.r), qAdd(this->d, other.d));
        }

        DualQuaternion &operator+=(CR<DualQuaternion> other) {
            (*this) = (*this) + other;
            return *this;
        }

        DualQuaternion operator-() const {
            return DualQuaternion(qNeg(this->r), qNeg(this->d));
        }

        DualQuaternion operator-(CR<DualQuaternion> other) const {
            return (*this) + (-other);
        }

        DualQuaternion &operator-=(CR<DualQuaternion> other) {
            (*this) = (*this) - other;
            return (*this);
        }

        [[nodiscard]] Eigen::Matrix<T, 3, 1> transform(CR<Eigen::Matrix<T, 3, 1>> p) const {
            return ((*this) * DualQuaternion(qIdentity<T>(), vToQ(p)) *
                    this->quaternionDualConjugate()).getDual().vec();
        }

        [[nodiscard]] Eigen::Matrix<T, 3, 1> rotate(CR<Eigen::Matrix<T, 3, 1>> v) const {
            return ((*this) * DualQuaternion(vToQ(v), qZero<T>()) *
                    this->quaternionDualConjugate()).getRotation().vec();
        }

        [[nodiscard]] Eigen::Matrix<T, 3, 1> translate(CR<Eigen::Matrix<T, 3, 1>> t) const {
            return ((*this) * DualQuaternion(qZero<T>(), vToQ(t)) * this->quaternionDualConjugate()).getTranslation();
        }

        [[nodiscard]] DualQuaternion addRotation(CR<Eigen::Quaternion<T>> q) const {
            return DualQuaternion(q * this->r, this->getTranslation());
        }

        [[nodiscard]] DualQuaternion addRotationRight(CR<Eigen::Quaternion<T>> q) const {
            return DualQuaternion(this->r * q, this->getTranslation());
        }

        [[nodiscard]] DualQuaternion addTranslation(CR<Eigen::Matrix<T, 3, 1>> t) const {
            return DualQuaternion(this->r, t + this->getTranslation());
        }

        [[nodiscard]] Eigen::Quaternion<T> getRotation() const {
            return this->r;
        }

        [[nodiscard]] Eigen::Matrix<T, 3, 3> getRotationAsMatrix() const {
            return this->r.toRotationMatrix();
        }

        [[nodiscard]] Eigen::Quaternion<T> getDual() const {
            return this->d;
        }

        [[nodiscard]] Eigen::Matrix<T, 3, 1> getTranslation() const {
            return qMulScalar(this->d * this->r.conjugate(), T(2)).vec();
        }

        [[nodiscard]] Eigen::Matrix<T, 4, 4> getTransformationMatrix() const {
            auto q = this->normalized();
            Eigen::Matrix<T, 4, 4> M = Eigen::Matrix<T, 4, 4>::Identity();
            M.template block<3, 3>(0, 0) = q.getRotationAsMatrix();  // Extract rotational information
            M.col(3).template topRows<3>() = q.getTranslation();  // Extract translation information
            return M;
        }

        [[nodiscard]] DualQuaternion sclerp(CR<T> tau, CR<DualQuaternion> q) {
            DualQuaternion thisNormed = this->normalized(), qNormed = q.normalized();
            DualQuaternion qAsSeenFromThis = thisNormed.dualQuaternionInverse() * qNormed;
            return (tau == 0) ? thisNormed : thisNormed * qAsSeenFromThis.powScrew(tau);
        }

        friend std::ostream &operator<<(std::ostream &os, CR<DualQuaternion> q) {
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

    template<class T>
    DualQuaternion<T> const DualQuaternion<T>::zero;

    template<class T>
    DualQuaternion<T> const DualQuaternion<T>::one = DualQuaternion<T>(T(1));

    template<class T>
    DualQuaternion<T> const DualQuaternion<T>::i = DualQuaternion<T>(T(0), T(1));

    template<class T>
    DualQuaternion<T> const DualQuaternion<T>::j = DualQuaternion<T>(T(0), T(0), T(1));

    template<class T>
    DualQuaternion<T> const DualQuaternion<T>::k = DualQuaternion<T>(T(0), T(0), T(0), T(1));

    template<class T>
    DualQuaternion<T> const DualQuaternion<T>::e = DualQuaternion<T>(T(0), T(0), T(0), T(0), T(1));

    template<class T>
    DualQuaternion<T> const DualQuaternion<T>::ei = DualQuaternion<T>(T(0), T(0), T(0), T(0), T(0), T(1));

    template<class T>
    DualQuaternion<T> const DualQuaternion<T>::ej = DualQuaternion<T>(T(0), T(0), T(0), T(0), T(0), T(0), T(1));

    template<class T>
    DualQuaternion<T> const DualQuaternion<T>::ek = DualQuaternion<T>(T(0), T(0), T(0), T(0), T(0), T(0), T(0), T(1));

    using Posef = DualQuaternion<float>;
    using Posed = DualQuaternion<double>;
    using Poseld = DualQuaternion<long double>;

    using Pose = Posed;
}

#endif //ANDREIUTILS_DUALQUATERNION_HPP
