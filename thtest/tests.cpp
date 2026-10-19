#include "integrator.h"
#include <cmath>
#include <iostream>
#include <stdexcept>

void check(bool passed, const char* description) {
    if (!passed) throw std::runtime_error(description);
    std::cout << "PASS: " << description << '\n';
}

double oscillatorError(double dt, bool implicit) {
    SpringChain s(1, 100.0);
    s.externalForce.setZero();
    s.damping = 0.0;
    s.x[0] = 1.0;
    int steps = static_cast<int>(std::round(1.0 / dt));
    for (int i = 0; i < steps; ++i) {
        if (implicit) s.stepBackwardEuler(dt);
        else s.stepSemiImplicit(dt);
    }
    // m=1, k=100£¬½âÎö½â£ºx=cos(10t), v=-10sin(10t)¡£
    return std::hypot(s.x[0] - std::cos(10.0), (s.v[0] + 10.0 * std::sin(10.0)) / 10.0);
}

int main() {
    try {
        SpringChain s;
        Eigen::VectorXd equilibrium = s.K.ldlt().solve(s.externalForce);
        for (int i = 0; i < 5; ++i) {
            double stretch = equilibrium[i] - (i == 0 ? 0.0 : equilibrium[i - 1]);
            check(std::abs(800.0 * stretch - (5 - i) * 9.81) < 1e-9,
                "Static spring force balances the weight below it");
        }
        s.x = equilibrium;
        SpringChain other = s;
        for (int i = 0; i < 100; ++i) {
            s.stepSemiImplicit(0.001);
            other.stepBackwardEuler(0.05);
        }
        check((s.x - equilibrium).norm() < 1e-10 && (other.x - equilibrium).norm() < 1e-10,
            "Both methods preserve static equilibrium");

        for (bool implicit : {false, true}) {
            double coarse = oscillatorError(0.01, implicit);
            double fine = oscillatorError(0.005, implicit);
            check(fine < 0.75 * coarse, "Halving dt improves accuracy against analytic oscillator");
        }

        SpringChain be;
        be.damping = 0.0;
        be.v[4] = -1.5;
        double lastEnergy = be.energy();
        for (int i = 0; i < 200; ++i) {
            be.stepBackwardEuler(0.05);
            double e = be.energy();
            if (!be.x.allFinite() || e > lastEnergy + 1e-10)
                throw std::runtime_error("Backward Euler energy check failed");
            lastEnergy = e;
        }
        check(true, "Backward Euler dissipates energy without physical damping");

        SpringChain se;
        se.v[4] = -1.5;
        bool large = false;
        for (int i = 0; i < 50; ++i) {
            se.stepSemiImplicit(0.05);
            if (se.x.cwiseAbs().maxCoeff() > 100.0) { large = true; break; }
        }
        check(large, "Large dt exposes the semi-implicit Euler stability limit");
    }
    catch (const std::exception& e) {
        std::cerr << "FAIL: " << e.what() << '\n';
        return 1;
    }
    return 0;
}
