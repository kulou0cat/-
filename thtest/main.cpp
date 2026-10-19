#include "integrator.h"
#include <opencv2/opencv.hpp>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <string>

// 初学者先阅读 integrator.h。下面主要是显示与键盘交互。
// OpenCV 默认字体不支持中文，所以窗口文字采用英文。
void text(cv::Mat& image, const std::string& words, int x, int y,
    cv::Scalar color = cv::Scalar(215, 220, 230)) {
    cv::putText(image, words, cv::Point(x, y), cv::FONT_HERSHEY_SIMPLEX,
        0.52, color, 1, cv::LINE_AA);
}

void drawChain(cv::Mat& image, const SpringChain& sim, int left,
    const std::string& title, bool stopped) {
    const int centerX = left + 240;
    const int top = 180;
    const double pixelsPerMeter = 125.0;
    const double restLength = 0.38; // 仅影响自然位置和显示，不改变刚度
    const cv::Scalar blue(240, 180, 70);

    text(image, title, left + 25, 125, blue);
    text(image, "Energy above equilibrium: " + cv::format("%.4f J", sim.energy()),
        left + 25, 150);
    cv::line(image, { centerX - 40, top }, { centerX + 40, top }, blue, 3);

    cv::Point previous(centerX, top);
    for (int i = 0; i < sim.x.size(); ++i) {
        const double naturalY = restLength * (i + 1);
        const double actualY = naturalY + sim.x[i];
        // 防止失稳时屏幕坐标溢出；此截断只影响显示，不修改模拟状态。
        int screenY = static_cast<int>(std::clamp(top + actualY * pixelsPerMeter,
            175.0, 550.0));
        cv::Point current(centerX, screenY);
        cv::circle(image, { centerX + 85, top + static_cast<int>(naturalY * pixelsPerMeter) },
            4, cv::Scalar(95, 95, 95), 1, cv::LINE_AA);

        // 用折线画弹簧，粒子的运动实际只有竖直一个自由度。
        cv::Point last = previous;
        for (int j = 1; j <= 12; ++j) {
            int dx = (j == 12) ? 0 : ((j % 2 == 0) ? -8 : 8);
            cv::Point next(centerX + dx, previous.y + (current.y - previous.y) * j / 12);
            cv::line(image, last, next, cv::Scalar(145, 155, 165), 1, cv::LINE_AA);
            last = next;
        }
        cv::circle(image, current, 11, blue, -1, cv::LINE_AA);
        previous = current;
    }
    if (stopped)
        text(image, "STOP: state exceeded demo range; press R", left + 20, 575,
            cv::Scalar(80, 100, 255));
}

// 发散后冻结当前状态，便于比较；不是碰撞处理或稳定化算法。
bool advance(SpringChain& sim, double dt, bool useBackwardEuler) {
    SpringChain old = sim;
    if (useBackwardEuler) sim.stepBackwardEuler(dt);
    else sim.stepSemiImplicit(dt);
    if (!sim.x.allFinite() || !sim.v.allFinite() ||
        sim.x.cwiseAbs().maxCoeff() > 2.5 || sim.v.cwiseAbs().maxCoeff() > 100.0) {
        sim = old;
        return false;
    }
    return true;
}

int main(int argc, char** argv) {
    // --snapshot 文件名：生成一张对比图，不创建窗口，用于验证或汇报。
    bool snapshot = argc == 3 && std::string(argv[1]) == "--snapshot";
    double dt = 1.0 / 240.0;
    double stiffness = 800.0;
    double damping = 0.4;
    double time = 0.0;
    bool paused = false, leftStopped = false, rightStopped = false;
    SpringChain semi, implicit;

    auto reset = [&]() {
        semi = SpringChain(5, stiffness);
        semi.damping = damping;
        semi.v[4] = -1.5; // 给最下方质点一个向上的初始速度
        implicit = semi; // 两种方法必须从完全相同的状态开始
        time = 0.0;
        leftStopped = rightStopped = false;
        };
    reset();

    std::cout << "Left: semi-implicit Euler. Right: backward Euler.\n"
        << "Space: pause | N: one step | R: reset | Esc: quit\n"
        << "1/2/3: dt=1/240,1/60,1/20 | K: stiffness | D: damping\n";

    try {
        int key = -1;
        for (int frame = 0; ; ++frame) {
            // 每个显示帧推进一个物理步；展示速度不绑定真实时间。
            if (!paused || key == 'n' || key == 'N') {
                if (!leftStopped) leftStopped = !advance(semi, dt, false);
                if (!rightStopped) rightStopped = !advance(implicit, dt, true);
                time += dt;
            }
            cv::Mat image(650, 1000, CV_8UC3, cv::Scalar(28, 24, 20));
            text(image, "GAMES201: time integration / linear spring chain", 25, 30);
            text(image, cv::format("Requested simulation time: %.3f s | dt=%.5f | k=%.0f | damping=%.1f",
                time, dt, stiffness, damping), 25, 57);
            text(image, "1/2/3 dt | K stiffness | D damping | Space pause | N step | R reset | Esc quit", 25, 84);
            cv::line(image, { 500, 105 }, { 500, 590 }, cv::Scalar(70, 70, 70));
            drawChain(image, semi, 0, "Semi-implicit Euler (velocity first)", leftStopped);
            drawChain(image, implicit, 500, "Backward Euler (Eigen LDLT solve)", rightStopped);
            text(image, "Small gray circles: natural positions. Downward is positive.", 25, 610);
            text(image, "One physics step per displayed frame; NOT synchronized to wall-clock time.", 25, 633);

            if (snapshot) {
                if (frame == 119) {
                    if (!cv::imwrite(argv[2], image)) return 1;
                    std::cout << "Snapshot saved after 120 steps.\n";
                    break;
                }
                continue;
            }
            cv::imshow("GAMES201 Time Integrator", image);
            key = cv::waitKey(16) & 0xff;
            if (key == 27) break;
            if (key == ' ') paused = !paused;
            if (key == 'r' || key == 'R') reset();
            if (key >= '1' && key <= '3') {
                dt = key == '1' ? 1.0 / 240.0 : (key == '2' ? 1.0 / 60.0 : 1.0 / 20.0);
                reset();
            }
            if (key == 'k' || key == 'K') {
                stiffness = stiffness == 800.0 ? 8000.0 : 800.0;
                reset();
            }
            if (key == 'd' || key == 'D') {
                damping = damping == 0.4 ? 0.0 : 0.4;
                reset();
            }
        }
    }
    catch (const cv::Exception& e) {
        std::cerr << "OpenCV error: " << e.what() << '\n';
        return 1;
    }
    return 0;
}
