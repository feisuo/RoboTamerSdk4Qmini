// Standalone Startq (motor zero-position) calibration tool.
// Run from bin/ so it can find/write motor_calib.yaml next to the other runtime configs.
//
// 独立的 Startq（电机零位偏移）标定工具。
// 需在 bin/ 目录下运行，才能读写与其他运行时配置放在一起的 motor_calib.yaml。
#include <unitree/idl/hg/LowCmd_.hpp>
#include <unitree/idl/hg/LowState_.hpp>
#include "Motor_thread.hpp"
#include <iostream>
#include <iomanip>
#include <string>
#include <algorithm>
#include <limits>
#include <cmath>
#include <thread>
#include <chrono>

namespace {

/// Joint angles at the pose where every link frame is parallel to base_link. Obtained by
/// cancelling the fixed rotations baked into the joint origins of assets/q1/urdf/q1.urdf
/// (hip_yaw +-0.4, hip_pitch 1.5, knee 1.05, ankle 1.22).
///
/// This is the calibration reference rather than q = 0, because q = 0 folds the foot above
/// the hip and puts all six pitch joints exactly on their limit, which is awkward to hold and
/// impossible to verify. At the alignment pose every link points the same way as the torso,
/// so with the torso level the soles are horizontal and the toes point straight forward -
/// something a spirit level and a square can confirm.
///
/// 对齐位姿：所有连杆坐标系都与 base_link 平行时的各关节角。由 assets/q1/urdf/q1.urdf
/// 中烘焙在关节 origin 里的固定旋转（hip_yaw ±0.4、hip_pitch 1.5、knee 1.05、ankle 1.22）
/// 逐一抵消得到。
///
/// 用它而不用 q = 0 作标定基准：q = 0 时脚踝翻折到髋关节上方，且六个 pitch 关节恰好
/// 全部压在限位边界上，既难摆住又无从核验。而对齐位姿下每节连杆的朝向都与躯干一致，
/// 所以躯干摆水平时脚掌应水平、脚尖应正前方 —— 这是水平尺和直角尺能量出来的。
const std::array<float, 10> kAlignPose = {0.4f, 0.0f, -1.5f, 1.05f, -1.22f,
                                          -0.4f, 0.0f, 1.5f, -1.05f, 1.22f};

/// Startq belongs to one particular robot's assembly, so its numeric value can never be
/// matched against a reference vector. Only the pose can be validated, hence these checks
/// are advisory: they catch a shaky or lopsided pose, but the operator decides what to save.
///
/// Startq 归属于某一台机器的具体装配，其数值永远不该拿去和一组参考值比对。可校验的
/// 只有姿态本身，所以下面这些检查一律只作提示：能查出姿态不稳或左右不一致，但存不存
/// 由操作者决定。
constexpr float kStillTol = 0.02f;      ///< peak-to-peak per joint while sampling / 采样期间单关节允许的峰峰值 [rad]
constexpr float kSymmetryTol = 0.05f;   ///< left/right mismatch of the pose error / 姿态误差允许的左右失配量 [rad]
constexpr float kPoseErrorWarn = 0.20f; ///< pose error worth flagging / 值得告警的姿态误差，按旧偏移量衡量 [rad]
constexpr float kRad2Deg = 180.f / static_cast<float>(M_PI);

const char *const kJointNames[10] = {"l_hyaw", "l_hrol", "l_hpit", "l_knee", "l_apit",
                                     "r_hyaw", "r_hrol", "r_hpit", "r_knee", "r_apit"};

struct PoseSample {
    std::array<float, 10> raw{};  ///< mean raw joint angle over the window / 采样窗口内的原始关节角均值
    std::array<float, 10> ptp{};  ///< peak-to-peak spread / 峰峰值，用于判断机器人是否被扶稳
};

/// The raw angle is the reported angle plus the offset currently in effect.
/// 原始角 = 上报角 + 当前生效的偏移量。
PoseSample SamplePose(const MotorController &motor, int sampleCount, int sampleIntervalMs) {
    std::array<double, 10> sum{};
    std::array<float, 10> lo{}, hi{};
    sum.fill(0.0);
    lo.fill(std::numeric_limits<float>::max());
    hi.fill(std::numeric_limits<float>::lowest());

    for (int s = 0; s < sampleCount; ++s) {
        for (int i = 0; i < 10; ++i) {
            const float raw = motor.allMotorData[i].q + motor.Startq[i];
            sum[i] += raw;
            lo[i] = std::min(lo[i], raw);
            hi[i] = std::max(hi[i], raw);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(sampleIntervalMs));
    }

    PoseSample sample;
    for (int i = 0; i < 10; ++i) {
        sample.raw[i] = static_cast<float>(sum[i] / sampleCount);
        sample.ptp[i] = hi[i] - lo[i];
    }
    return sample;
}

/// Declaring the sampled pose to be kAlignPose means logical q = raw - Startq must equal
/// kAlignPose there.
/// 把采样到的姿态声明为 kAlignPose，即要求该处的逻辑角 q = raw - Startq 等于 kAlignPose。
std::array<float, 10> StartqFor(const PoseSample &sample) {
    std::array<float, 10> startq{};
    for (int i = 0; i < 10; ++i) startq[i] = sample.raw[i] - kAlignPose[i];
    return startq;
}

/// How far the sampled pose sits from kAlignPose, measured with the offsets already in effect.
/// Only as trustworthy as the previous calibration, but it is the one absolute-ish handle there is.
///
/// 用当前已生效的偏移量衡量，采样姿态离 kAlignPose 还差多少。
/// 其可信度取决于上一次标定，但这已是手上唯一接近绝对的参照。
std::array<float, 10> PoseError(const PoseSample &sample, const std::array<float, 10> &previous) {
    std::array<float, 10> error{};
    for (int i = 0; i < 10; ++i) error[i] = (sample.raw[i] - previous[i]) - kAlignPose[i];
    return error;
}

void PrintPose(const PoseSample &sample, const std::array<float, 10> &previous,
               const std::array<float, 10> &error) {
    std::cout << std::fixed;
    std::cout << "  idx joint    measured   target   error(deg)  spread(deg)  new startq" << std::endl;
    for (int i = 0; i < 10; ++i) {
        std::cout << "  " << std::setw(3) << i
                  << " " << std::setw(7) << kJointNames[i]
                  << " " << std::setw(9) << std::setprecision(4) << sample.raw[i] - previous[i]
                  << " " << std::setw(8) << kAlignPose[i]
                  << " " << std::setw(12) << std::setprecision(1) << error[i] * kRad2Deg
                  << " " << std::setw(12) << sample.ptp[i] * kRad2Deg
                  << " " << std::setw(11) << std::setprecision(4) << sample.raw[i] - kAlignPose[i]
                  << (std::fabs(error[i]) > kPoseErrorWarn ? "  <- far off" : "")
                  << std::endl;
    }
    std::cout << std::setprecision(2);
}

void PrintChecks(const PoseSample &sample, const std::array<float, 10> &error) {
    int worst = 0;
    for (int i = 1; i < 10; ++i) {
        if (sample.ptp[i] > sample.ptp[worst]) worst = i;
    }
    if (sample.ptp[worst] > kStillTol) {
        std::cout << "  [!] not steady: " << kJointNames[worst] << " drifted "
                  << sample.ptp[worst] * kRad2Deg << " deg during the window."
                  << " Support the robot better and sample again." << std::endl;
    } else {
        std::cout << "  [ok] held steady (worst spread " << sample.ptp[worst] * kRad2Deg << " deg)" << std::endl;
    }

    // Both kAlignPose and the joint angle convention are anti-symmetric between the legs
    // (see ref_joint_act in config.yaml), so a pose mirrored as well as the previous
    // calibration pose was has error_left + error_right == 0. This says nothing about the
    // absolute zero, only that the two legs were posed alike.
    //
    // kAlignPose 与关节角约定在左右腿之间都是反对称的（参见 config.yaml 的 ref_joint_act），
    // 因此只要姿态摆得和上次标定时一样镜像，就有 误差_左 + 误差_右 == 0。此项说明不了
    // 绝对零位在哪，只能说明两条腿摆得一致。
    float worst_residual = 0.f;
    int worst_pair = 0;
    for (int i = 0; i < 5; ++i) {
        const float residual = error[i] + error[i + 5];
        if (std::fabs(residual) > std::fabs(worst_residual)) {
            worst_residual = residual;
            worst_pair = i;
        }
    }
    if (std::fabs(worst_residual) > kSymmetryTol) {
        std::cout << "  [!] legs not mirrored: " << kJointNames[worst_pair] << "/"
                  << kJointNames[worst_pair + 5] << " differ by " << worst_residual * kRad2Deg
                  << " deg (assumes the previous calibration was left/right symmetric)." << std::endl;
    } else {
        std::cout << "  [ok] legs mirrored within " << std::fabs(worst_residual) * kRad2Deg
                  << " deg" << std::endl;
    }

    float max_error = 0.f;
    for (int i = 0; i < 10; ++i) max_error = std::max(max_error, std::fabs(error[i]));
    if (max_error > kPoseErrorWarn) {
        std::cout << "  [!] pose is up to " << max_error * kRad2Deg
                  << " deg away from the alignment pose. Either it is not posed accurately,"
                  << " or the previous calibration was already off." << std::endl;
    }
}

} // namespace

int main() {
    std::cout << "=== Qmini Startq (motor zero-position) calibration ===" << std::endl;
    std::cout << "Startq is per-robot: it records this robot's raw joint angles at a known pose," << std::endl;
    std::cout << "so there is no correct numeric value to aim for. What must be reproduced is the" << std::endl;
    std::cout << "pose, and the reference pose here is the URDF alignment pose: every link points" << std::endl;
    std::cout << "the same way as the torso." << std::endl;
    std::cout << std::endl;
    std::cout << "1. Support the robot so all 10 joints hang free of the ground and any obstacle," << std::endl;
    std::cout << "   with the torso level (check it with a spirit level)." << std::endl;
    std::cout << "2. Pose it so that, for both legs, the soles are horizontal, the toes point straight" << std::endl;
    std::cout << "   forward, and the hips are square - no splay, no lean." << std::endl;
    std::cout << "3. Hold it still and press Enter to sample." << std::endl;

    MotorController motor;
    std::cout << "Waiting for motor communication to settle..." << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    const std::array<float, 10> previous = motor.Startq;
    int attempt = 0;

    while (true) {
        std::cout << std::endl << "Press Enter to sample (q = quit): " << std::flush;
        std::string line;
        if (!std::getline(std::cin, line)) break;
        if (line == "q" || line == "Q") break;

        ++attempt;
        std::cout << "Sampling, please keep the robot still..." << std::endl;
        const PoseSample sample = SamplePose(motor, 100, 5);
        const std::array<float, 10> error = PoseError(sample, previous);

        std::cout << "Attempt " << attempt << ":" << std::endl;
        PrintPose(sample, previous, error);
        PrintChecks(sample, error);

        std::cout << std::endl << "Save this pose as the alignment pose? (y = save, Enter = sample again, q = quit): "
                  << std::flush;
        if (!std::getline(std::cin, line)) break;
        if (line == "q" || line == "Q") break;
        if (line != "y" && line != "Y") continue;

        motor.Startq = StartqFor(sample);
        motor.SaveStartq(motor.calibFilePath);

        std::cout << "Calibration done. New Startq saved to " << motor.calibFilePath << ":" << std::endl;
        std::cout << std::setprecision(6);
        for (size_t i = 0; i < motor.Startq.size(); ++i) {
            std::cout << "  q" << i << " (" << kJointNames[i] << ") = " << motor.Startq[i] << std::endl;
        }
        std::cout << "Verify on the robot: run run_interface and enter position stand mode, the legs"
                  << " should settle symmetrically on ref_joint_act." << std::endl;
        motor.Stop();
        return 0;
    }

    std::cout << "Nothing saved, " << motor.calibFilePath << " left untouched." << std::endl;
    motor.Stop();
    return 1;
}

