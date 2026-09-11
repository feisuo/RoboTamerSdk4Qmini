#include <unistd.h>
#include <iostream>
#include <vector>
#include <chrono>
#include <array>
#include <memory>
#include <thread>
#include <mutex>
#include <atomic>
#include <csignal>
#include "serialPort/SerialPort.h"
#include "unitreeMotor/unitreeMotor.h"
#include <fstream>
#include <iomanip>
#include <yaml-cpp/yaml.h>

struct SerialGroup {
    const char *port;
    std::vector<int> motorIDs;
};

class MotorController {
public:
    std::vector<SerialGroup> serialGroups = {
        {"/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTB5JTP9-if03-port0", {0,5}},
        {"/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTB5JTP9-if02-port0", {1,6}},
        {"/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTB5JTP9-if00-port0", {2, 3, 4}},
        {"/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTB5JTP9-if01-port0", {7, 8, 9}}
    };
    MotorController() {
	LoadStartq(calibFilePath);    
        InitializeSerialPorts();
        for(std::array<ThreadData, 4>::iterator td = threadData.begin(); td != threadData.end(); ++td) {
            td->start_time = std::chrono::high_resolution_clock::now();
        }
        // Start the motor control thread
        workerThreads[0] = std::thread(&MotorController::RunThread<0>, this);
        workerThreads[1] = std::thread(&MotorController::RunThread<1>, this);
        workerThreads[2] = std::thread(&MotorController::RunThread<2>, this);
        workerThreads[3] = std::thread(&MotorController::RunThread<3>, this);
        workerThreads[4] = std::thread(&MotorController::MonitorThread, this);
        std::cout << "Start motor thread： Done!" << std::endl;
    }
    ~MotorController() {
        Stop();
        if (dataFile.is_open()) {
            dataFile.close();
        }
    }
    struct ThreadData {
        std::atomic<int> count{0};
        std::chrono::time_point<std::chrono::high_resolution_clock> start_time;
    };

    std::array<ThreadData, 4> threadData;
    std::mutex printMutex;
    std::atomic<bool> running{true};
    std::array<std::mutex, 10> motorMutexes;
    std::mutex fileMutex;
    std::array<std::thread, 5> workerThreads; 

    unitree_hg::msg::dds_::LowCmd_ current_cmd_;
    std::mutex cmd_mutex_;

    void Run(const unitree_hg::msg::dds_::LowCmd_& dds_low_command) {
        {
            std::lock_guard<std::mutex> lock(cmd_mutex_);
            current_cmd_ = dds_low_command;
        }

    }

    void Stop() {
        running = false;
        for(std::thread& thread : workerThreads) {
            if(thread.joinable()) {
                thread.join();
            }
        }
        std::cout << "All worker threads stopped" << std::endl;
    }

public:

    /// Startq（0位偏移）： 左腿roll 内扣，则需增大，右腿内扣则需减小
    /// 这是每台机器人装配后的电机零位标定值，因个体而异。
    /// 下面这组数值只是找不到标定文件时的兜底默认值，实际值以 calibFilePath
    /// （默认 motor_calib.yaml）里的内容为准，由 calibrate_motor 工具生成。
    std::array<float, 10> Startq ={0.65,  0.45 , 1.28,   0.86,  0.56,
                                   0.8, 0.,  0.301131,  0.513495,  0.2};
     
    std::string calibFilePath = "motor_calib.yaml";

    //    std::array<float, 10> Startq ={0.,  0. , 0,   0.0,  0.0, 0.0, -0.0,  0.0,  0.0,  0.0};

    std::array<MotorData, 10> allMotorData;
    float Speed_Ratio = 6.33;
    float Gear_Ratio = 3.;
    std::vector<std::unique_ptr<SerialPort>> serialPorts;

    std::ofstream dataFile;
    std::chrono::time_point<std::chrono::system_clock> lastSaveTime;
    const std::chrono::milliseconds saveInterval{4}; // 100ms保存一次

    void InitializeSerialPorts() {
        for(std::vector<SerialGroup>::iterator group = serialGroups.begin(); group != serialGroups.end(); ++group) {
            std::unique_ptr<SerialPort> port = std::make_unique<SerialPort>(group->port);
            serialPorts.push_back(std::move(port));
        }
    }

    template<int N>
    void RunThread() {

        SerialPort& serial = *serialPorts[N];
        ThreadData& td = threadData[N];
        
         while(running)
           {
            std::chrono::time_point<std::chrono::high_resolution_clock> start = std::chrono::high_resolution_clock::now();
            
            for(std::vector<int>::iterator motorID = serialGroups[N].motorIDs.begin(); 
                motorID != serialGroups[N].motorIDs.end(); ++motorID) {
                MotorCmd cmd;
                MotorData data;
                
                ConfigureMotorCommand(cmd, *motorID, current_cmd_);
                data.motorType = MotorType::GO_M8010_6;
                serial.sendRecv(&cmd, &data);
                ParseMotorFeedback(data, *motorID);
            }
            td.count++;
          }
    }

    void MonitorThread() {
            std::this_thread::sleep_for(std::chrono::seconds(1));
            
            std::lock_guard<std::mutex> lock(printMutex);
            std::chrono::time_point<std::chrono::high_resolution_clock> now = std::chrono::high_resolution_clock::now();
            
            for(int i = 0; i < 4; ++i) {
                long elapsed = std::chrono::duration_cast<std::chrono::seconds>(
                    now - threadData[i].start_time).count();
                int freq = elapsed > 0 ? threadData[i].count / elapsed : 0;
                
                threadData[i].count = 0;
                threadData[i].start_time = now;
            }
    }

    int CalculateChannelID(int motorID) {
        if (motorID == 1) return motorID - 1;
        else if (motorID >= 2 && motorID <= 4) return motorID - 2;
        else if (motorID == 5) return motorID - 4;
        else if (motorID == 6) return motorID - 5;
        else if (motorID >= 7 && motorID <= 9) return motorID - 7;
        return motorID;
    }

    bool IsSpecialMotor(int motorID) const {
        return (motorID == 1 || motorID == 6);
    }

    void PrintFeedback() {
        for (int i = 0; i < 10; ++i) {
            std::cout << "m" << i << ": ";
            if (IsSpecialMotor(i)) {
                std::cout << allMotorData[i].q;
            } else {
                std::cout << allMotorData[i].q;
            }
        }
        std::cout << std::endl;
    }

    void ConfigureMotorCommand(MotorCmd& cmd, int motorID, const unitree_hg::msg::dds_::LowCmd_& dds_low_command) {
        cmd.motorType = MotorType::GO_M8010_6;
        cmd.mode = queryMotorMode(MotorType::GO_M8010_6, MotorMode::FOC);
        cmd.id = CalculateChannelID(motorID);
        cmd.kp = dds_low_command.motor_cmd().at(motorID).kp();
        cmd.kd = dds_low_command.motor_cmd().at(motorID).kd();
        cmd.tau = dds_low_command.motor_cmd().at(motorID).tau();
        
        const bool is_special = IsSpecialMotor(motorID);
        const float ratio = is_special ? (Speed_Ratio * Gear_Ratio) : Speed_Ratio;
        
        cmd.q = (dds_low_command.motor_cmd().at(motorID).q() + Startq[motorID]) * ratio;
        cmd.dq = dds_low_command.motor_cmd().at(motorID).dq() * ratio;
    }

    void ParseMotorFeedback(MotorData& data, int motorID) {
        const bool is_special = IsSpecialMotor(motorID);
        const float ratio = is_special ? (Speed_Ratio * Gear_Ratio) : Speed_Ratio;
        
        allMotorData.at(motorID).q = data.q / ratio - Startq[motorID];
        allMotorData.at(motorID).dq = data.dq / ratio;
    }

    const std::array<MotorData, 10> &GetData() const {

        return allMotorData;
    }


        /// 从标定文件加载 Startq；文件不存在或格式错误时保留当前(兜底默认)值。
    void LoadStartq(const std::string &path) {
        try {
            YAML::Node node = YAML::LoadFile(path);
            auto vec = node["startq"].as<std::vector<float>>();
            if (vec.size() != Startq.size()) {
                std::cerr << "[MotorController] " << path << " startq size mismatch, expected "
                          << Startq.size() << " got " << vec.size() << ". Keeping previous values." << std::endl;
                return;
            }
            std::copy(vec.begin(), vec.end(), Startq.begin());
            std::cout << "[MotorController] Loaded Startq from " << path << std::endl;
        } catch (const std::exception &e) {
            std::cerr << "[MotorController] Failed to load " << path << ": " << e.what()
                      << ". Using built-in default Startq." << std::endl;
        }
    }

    /// 把当前 Startq 写回标定文件，供下次启动直接加载。
    void SaveStartq(const std::string &path) const {
        YAML::Emitter out;
        out << YAML::Comment("Motor zero-position offsets (per-robot calibration).");
        out << YAML::Comment("Generated by the calibrate_motor tool - do not hand-edit unless you know what you're doing.");
        out << YAML::BeginMap;
        out << YAML::Key << "startq" << YAML::Value << YAML::Flow
            << std::vector<float>(Startq.begin(), Startq.end());
        out << YAML::EndMap;
        std::ofstream fout(path);
        fout << out.c_str() << std::endl;
    }

    /// 标定流程：机器人被摆放到"零位参考姿态"并保持静止后调用。
    /// 采样 sampleCount 次当前(旧 Startq 下的)逻辑关节角并取平均，
    /// 作为该姿态相对旧零位的偏差，叠加进 Startq，使这个姿态之后读数为 0，
    /// 然后把新的 Startq 落盘到 path。
    bool Calibrate(const std::string &path, int sampleCount = 200, int sampleIntervalMs = 5) {
        std::array<double, 10> sum{};
        sum.fill(0.0);
        for (int s = 0; s < sampleCount; ++s) {
            for (int i = 0; i < 10; ++i) sum[i] += allMotorData[i].q;
            std::this_thread::sleep_for(std::chrono::milliseconds(sampleIntervalMs));
        }
        for (int i = 0; i < 10; ++i) {
            float avg_logical_q = static_cast<float>(sum[i] / sampleCount);
            Startq[i] += avg_logical_q;
        }
        SaveStartq(path);
        return true;
    }
};
