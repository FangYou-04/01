#ifndef CONFIG_HPP
#define CONFIG_HPP

#include <string>
#include <yaml-cpp/yaml.h>
#include <mutex>
#include <stdexcept>
#include <filesystem>

// YOLO检测核心配置
struct YoloConfig {
    float conf_thred_blue = 0.6f; // 蓝色装甲板置信度阈值
    float conf_thred_red = 0.2f; // 红色装甲板置信度阈值
    float nms_thresh = 0.45f; // NMS阈值
    int imgsz = 320; // 图像尺寸
    std::string model_path; // 模型路径
};

// 相机参数配置
struct CameraConfig {
    double fx = 0.0, fy = 0.0, cx = 0.0, cy = 0.0; // 内参
    double k1 = 0.0, k2 = 0.0, p1 = 0.0, p2 = 0.0, k3 = 0.0; //畸变参数
};

// 装甲板物理尺寸
struct ArmorSizeConfig {
    float width = 0.141f;
    float height = 0.125f;
};

// EKF配置
struct KalmanConfig {
    float processNoisePos = 1e-4f;
    float processNoiseVel = 1e-2f;
    float measurementNoisePos = 1e-4f;
    float measurementNoiseYaw = 0.0001f;
    float initialErrorCov = 0.1f;
};

// 总配置
struct AppConfig {
    YoloConfig yolo_config;
    CameraConfig armor_params;
    ArmorSizeConfig armor_size;
    KalmanConfig kalman;
};

namespace YAML {
    template<> struct convert<YoloConfig> {
        static Node encode(const YoloConfig& rhs) {
            Node node;
            node["conf_thred_blue"] = rhs.conf_thred_blue;
            node["conf_thred_red"] = rhs.conf_thred_red;
            node["nms_thresh"] = rhs.nms_thresh;
            node["imgsz"] = rhs.imgsz;
            node["model_path"] = rhs.model_path;
            return node;
        }
        static bool decode(const Node& node, YoloConfig& rhs) {
            if (!node.IsMap()) return false;
            rhs.conf_thred_blue = node["conf_thred_blue"].as<float>(0.6f);
            rhs.conf_thred_red = node["conf_thred_red"].as<float>(0.2f);
            rhs.nms_thresh = node["nms_thresh"].as<float>(0.45f);
            rhs.imgsz = node["imgsz"].as<int>(320);
            rhs.model_path = node["model_path"].as<std::string>("");
            return true;
        }
    };
    template<> struct convert<CameraConfig> {
        static Node encode(const CameraConfig& rhs) {
            Node node;
            node["fx"] = rhs.fx; node["fy"] = rhs.fy;
            node["cx"] = rhs.cx; node["cy"] = rhs.cy;
            node["k1"] = rhs.k1; node["k2"] = rhs.k2;
            node["p1"] = rhs.p1; node["p2"] = rhs.p2;
            node["k3"] = rhs.k3;
            return node;
        }
        static bool decode(const Node& node, CameraConfig& rhs) {
            if (!node.IsMap()) return false;
            rhs.fx = node["fx"].as<double>(0.0);
            rhs.fy = node["fy"].as<double>(0.0);
            rhs.cx = node["cx"].as<double>(0.0);
            rhs.cy = node["cy"].as<double>(0.0);
            rhs.k1 = node["k1"].as<double>(0.0);
            rhs.k2 = node["k2"].as<double>(0.0);
            rhs.p1 = node["p1"].as<double>(0.0);
            rhs.p2 = node["p2"].as<double>(0.0);
            rhs.k3 = node["k3"].as<double>(0.0);
            return true;
        }
    };
    template<> struct convert<ArmorSizeConfig> {
        static Node encode(const ArmorSizeConfig& rhs) {
            Node node;
            node["width"] = rhs.width; node["height"] = rhs.height;
            return node;
        }
        static bool decode(const Node& node, ArmorSizeConfig& rhs) {
            if (!node.IsMap()) return false;
            rhs.width = node["width"].as<float>(0.141f);
            rhs.height = node["height"].as<float>(0.125f);
            return true;
        }
    };
    template<> struct convert<KalmanConfig> {
        static Node encode(const KalmanConfig& rhs) {
            Node node;
            node["processNoisePos"] = rhs.processNoisePos;
            node["processNoiseVel"] = rhs.processNoiseVel;
            node["measurementNoisePos"] = rhs.measurementNoisePos;
            node["measurementNoiseYaw"] = rhs.measurementNoiseYaw;
            node["initialErrorCov"] = rhs.initialErrorCov;
            return node;
        }
        static bool decode(const Node& node, KalmanConfig& rhs) {
            if (!node.IsMap()) return false;
            rhs.processNoisePos = node["processNoisePos"].as<float>(1e-4f);
            rhs.processNoiseVel = node["processNoiseVel"].as<float>(1e-2f);
            rhs.measurementNoisePos = node["measurementNoisePos"].as<float>(1e-4f);
            rhs.measurementNoiseYaw = node["measurementNoiseYaw"].as<float>(0.0001f);
            rhs.initialErrorCov = node["initialErrorCov"].as<float>(0.1f);
            return true;
        }
    };
    template<> struct convert<AppConfig> {
        static Node encode(const AppConfig& rhs) {
            Node node;
            node["yolo_config"] = rhs.yolo_config;
            node["armor_params"] = rhs.armor_params;
            node["armor_size"] = rhs.armor_size;
            node["kalman_config"] = rhs.kalman;
            return node;
        }
        static bool decode(const Node& node, AppConfig& rhs) {
            if (!node.IsMap()) return false;
            rhs.yolo_config = node["yolo_config"].as<YoloConfig>();
            rhs.armor_params = node["armor_params"].as<CameraConfig>();
            rhs.armor_size = node["armor_size"].as<ArmorSizeConfig>();
            rhs.kalman = node["kalman_config"].as<KalmanConfig>();
            return true;
        }
    };
}

class Config {
private:
    AppConfig config_;
    static Config* instance_;
    static std::mutex mutex_;
    std::string config_path_;

    Config() {
        // 默认配置文件路径：可执行文件所在目录下的 config/config.yaml
        std::string exe_path = std::filesystem::current_path().string();
        config_path_ = exe_path + "/config/config.yaml";
        loadConfig();
    }
    explicit Config(const std::string& path) : config_path_(path) { loadConfig(); }
    Config(const Config&) = delete;
    Config& operator=(const Config&) = delete;

    void loadConfig() {
        if (!std::filesystem::exists(config_path_))
            throw std::runtime_error("配置文件不存在: " + config_path_);
        try {
            YAML::Node root = YAML::LoadFile(config_path_);
            config_ = root.as<AppConfig>();
        } catch (const YAML::Exception& e) {
            throw std::runtime_error("配置文件解析失败: " + std::string(e.what()));
        }
    }

public:
    static Config* getInstance(const std::string& path = "") {
        std::lock_guard<std::mutex> lock(mutex_);
        if (instance_ == nullptr) {
            if (path.empty()) instance_ = new Config();
            else instance_ = new Config(path);
        }
        return instance_;
    }
    void reloadConfig(const std::string& new_path = "") {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!new_path.empty()) config_path_ = new_path;
        loadConfig();
    }
    const AppConfig& getConfig() const { return config_; }
    ~Config() = default;
};


#endif // CONFIG_HPP