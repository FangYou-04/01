#pragma once
#include <string>
#include <yaml-cpp/yaml.h>
#include <mutex>
#include <cstdint>

// 定义与配置相关的结构体
struct MorphConfig
{
    int kernel_size;
};

struct LightConfig
{
    int area;
    double ratio_min;
    double ratio_max;
    int angle;
    float min_fill_ratio; // 新增：最小填充率
};
struct ArmorConfig
{
    double distance_min;
    double distance_max;
    double armor_angle;
    double height_diff;
    double angle_diff;
    double armor_ratio_min;
    double armor_ratio_max;
};

struct KalmanConfig
{
    float processNoisePos;
    float processNoiseVel;
    float measurementNoisePos;
    float measurementNoiseYaw;
    float initialErrorCov;
    float yawnoise;
};

// 在 Congfig.hpp 中添加 UdpConfig 定义（放在其他 Config 结构体附近）
struct UdpConfig
{
    bool enabled = false;
    std::string host = "127.0.0.1";
    int port = 9870;
};

// 在 AppConfig 结构体中增加成员
struct AppConfig
{
    MorphConfig morph_config;
    LightConfig light_config;
    ArmorConfig armor_config;
    KalmanConfig kalman;
    UdpConfig udp;   // 新增
};

namespace YAML
{
    // 定义 YAML 转换函数
    template<>
    struct convert<MorphConfig>
    {
        static Node encode(const MorphConfig& rhs)
        {
            Node node;
            node["kernel_size"] = rhs.kernel_size;
            return node;
        }

        static bool decode(const Node& node, MorphConfig& rhs)
        {
            if (!node.IsMap() || !node["kernel_size"])
                return false;
            rhs.kernel_size = node["kernel_size"].as<int>();
            return true;
        }
    };

    template<>
    struct convert<LightConfig>
    {
        static Node encode(const LightConfig& rhs)
        {
            Node node;
            node["area"] = rhs.area;
            node["ratio_min"] = rhs.ratio_min;
            node["ratio_max"] = rhs.ratio_max;
            node["angle"] = rhs.angle;
            return node;
        }

        static bool decode(const Node& node, LightConfig& rhs)
        {
            if (!node.IsMap() || !node["area"] || !node["ratio_min"] || !node["ratio_max"] || !node["angle"])
                return false;
            rhs.area = node["area"].as<int>();
            rhs.ratio_min = node["ratio_min"].as<double>();
            rhs.ratio_max = node["ratio_max"].as<double>();
            rhs.angle = node["angle"].as<int>();
            return true;
        }
    };

    template<>
    struct convert<ArmorConfig>
    {
        static Node encode(const ArmorConfig& rhs)
        {
            Node node;
            node["distance_min"] = rhs.distance_min;
            node["distance_max"] = rhs.distance_max;
            node["armor_angle"] = rhs.armor_angle;
            node["height_diff"] = rhs.height_diff;
            node["angle_diff"] = rhs.angle_diff;
            node["armor_ratio_min"] = rhs.armor_ratio_min;
            node["armor_ratio_max"] = rhs.armor_ratio_max;
            return node;
        }

        static bool decode(const Node& node, ArmorConfig& rhs)
        {
            if (!node.IsMap() || !node["distance_min"] || !node["distance_max"] || !node["armor_angle"] || !node["height_diff"] || !node["angle_diff"] || !node["armor_ratio_min"] || !node["armor_ratio_max"])
                return false;
            rhs.distance_min = node["distance_min"].as<double>();
            rhs.distance_max = node["distance_max"].as<double>();
            rhs.armor_angle = node["armor_angle"].as<double>();
            rhs.height_diff = node["height_diff"].as<double>();
            rhs.angle_diff = node["angle_diff"].as<double>();
            rhs.armor_ratio_min = node["armor_ratio_min"].as<double>();
            rhs.armor_ratio_max = node["armor_ratio_max"].as<double>();
            return true;
        }
    };  

    template<>
    struct convert<KalmanConfig>
    {
        static Node encode(const KalmanConfig& rhs)
        {
            Node node;
            node["processNoisePos"] = rhs.processNoisePos;
            node["processNoiseVel"] = rhs.processNoiseVel;
            node["measurementNoisePos"] = rhs.measurementNoisePos;
            node["measurementNoiseYaw"] = rhs.measurementNoiseYaw;
            node["initialErrorCov"] = rhs.initialErrorCov;
            node["yawnoise"] = rhs.yawnoise;
            return node;
        }

        static bool decode(const Node& node, KalmanConfig& rhs)
        {
            if(!node.IsMap() || !node["processNoisePos"] || !node["processNoiseVel"] || !node["measurementNoisePos"] || !node["initialErrorCov"])
                return false;
            rhs.processNoisePos = node["processNoisePos"].as<float>();
            rhs.processNoiseVel = node["processNoiseVel"].as<float>();
            rhs.measurementNoisePos = node["measurementNoisePos"].as<float>();
            rhs.measurementNoiseYaw = node["measurementNoiseYaw"].as<float>();
            rhs.initialErrorCov = node["initialErrorCov"].as<float>();
            rhs.yawnoise = node["yawnoise"] ? node["yawnoise"].as<float>() : 0.5f; // 默认值
            return true;
        }
    };

    template<>
    struct convert<UdpConfig>
    {
        static Node encode(const UdpConfig& rhs)
        {
            Node node;
            node["enabled"] = rhs.enabled;
            node["host"] = rhs.host;
            node["port"] = rhs.port;
            return node;
        }

        static bool decode(const Node& node, UdpConfig& rhs)
        {
            if (!node.IsMap() || !node["enabled"] || !node["host"] || !node["port"])
                return false;
            rhs.enabled = node["enabled"].as<bool>();
            rhs.host = node["host"].as<std::string>();
            rhs.port = node["port"].as<int>();
            return true;
        }
    };

    template<>
    struct convert<AppConfig>
    {
        static Node encode(const AppConfig& rhs)
        {            
            Node node;
            node["morph_config"] = rhs.morph_config;
            node["light_config"] = rhs.light_config;
            node["armor_config"] = rhs.armor_config;
            node["kalman"] = rhs.kalman;
            node["udp"] = rhs.udp;
            return node;
        }

        static bool decode(const Node& node, AppConfig& rhs)
        {
            if (!node.IsMap() || !node["morph_config"] || !node["light_config"] || !node["armor_config"])
                return false;
            rhs.morph_config = node["morph_config"].as<MorphConfig>();
            rhs.light_config = node["light_config"].as<LightConfig>();
            rhs.armor_config = node["armor_config"].as<ArmorConfig>();
            rhs.kalman = node["kalman"].as<KalmanConfig>();
            rhs.udp = node["udp"].as<UdpConfig>();
            return true;
        }
    };  
}

class Config
{
private:
    AppConfig config_;
    static Config* instance_;
    static std::mutex mutex_;

    // 构造函数私有化，禁止外部实例化
    Config()
    {
        YAML::Node root = YAML::LoadFile("src/congfig.yaml");
        config_ = root.as<AppConfig>();
    }

    // 禁止拷贝构造和赋值操作
    Config(const Config&) = delete; // 禁止拷贝构造
    Config& operator=(const Config&) = delete; // 禁止赋值操作

public:
    // 全局获取入口（线程安全的单例模式）
    static Config* getInstance()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (instance_ == nullptr)
        {
            instance_ = new Config();
        }
        return instance_;
    }

    // 只读获取配置
    const AppConfig& getConfig() const
    {
        return config_;
    }
};