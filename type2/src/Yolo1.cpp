#include "Yolo.hpp"
#include <iostream>
#include <fstream>
#include <algorithm>
#include <ctime>

// ====================== YOLOSolver类实现 ======================
// 构造函数：加载模型和类别文件
YOLOSolver::YOLOSolver(const std::string& model_path, const std::string& labels_path) {
    // 1. 加载类别名称
    labels_ = loadLabels(labels_path);
    if (labels_.empty()) {
        throw std::runtime_error("Failed to load labels file: " + labels_path);
    }

    // 2. 加载YOLO模型（纯CPU模式）
    net_ = cv::dnn::readNetFromONNX(model_path);
    if (net_.empty()) {
        throw std::runtime_error("Failed to load YOLO model: " + model_path);
    }

    // 关键：强制使用CPU推理（禁用所有GPU依赖）
    net_.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
    net_.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);

    std::cout << "[INFO] YOLO model loaded successfully (CPU mode)" << std::endl;
}

// 析构函数
YOLOSolver::~YOLOSolver() {
    // OpenCV的Net会自动释放资源，无需额外操作
}

// 加载类别名称（私有方法）
std::vector<std::string> YOLOSolver::loadLabels(const std::string& path) {
    std::vector<std::string> labels;
    std::ifstream file(path);
    if (!file.is_open()) {
        std::cerr << "[ERROR] Cannot open labels file: " << path << std::endl;
        return labels;
    }

    std::string line;
    while (std::getline(file, line)) {
        if (!line.empty()) {
            labels.push_back(line);
        }
    }
    file.close();
    return labels;
}

// 图像预处理（私有方法）
cv::Mat YOLOSolver::preprocessImage(const cv::Mat& img) {
    cv::Mat blob;
    cv::dnn::blobFromImage(
        img,
        blob,
        SCALE,
        cv::Size(INPUT_WIDTH, INPUT_HEIGHT),
        MEAN,
        true,  // 交换RB通道（OpenCV BGR → YOLO RGB）
        false  // 不裁剪，保持比例填充黑边
    );
    return blob;
}

std::vector<DetectionResult> YOLOSolver::parseOutput(
    const cv::Mat& output, 
    const cv::Size& img_size
) {
    std::vector<DetectionResult> results;
    if (output.dims != 3) return results;

    int dim0 = output.size[0];  // batch size (通常为1)
    int dim1 = output.size[1];  // 通道数 = 4 + num_classes
    int dim2 = output.size[2];  // 检测框数量
    int num_classes = dim1 - 4;
    int num_boxes = dim2;

    std::cout << "[INFO] 解析输出: " << dim0 << "x" << dim1 << "x" << dim2
              << ", num_boxes=" << num_boxes << ", num_classes=" << num_classes << std::endl;

    // 遍历每个检测框
    for (int i = 0; i < num_boxes; ++i) {
        // 对于形状 [1, 6, 2100]，每个框的数据在列中连续存储
        const float* data = output.ptr<float>(0, 0, i);
        float cx = data[0];
        float cy = data[1];
        float w = data[2];
        float h = data[3];
        const float* scores = data + 4;

        // 找到最高置信度类别
        float max_conf = 0.0f;
        int best_class = 0;
        for (int c = 0; c < num_classes; ++c) {
            float conf = scores[c];
            if (conf > max_conf) {
                max_conf = conf;
                best_class = c;
            }
        }

        if (max_conf < CONF_THRESHOLD) continue;

        // 转换坐标到原始图像尺寸
        float x = (cx - w / 2) * img_size.width;
        float y = (cy - h / 2) * img_size.height;
        float width = w * img_size.width;
        float height = h * img_size.height;

        x = std::clamp(x, 0.0f, (float)img_size.width - 1);
        y = std::clamp(y, 0.0f, (float)img_size.height - 1);
        width = std::min(width, (float)img_size.width - x);
        height = std::min(height, (float)img_size.height - y);

        DetectionResult res;
        res.class_id = best_class;
        res.class_name = (best_class < (int)labels_.size()) ? labels_[best_class] : "unknown";
        res.confidence = max_conf;
        res.bbox = cv::Rect((int)x, (int)y, (int)width, (int)height);
        results.push_back(res);
    }

    // NMS 非极大值抑制
    std::vector<int> indices;
    std::vector<cv::Rect> bboxes;
    std::vector<float> confidences;
    for (const auto& res : results) {
        bboxes.push_back(res.bbox);
        confidences.push_back(res.confidence);
    }
    cv::dnn::NMSBoxes(bboxes, confidences, CONF_THRESHOLD, NMS_THRESHOLD, indices);

    std::vector<DetectionResult> final_results;
    for (int idx : indices) {
        final_results.push_back(results[idx]);
    }
    return final_results;
}

// 核心方法：检测单张图像
std::vector<DetectionResult> YOLOSolver::detect(const cv::Mat& img) {
    if (img.empty()) {
        throw std::runtime_error("Input image is empty!");
    }

    // 1. 预处理
    cv::Mat blob = preprocessImage(img);
    // 2. 设置输入
    net_.setInput(blob);
    // 3. 推理
    std::vector<cv::Mat> outputs;
    net_.forward(outputs, net_.getUnconnectedOutLayersNames());
    // 4. 解析输出
    return parseOutput(outputs[0], img.size());
}

// 辅助方法：绘制检测结果
void YOLOSolver::drawResults(cv::Mat& img, const std::vector<DetectionResult>& results) {
    // 随机生成类别颜色（固定种子，保证同一类别颜色一致）
    std::vector<cv::Scalar> colors;
    srand(12345); // 固定随机种子
    for (int i = 0; i < 100; ++i) { // 支持最多100类
        int r = rand() % 256;
        int g = rand() % 256;
        int b = rand() % 256;
        colors.push_back(cv::Scalar(b, g, r));
    }

    // 绘制每个检测框
    for (const auto& res : results) {
        // 绘制矩形框
        cv::rectangle(img, res.bbox, colors[res.class_id], 2);
        // 绘制类别+置信度文本
        std::string text = res.class_name + " " + cv::format("%.2f", res.confidence);
        // 文本背景（避免看不清）
        cv::Rect text_rect = cv::Rect(
            res.bbox.x, 
            res.bbox.y - 20, 
            text.length() * 10, 
            20
        );
        cv::rectangle(img, text_rect, colors[res.class_id], -1);
        // 绘制文本
        cv::putText(
            img, 
            text, 
            cv::Point(res.bbox.x, res.bbox.y - 5), 
            cv::FONT_HERSHEY_SIMPLEX, 
            0.5, 
            cv::Scalar(255, 255, 255), 
            1
        );
    }
}

// ====================== 主函数（测试入口）======================
// int main(int argc, char** argv) {
//     try {
//         // 1. 检查参数
//         if (argc < 3) {
//             std::cerr << "Usage: " << argv[0] << " <model_path> <labels_path> [image_path/camera]" << std::endl;
//             std::cerr << "Example: " << std::endl;
//             std::cerr << "  ./yolo yolov8n.onnx coco.names test.jpg  (检测单张图像)" << std::endl;
//             std::cerr << "  ./yolo yolov8n.onnx coco.names camera    (摄像头实时检测)" << std::endl;
//             return -1;
//         }

//         // 2. 初始化YOLO求解器
//         std::string model_path = argv[1];
//         std::string labels_path = argv[2];
//         YOLOSolver yolo(model_path, labels_path);

//         // 3. 选择检测模式（图像/摄像头）
//         std::string mode = (argc > 3) ? argv[3] : "camera";
//         cv::Mat frame;
//         cv::VideoCapture cap;

//         if (mode == "camera") {
//             // 摄像头模式
//             cap.open(0);
//             if (!cap.isOpened()) {
//                 throw std::runtime_error("Failed to open camera!");
//             }
//             std::cout << "[INFO] Camera opened, press ESC to exit..." << std::endl;

//             while (true) {
//                 cap >> frame;
//                 if (frame.empty()) break;

//                 // 检测 + 绘制
//                 auto results = yolo.detect(frame);
//                 yolo.drawResults(frame, results);

//                 // 显示
//                 cv::imshow("YOLO CPU Detection", frame);
//                 if (cv::waitKey(1) == 27) break; // ESC退出
//             }
//             cap.release();
//         } else {
//             // 图像模式
//             frame = cv::imread(mode);
//             if (frame.empty()) {
//                 throw std::runtime_error("Failed to read image: " + mode);
//             }

//             // 检测 + 绘制 + 保存结果
//             auto results = yolo.detect(frame);
//             yolo.drawResults(frame, results);
//             cv::imwrite("detection_result.jpg", frame);
//             std::cout << "[INFO] Detection completed! Result saved to detection_result.jpg" << std::endl;

//             // 显示结果
//             cv::imshow("YOLO CPU Detection", frame);
//             cv::waitKey(0);
//         }

//         cv::destroyAllWindows();
//     } catch (const std::exception& e) {
//         std::cerr << "[ERROR] " << e.what() << std::endl;
//         return -1;
//     }

//     return 0;
// }
