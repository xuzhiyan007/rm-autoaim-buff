#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>
#include <vector>
#include <cmath>

// 全局状态
static cv::Mat g_src;
static int g_img_w = 0;
static int g_img_h = 0;

// 滑动条变量（必须在回调中可访问，用全局或 static）
static int g_cx = 720;    // 中心 x
static int g_cy = 540;    // 中心 y
static int g_rw = 400;    // ROI 宽
static int g_rh = 400;    // ROI 高
static int g_thresh = 128;// 二值化阈值
static int g_dk = 5;      // 膨胀核尺寸

static const std::string WIN_CONTROL = "Control";
static const std::string WIN_ROI = "ROI";
static const std::string WIN_V = "V Channel";
static const std::string WIN_BIN = "Binary Dilate";

static void clamp_roi(int& x, int& y, int& w, int& h)
{
    if (w < 10) w = 10;
    if (h < 10) h = 10;
    if (w > g_img_w) w = g_img_w;
    if (h > g_img_h) h = g_img_h;

    int x0 = x - w / 2;
    int y0 = y - h / 2;
    int x1 = x0 + w;
    int y1 = y0 + h;

    if (x0 < 0) { x1 -= x0; x0 = 0; }
    if (y0 < 0) { y1 -= y0; y0 = 0; }
    if (x1 > g_img_w) { x0 -= (x1 - g_img_w); x1 = g_img_w; }
    if (y1 > g_img_h) { y0 -= (y1 - g_img_h); y1 = g_img_h; }

    if (x0 < 0) x0 = 0;
    if (y0 < 0) y0 = 0;

    w = x1 - x0;
    h = y1 - y0;
    x = x0 + w / 2;
    y = y0 + h / 2;
}

static void on_trackbar(int, void*)
{
    if (g_src.empty()) return;

    int cx = g_cx;
    int cy = g_cy;
    int rw = g_rw;
    int rh = g_rh;
    clamp_roi(cx, cy, rw, rh);

    int x0 = cx - rw / 2;
    int y0 = cy - rh / 2;

    // 实际 ROI
    cv::Rect roi_rect(x0, y0, rw, rh);
    cv::Mat roi = g_src(roi_rect);

    // 2. HSV 提取 V 通道
    cv::Mat hsv;
    cv::cvtColor(roi, hsv, cv::COLOR_BGR2HSV);
    std::vector<cv::Mat> channels;
    cv::split(hsv, channels);
    cv::Mat v_chan = channels[2];

    // 3. 二值化
    cv::Mat binary;
    cv::threshold(v_chan, binary, g_thresh, 255, cv::THRESH_BINARY);

    // 4. 圆形膨胀核膨胀
    int k = g_dk;
    if (k < 1) k = 1;
    if (k % 2 == 0) k += 1; // 保证奇数
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(k, k));
    cv::Mat dilated;
    cv::dilate(binary, dilated, kernel);
    cv::imshow(WIN_BIN, dilated);

    // 5. 轮廓提取
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(dilated, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    // 6. 计算哪个轮廓更像圆形
    double best_circ = -1.0;
    size_t best_idx = 0;
    const double min_area = 50.0; // 过滤极小噪声
    for (size_t i = 0; i < contours.size(); ++i) {
        double area = cv::contourArea(contours[i]);
        if (area < min_area) continue;
        double peri = cv::arcLength(contours[i], true);
        if (peri < 1e-6) continue;
        double circ = (4.0 * CV_PI * area) / (peri * peri);
        if (circ > best_circ) {
            best_circ = circ;
            best_idx = i;
        }
    }

    // 1. ROI 截取可视化（在整图上画矩形框 + 检测结果标注）
    cv::Mat vis_roi = g_src.clone();
    cv::rectangle(vis_roi, cv::Point(x0, y0), cv::Point(x0 + rw, y0 + rh), cv::Scalar(0, 255, 0), 2);
    cv::circle(vis_roi, cv::Point(cx, cy), 3, cv::Scalar(0, 0, 255), -1);
    cv::putText(vis_roi, "ROI", cv::Point(x0, y0 - 5), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);

    // 如果检测到有效轮廓，在原图中标注近似圆的圆心和外接圆
    if (!contours.empty() && best_circ >= 0.0) {
        cv::Point2f roi_center;
        float roi_radius;
        cv::minEnclosingCircle(contours[best_idx], roi_center, roi_radius);
        cv::Point2f img_center(roi_center.x + x0, roi_center.y + y0);
        cv::circle(vis_roi, img_center, static_cast<int>(roi_radius), cv::Scalar(255, 0, 0), 2);
        cv::circle(vis_roi, img_center, 6, cv::Scalar(0, 255, 255), -1); // 圆心黄点
        cv::circle(vis_roi, img_center, 6, cv::Scalar(0, 0, 0), 1);      // 圆心黑边
        std::string txt = cv::format("C=%.3f", best_circ);
        cv::putText(vis_roi, txt, cv::Point(static_cast<int>(img_center.x) + 10, static_cast<int>(img_center.y)),
                    cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 0, 0), 2);
    }

    cv::imshow(WIN_ROI, vis_roi);

    // 7. 在 V 通道可视化图上标注
    cv::Mat v_vis;
    cv::cvtColor(v_chan, v_vis, cv::COLOR_GRAY2BGR);
    if (!contours.empty() && best_circ >= 0.0) {
        // 画出最圆轮廓
        cv::drawContours(v_vis, contours, static_cast<int>(best_idx), cv::Scalar(0, 0, 255), 2);
        // 外接圆
        cv::Point2f center;
        float radius;
        cv::minEnclosingCircle(contours[best_idx], center, radius);
        cv::circle(v_vis, center, static_cast<int>(radius), cv::Scalar(0, 255, 0), 2);
        // 显示圆度
        std::string txt = cv::format("Circularity: %.3f", best_circ);
        cv::putText(v_vis, txt, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
    } else {
        cv::putText(v_vis, "No valid contour", cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 255), 2);
    }
    cv::imshow(WIN_V, v_vis);
}

int main(int argc, char** argv)
{
    std::string img_path = "20260512_1706_little_red_01_0004.jpg";
    if (argc > 1) img_path = argv[1];

    g_src = cv::imread(img_path, cv::IMREAD_COLOR);
    // 如果从 build/ 子目录运行，尝试父目录
    if (g_src.empty() && img_path == "20260512_1706_little_red_01_0004.jpg") {
        g_src = cv::imread("../" + img_path, cv::IMREAD_COLOR);
        if (!g_src.empty()) img_path = "../" + img_path;
    }
    if (g_src.empty()) {
        std::cerr << "Failed to load image: " << img_path << std::endl;
        return -1;
    }
    g_img_w = g_src.cols;
    g_img_h = g_src.rows;

    // 初始化滑动条默认值在图像中心附近
    g_cx = g_img_w / 2;
    g_cy = g_img_h / 2;
    g_rw = g_img_w / 3;
    g_rh = g_img_h / 3;

    cv::namedWindow(WIN_CONTROL, cv::WINDOW_AUTOSIZE);
    cv::namedWindow(WIN_ROI, cv::WINDOW_AUTOSIZE);
    cv::namedWindow(WIN_V, cv::WINDOW_AUTOSIZE);
    cv::namedWindow(WIN_BIN, cv::WINDOW_AUTOSIZE);

    cv::createTrackbar("Center X", WIN_CONTROL, &g_cx, g_img_w, on_trackbar);
    cv::createTrackbar("Center Y", WIN_CONTROL, &g_cy, g_img_h, on_trackbar);
    cv::createTrackbar("ROI W",    WIN_CONTROL, &g_rw, g_img_w, on_trackbar);
    cv::createTrackbar("ROI H",    WIN_CONTROL, &g_rh, g_img_h, on_trackbar);
    cv::createTrackbar("Thresh",   WIN_CONTROL, &g_thresh, 255, on_trackbar);
    cv::createTrackbar("Dilate K", WIN_CONTROL, &g_dk, 31, on_trackbar);

    // 首次刷新
    on_trackbar(0, nullptr);

    std::cout << "Usage: adjust trackbars in '" << WIN_CONTROL << "' window." << std::endl;
    std::cout << "Press ESC or 'q' to quit." << std::endl;

    while (true) {
        int key = cv::waitKey(30);
        if (key == 27 || key == 'q' || key == 'Q') break;
    }

    cv::destroyAllWindows();
    return 0;
}
