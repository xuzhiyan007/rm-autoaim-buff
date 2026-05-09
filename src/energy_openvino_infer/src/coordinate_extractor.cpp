/**
 * coordinate_extractor.cpp
 *
 * 图片坐标拾取工具（双模式）
 *
 * 启动时选择模式：
 *   按 1  截取模板匹配区域
 *         - 从 raw_image/ 读取图片
 *         - 右键依次选取两个角点，自动预览矩形边框
 *         - 第三次右键重置，重新选起点
 *         - Enter：输入文件名，截图保存到 template/
 *   按 2  标记关键点（原功能）
 *         - 从 template/ 读取图片
 *         - 右键标记关键点，Enter 输出坐标列表
 *
 * 通用操作：
 *   左键单击  以点击处为中心放大 ×1.5
 *   左键拖拽  平移图片
 *   + / =    以当前显示中心放大
 *   -        以当前显示中心缩小
 *   n / →    下一张图片
 *   p / ←    上一张图片
 *   z        撤回上一个点
 *   q / ESC  退出
 *
 * 用法：
 *   ./coordinate_extractor [raw_image目录] [template目录]
 */

#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <string>
#include <algorithm>
#include <sstream>
#include <filesystem>
#include <cmath>

namespace fs = std::filesystem;

// ─────────────────────────────────────────────
//  常量
// ─────────────────────────────────────────────
static constexpr int    OUT_W       = 1200;
static constexpr int    OUT_H       = 800;
static constexpr double ZOOM_CLICK  = 1.5;
static constexpr double ZOOM_KEY    = 1.15;
static constexpr double SCALE_MIN   = 0.02;
static constexpr double SCALE_MAX   = 32.0;
static constexpr int    DRAG_THRESH = 5;

// ─────────────────────────────────────────────
//  模式枚举
// ─────────────────────────────────────────────
enum class AppMode { CROP_TEMPLATE, MARK_KEYPOINTS };

// ─────────────────────────────────────────────
//  全局应用状态
// ─────────────────────────────────────────────
struct AppState {
    AppMode     mode          = AppMode::MARK_KEYPOINTS;
    std::string raw_image_dir;
    std::string template_dir;

    std::vector<std::string>  image_paths;
    int                       current_index = 0;

    double      scale       = 1.0;
    cv::Point2d view_center = {0.0, 0.0};

    // Mode 1: 最多 2 个角点；Mode 2: 无限关键点
    std::vector<cv::Point2f> selected_points;

    cv::Mat     original_image;
    std::string window_name = "Coordinate Extractor";

    // 拖拽状态
    bool        dragging           = false;
    bool        drag_moved         = false;
    cv::Point   drag_start_screen  = {0, 0};
    cv::Point2d drag_start_center  = {0.0, 0.0};
};

static AppState g_state;

// ─────────────────────────────────────────────
//  坐标转换
// ─────────────────────────────────────────────

// 屏幕坐标 → 原始图像坐标
static cv::Point2d screen_to_orig(const AppState& s, int sx, int sy)
{
    double tx = OUT_W / 2.0 - s.view_center.x * s.scale;
    double ty = OUT_H / 2.0 - s.view_center.y * s.scale;
    return { (sx - tx) / s.scale, (sy - ty) / s.scale };
}

// 原始图像坐标 → 屏幕坐标
static cv::Point orig_to_screen(const AppState& s, double ox, double oy)
{
    double tx = OUT_W / 2.0 - s.view_center.x * s.scale;
    double ty = OUT_H / 2.0 - s.view_center.y * s.scale;
    return { static_cast<int>(std::round(ox * s.scale + tx)),
             static_cast<int>(std::round(oy * s.scale + ty)) };
}

// ─────────────────────────────────────────────
//  辅助：带描边文字
// ─────────────────────────────────────────────
static void draw_text_outline(cv::Mat& img,
                               const std::string& text,
                               cv::Point origin,
                               double font_scale,
                               cv::Scalar color,
                               int thickness = 1)
{
    cv::putText(img, text, origin, cv::FONT_HERSHEY_SIMPLEX,
                font_scale, cv::Scalar(0, 0, 0), thickness + 2, cv::LINE_AA);
    cv::putText(img, text, origin, cv::FONT_HERSHEY_SIMPLEX,
                font_scale, color, thickness, cv::LINE_AA);
}

// ─────────────────────────────────────────────
//  渲染
// ─────────────────────────────────────────────
static void render(AppState& state)
{
    if (state.original_image.empty()) return;

    double sc = state.scale;
    double tx = OUT_W / 2.0 - state.view_center.x * sc;
    double ty = OUT_H / 2.0 - state.view_center.y * sc;

    cv::Mat M = (cv::Mat_<double>(2, 3) << sc, 0, tx, 0, sc, ty);
    cv::Mat canvas(OUT_H, OUT_W, CV_8UC3, cv::Scalar(40, 40, 40));
    cv::warpAffine(state.original_image, canvas, M,
                   cv::Size(OUT_W, OUT_H), cv::INTER_LINEAR,
                   cv::BORDER_CONSTANT, cv::Scalar(40, 40, 40));

    if (state.mode == AppMode::CROP_TEMPLATE) {
        // ── Mode 1: 绘制角点和矩形预览 ──
        for (size_t i = 0; i < state.selected_points.size(); ++i) {
            cv::Point dp = orig_to_screen(state,
                                          state.selected_points[i].x,
                                          state.selected_points[i].y);
            cv::circle(canvas, dp, 7,  cv::Scalar(255, 255, 255), 2, cv::LINE_AA);
            cv::circle(canvas, dp, 5,  cv::Scalar(0,   220,   0), -1, cv::LINE_AA);
            std::string label = (i == 0) ? "P1" : "P2";
            draw_text_outline(canvas, label, dp + cv::Point(10, 5), 0.55,
                              cv::Scalar(0, 255, 0));
        }
        // 两点都选了时绘制矩形
        if (state.selected_points.size() == 2) {
            cv::Point dp1 = orig_to_screen(state,
                                           state.selected_points[0].x,
                                           state.selected_points[0].y);
            cv::Point dp2 = orig_to_screen(state,
                                           state.selected_points[1].x,
                                           state.selected_points[1].y);
            cv::rectangle(canvas, dp1, dp2, cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
        }
    } else {
        // ── Mode 2: 绘制关键点 ──
        for (size_t i = 0; i < state.selected_points.size(); ++i) {
            const cv::Point2f& op = state.selected_points[i];
            cv::Point dp = orig_to_screen(state, op.x, op.y);

            cv::circle(canvas, dp, 6,  cv::Scalar(255, 255, 255), 2, cv::LINE_AA);
            cv::circle(canvas, dp, 4,  cv::Scalar(0,   0,   255), -1, cv::LINE_AA);

            std::ostringstream oss;
            oss << "#" << (i + 1) << " ("
                << static_cast<int>(std::round(op.x)) << ", "
                << static_cast<int>(std::round(op.y)) << ")";

            cv::Point label_pos = dp + cv::Point(9, 5);
            if (label_pos.x + 160 > OUT_W) label_pos.x = dp.x - 170;
            draw_text_outline(canvas, oss.str(), label_pos, 0.45,
                              cv::Scalar(0, 255, 255));
        }
    }

    // ── 顶部状态栏 ──
    {
        std::string img_name =
            fs::path(state.image_paths[state.current_index]).filename().string();
        std::string mode_str = (state.mode == AppMode::CROP_TEMPLATE)
                               ? "[Mode1: 截取模板]" : "[Mode2: 标记关键点]";
        std::ostringstream info;
        info << mode_str
             << "  [" << (state.current_index + 1) << "/" << state.image_paths.size()
             << "]  " << img_name
             << "   Zoom: " << static_cast<int>(std::round(state.scale * 100)) << "%"
             << "   Points: " << state.selected_points.size();
        draw_text_outline(canvas, info.str(), cv::Point(10, 28), 0.48,
                          cv::Scalar(255, 255, 255));
    }

    // ── 底部操作提示栏 ──
    {
        std::string ctrl;
        if (state.mode == AppMode::CROP_TEMPLATE)
            ctrl = "RClick:选角点(max2)  LClick:放大  LDrag:平移  +/-:缩放  n/p:翻页  z:撤回  Enter:截图保存  q:退出";
        else
            ctrl = "RClick:选点  LClick:放大  LDrag:平移  +/-:缩放  n/p:翻页  z:撤回  Enter:输出坐标  q:退出";
        draw_text_outline(canvas, ctrl,
                          cv::Point(10, OUT_H - 10), 0.37,
                          cv::Scalar(220, 220, 220));
    }

    cv::imshow(state.window_name, canvas);
}

// ─────────────────────────────────────────────
//  鼠标回调
// ─────────────────────────────────────────────
static void mouse_callback(int event, int x, int y, int /*flags*/, void* userdata)
{
    AppState* s = static_cast<AppState*>(userdata);

    switch (event) {

    // 左键按下：开始拖拽记录
    case cv::EVENT_LBUTTONDOWN:
        s->dragging           = true;
        s->drag_moved         = false;
        s->drag_start_screen  = { x, y };
        s->drag_start_center  = s->view_center;
        break;

    // 鼠标移动：拖拽平移
    case cv::EVENT_MOUSEMOVE:
        if (s->dragging) {
            int dx = x - s->drag_start_screen.x;
            int dy = y - s->drag_start_screen.y;
            if (std::abs(dx) > DRAG_THRESH || std::abs(dy) > DRAG_THRESH)
                s->drag_moved = true;
            if (s->drag_moved) {
                s->view_center.x = s->drag_start_center.x - dx / s->scale;
                s->view_center.y = s->drag_start_center.y - dy / s->scale;
                render(*s);
            }
        }
        break;

    // 左键释放：若未拖拽则以点击处为中心放大
    case cv::EVENT_LBUTTONUP:
        if (s->dragging && !s->drag_moved) {
            cv::Point2d orig_click = screen_to_orig(*s, x, y);
            double new_scale = std::min(s->scale * ZOOM_CLICK, SCALE_MAX);
            s->view_center.x = orig_click.x + (OUT_W / 2.0 - x) / new_scale;
            s->view_center.y = orig_click.y + (OUT_H / 2.0 - y) / new_scale;
            s->scale = new_scale;
            std::cout << "[放大] " << static_cast<int>(std::round(s->scale * 100)) << "%\n";
            render(*s);
        }
        s->dragging   = false;
        s->drag_moved = false;
        break;

    // 右键按下：标记关键点 / 选取截图角点
    case cv::EVENT_RBUTTONDOWN: {
        cv::Point2d orig = screen_to_orig(*s, x, y);
        float ox = std::max(0.f, std::min(static_cast<float>(orig.x),
                            static_cast<float>(s->original_image.cols - 1)));
        float oy = std::max(0.f, std::min(static_cast<float>(orig.y),
                            static_cast<float>(s->original_image.rows - 1)));

        if (s->mode == AppMode::CROP_TEMPLATE) {
            // 已有 2 点则重置，重新从第 1 点选
            if (s->selected_points.size() >= 2) {
                s->selected_points.clear();
                std::cout << "[重置] 重新选择角点\n";
            }
            s->selected_points.emplace_back(ox, oy);
            std::cout << "[角点 " << s->selected_points.size() << "] ("
                      << static_cast<int>(std::round(ox)) << ", "
                      << static_cast<int>(std::round(oy)) << ")\n";
            if (s->selected_points.size() == 2)
                std::cout << "[提示] 两个角点已选定，按 Enter 截图保存\n";
        } else {
            // Mode 2: 无限添加关键点
            s->selected_points.emplace_back(ox, oy);
            std::cout << "[选点] #" << s->selected_points.size()
                      << " -> (" << static_cast<int>(std::round(ox))
                      << ", "    << static_cast<int>(std::round(oy)) << ")\n";
        }
        render(*s);
        break;
    }

    default:
        break;
    }
}

// ─────────────────────────────────────────────
//  加载图片（自动适配窗口，清空点集）
// ─────────────────────────────────────────────
static bool load_image(AppState& state, int index)
{
    if (index < 0 || index >= static_cast<int>(state.image_paths.size()))
        return false;

    cv::Mat img = cv::imread(state.image_paths[index], cv::IMREAD_COLOR);
    if (img.empty()) {
        std::cerr << "[错误] 无法读取图片: " << state.image_paths[index] << "\n";
        return false;
    }

    state.original_image = img;
    state.current_index  = index;
    state.selected_points.clear();

    double sx = static_cast<double>(OUT_W) / img.cols;
    double sy = static_cast<double>(OUT_H) / img.rows;
    state.scale       = std::min(sx, sy) * 0.95;
    state.view_center = cv::Point2d(img.cols / 2.0, img.rows / 2.0);

    std::cout << "\n[加载] " << fs::path(state.image_paths[index]).filename().string()
              << "  (" << img.cols << "x" << img.rows << ")\n";
    return true;
}

// ─────────────────────────────────────────────
//  扫描图片目录
// ─────────────────────────────────────────────
static bool scan_images(AppState& state, const std::string& dir)
{
    state.image_paths.clear();
    const std::vector<std::string> valid_exts = {
        ".jpg", ".jpeg", ".png", ".bmp", ".tiff", ".tif", ".webp"
    };
    try {
        for (const auto& entry : fs::directory_iterator(dir)) {
            if (!entry.is_regular_file()) continue;
            std::string ext = entry.path().extension().string();
            std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);
            for (const auto& ve : valid_exts) {
                if (ext == ve) {
                    state.image_paths.push_back(entry.path().string());
                    break;
                }
            }
        }
    } catch (const fs::filesystem_error& e) {
        std::cerr << "[错误] 读取目录失败: " << e.what() << "\n";
        return false;
    }
    if (state.image_paths.empty()) {
        std::cerr << "[错误] 目录 " << dir << " 中未找到图片文件\n";
        return false;
    }
    std::sort(state.image_paths.begin(), state.image_paths.end());
    std::cout << "[INFO] 共找到 " << state.image_paths.size() << " 张图片\n";
    return true;
}

// ─────────────────────────────────────────────
//  Mode 2: 输出关键点坐标
// ─────────────────────────────────────────────
static void print_confirmed_points(const AppState& state)
{
    std::cout << "\n╔══════════════ 确认输出 ══════════════╗\n";
    std::cout << "  图片 : "
              << fs::path(state.image_paths[state.current_index]).filename().string()
              << "\n";
    std::cout << "  共选择了 " << state.selected_points.size() << " 个点:\n";
    for (size_t i = 0; i < state.selected_points.size(); ++i) {
        const auto& p = state.selected_points[i];
        std::cout << "    点 " << (i + 1) << " : ("
                  << static_cast<int>(std::round(p.x)) << ", "
                  << static_cast<int>(std::round(p.y)) << ")\n";
    }
    std::cout << "╚══════════════════════════════════════╝\n\n";
}

// ─────────────────────────────────────────────
//  Mode 1: 截图并保存
// ─────────────────────────────────────────────
static void crop_and_save(AppState& state)
{
    if (state.selected_points.size() < 2) {
        std::cout << "[提示] 请先用右键选取两个角点再按 Enter\n";
        return;
    }

    const cv::Point2f& p1 = state.selected_points[0];
    const cv::Point2f& p2 = state.selected_points[1];

    int x1 = static_cast<int>(std::round(std::min(p1.x, p2.x)));
    int y1 = static_cast<int>(std::round(std::min(p1.y, p2.y)));
    int x2 = static_cast<int>(std::round(std::max(p1.x, p2.x)));
    int y2 = static_cast<int>(std::round(std::max(p1.y, p2.y)));

    // 钳制到图像范围
    x1 = std::max(0, x1);
    y1 = std::max(0, y1);
    x2 = std::min(state.original_image.cols - 1, x2);
    y2 = std::min(state.original_image.rows - 1, y2);

    if (x2 <= x1 || y2 <= y1) {
        std::cout << "[错误] 选取区域无效（宽或高为 0）\n";
        return;
    }

    cv::Mat cropped = state.original_image(
        cv::Rect(x1, y1, x2 - x1, y2 - y1)).clone();

    std::cout << "\n截取区域: (" << x1 << ", " << y1
              << ") - (" << x2 << ", " << y2 << ")"
              << "   大小: " << cropped.cols << "x" << cropped.rows << "\n";
    std::cout << "请输入保存文件名（不含扩展名，留空取消）：";
    std::cout.flush();

    std::string filename;
    std::getline(std::cin, filename);

    // 去除首尾空白
    auto ltrim = filename.find_first_not_of(" \t\r\n");
    auto rtrim = filename.find_last_not_of(" \t\r\n");
    if (ltrim == std::string::npos) { filename.clear(); }
    else { filename = filename.substr(ltrim, rtrim - ltrim + 1); }

    if (filename.empty()) {
        std::cout << "[取消] 未保存\n";
        return;
    }

    // 确保目标目录存在
    try { fs::create_directories(state.template_dir); }
    catch (const std::exception& e) {
        std::cerr << "[错误] 创建目录失败: " << e.what() << "\n";
        return;
    }

    std::string save_path = state.template_dir + "/" + filename + ".png";
    if (cv::imwrite(save_path, cropped)) {
        std::cout << "[保存成功] " << save_path
                  << "  (" << cropped.cols << "x" << cropped.rows << ")\n\n";
        // 清空选点，继续下一次截图
        state.selected_points.clear();
        render(state);
    } else {
        std::cerr << "[错误] 保存失败: " << save_path << "\n";
    }
}

// ─────────────────────────────────────────────
//  启动模式选择菜单（OpenCV 小窗口）
// ─────────────────────────────────────────────
static AppMode show_mode_menu()
{
    cv::Mat menu(320, 640, CV_8UC3, cv::Scalar(30, 30, 30));

    draw_text_outline(menu, "Coordinate Extractor",
                      cv::Point(130, 60),  0.85, cv::Scalar(255, 255, 255), 2);
    draw_text_outline(menu, "Press  1  -  Crop Template Region",
                      cv::Point(80,  140), 0.65, cv::Scalar(80, 220, 80));
    draw_text_outline(menu, "             (read from raw_image/)",
                      cv::Point(80,  175), 0.45, cv::Scalar(140, 200, 140));
    draw_text_outline(menu, "Press  2  -  Mark Keypoints",
                      cv::Point(80,  225), 0.65, cv::Scalar(80, 180, 255));
    draw_text_outline(menu, "             (read from template/)",
                      cv::Point(80,  260), 0.45, cv::Scalar(140, 160, 220));
    draw_text_outline(menu, "Press  q  -  Quit",
                      cv::Point(80,  300), 0.50, cv::Scalar(180, 180, 180));

    const std::string menu_win = "Select Mode";
    cv::namedWindow(menu_win, cv::WINDOW_AUTOSIZE);
    cv::imshow(menu_win, menu);

    AppMode result = AppMode::MARK_KEYPOINTS;
    while (true) {
        int key = cv::waitKey(20) & 0xFF;
        if (key == '1') { result = AppMode::CROP_TEMPLATE;  break; }
        if (key == '2') { result = AppMode::MARK_KEYPOINTS; break; }
        if (key == 'q' || key == 27) { cv::destroyAllWindows(); exit(0); }
    }
    cv::destroyWindow(menu_win);
    return result;
}

// ─────────────────────────────────────────────
//  解析目录路径（相对路径优先，否则用绝对路径）
// ─────────────────────────────────────────────
static std::string resolve_dir(const std::string& rel, const std::string& abs_fallback)
{
    if (fs::exists(rel)) return rel;
    return abs_fallback;
}

// ─────────────────────────────────────────────
//  main
// ─────────────────────────────────────────────
int main(int argc, char* argv[])
{
    // 默认目录
    g_state.raw_image_dir = resolve_dir(
        "src/energy_openvino_infer/raw_image",
        DEFAULT_RAW_IMAGE_DIR);
    g_state.template_dir  = resolve_dir(
        "src/energy_openvino_infer/template",
        DEFAULT_TEMPLATE_DIR);

    // 命令行覆盖
    if (argc > 1) g_state.raw_image_dir = argv[1];
    if (argc > 2) g_state.template_dir  = argv[2];

    // 显示模式菜单
    g_state.mode = show_mode_menu();

    std::string img_dir = (g_state.mode == AppMode::CROP_TEMPLATE)
                          ? g_state.raw_image_dir
                          : g_state.template_dir;

    std::cout << "\n[INFO] 模式: "
              << (g_state.mode == AppMode::CROP_TEMPLATE
                  ? "截取模板  (raw_image → template)"
                  : "标记关键点  (template)")
              << "\n[INFO] 图片目录: " << img_dir << "\n";

    if (!scan_images(g_state, img_dir)) return 1;

    // 创建主窗口
    cv::namedWindow(g_state.window_name,
                    cv::WINDOW_NORMAL | cv::WINDOW_KEEPRATIO | cv::WINDOW_GUI_EXPANDED);
    cv::resizeWindow(g_state.window_name, OUT_W, OUT_H);
    cv::setMouseCallback(g_state.window_name, mouse_callback, &g_state);

    if (!load_image(g_state, 0)) return 1;
    render(g_state);

    // 主循环
    while (true) {
        int key = cv::waitKey(20) & 0xFF;
        if (key == 255) continue;

        // q / ESC → 退出
        if (key == 'q' || key == 27) {
            std::cout << "[INFO] 退出\n";
            break;
        }

        // n / → → 下一张
        if (key == 'n' || key == 83) {
            if (g_state.current_index + 1 < static_cast<int>(g_state.image_paths.size())) {
                load_image(g_state, g_state.current_index + 1);
                render(g_state);
            } else {
                std::cout << "[提示] 已是最后一张图片\n";
            }
            continue;
        }

        // p / ← → 上一张
        if (key == 'p' || key == 81) {
            if (g_state.current_index - 1 >= 0) {
                load_image(g_state, g_state.current_index - 1);
                render(g_state);
            } else {
                std::cout << "[提示] 已是第一张图片\n";
            }
            continue;
        }

        // + / = → 以显示中心放大（view_center 不变）
        if (key == '+' || key == '=') {
            g_state.scale = std::min(g_state.scale * ZOOM_KEY, SCALE_MAX);
            std::cout << "[缩放] " << static_cast<int>(std::round(g_state.scale * 100)) << "%\n";
            render(g_state);
            continue;
        }

        // - → 以显示中心缩小
        if (key == '-') {
            g_state.scale = std::max(g_state.scale / ZOOM_KEY, SCALE_MIN);
            std::cout << "[缩放] " << static_cast<int>(std::round(g_state.scale * 100)) << "%\n";
            render(g_state);
            continue;
        }

        // z → 撤回上一个点
        if (key == 'z') {
            if (!g_state.selected_points.empty()) {
                const auto removed = g_state.selected_points.back();
                g_state.selected_points.pop_back();
                std::cout << "[撤回] 点 (" << static_cast<int>(std::round(removed.x))
                          << ", "          << static_cast<int>(std::round(removed.y))
                          << ") 已移除，剩余 " << g_state.selected_points.size()
                          << " 个点\n";
                render(g_state);
            } else {
                std::cout << "[提示] 没有可撤回的点\n";
            }
            continue;
        }

        // Enter → 截图保存 / 输出坐标
        if (key == 13 || key == 10) {
            if (g_state.mode == AppMode::CROP_TEMPLATE)
                crop_and_save(g_state);
            else
                print_confirmed_points(g_state);
            continue;
        }
    }

    cv::destroyAllWindows();
    return 0;
}
