#include <chrono>
#include <iostream>
#include <opencv2/opencv.hpp>

std::vector<cv::Point2f> control_points;

constexpr int CONTROL_POINT_COUNT = 4;

void mouse_handler(int event, int x, int y, int flags, void *userdata) 
{
    if (event == cv::EVENT_LBUTTONDOWN && control_points.size() < CONTROL_POINT_COUNT)
    {
        std::cout << "Left button of the mouse is clicked - position (" << x << ", "
        << y << ")" << '\n';
        control_points.emplace_back(x, y);
    }     
}

void naive_bezier(const std::vector<cv::Point2f> &points, cv::Mat &window) 
{
    auto &p_0 = points[0];
    auto &p_1 = points[1];
    auto &p_2 = points[2];
    auto &p_3 = points[3];

    for (double t = 0.0; t <= 1.0; t += 0.001) 
    {
        auto point = std::pow(1 - t, 3) * p_0 + 3 * t * std::pow(1 - t, 2) * p_1 +
                 3 * std::pow(t, 2) * (1 - t) * p_2 + std::pow(t, 3) * p_3;

        window.at<cv::Vec3b>(point.y, point.x)[2] = 255;
    }
}

cv::Point2f recursive_bezier(const std::vector<cv::Point2f> &control_points, float t) 
{
    // Implement de Casteljau's algorithm
    if (control_points.size() == 1) {
        return control_points.front();
    }
    std::vector<cv::Point2f> points;
    points.reserve(control_points.size() - 1);
    for (int i = 0; i < control_points.size() - 1; i++) {
        points.push_back((1 - t) * control_points[i] + t * control_points[i + 1]);
    }
    return recursive_bezier(points, t);
}

void bezier(const std::vector<cv::Point2f> &control_points, cv::Mat &window) 
{
    // Iterate through all t = 0 to t = 1 with small steps, and call de Casteljau's
    // recursive Bezier algorithm.
    auto set = [&window](cv::Point2f point, cv::Vec3b color) {
        if (point.x >= 0 && point.x < window.cols && point.y >= 0 && point.y < window.rows) {
            for (int i = 0; i < 3; i++) {
                window.at<cv::Vec3b>(point.y, point.x)[i] = std::max(window.at<cv::Vec3b>(point.y, point.x)[i], color[i]);
            }
        }
    };
    constexpr int STEP_NUM = 1000;
    for (int i = 0; i < STEP_NUM; i++)
    {
        auto t = i / static_cast<float>(STEP_NUM);
        auto pt = recursive_bezier(control_points, t);
        auto p11 = cv::Point2f(floor(pt.x), floor(pt.y));
        auto p12 = cv::Point2f(floor(pt.x) + 1, floor(pt.y));
        auto p21 = cv::Point2f(floor(pt.x), floor(pt.y) + 1);
        auto p22 = cv::Point2f(floor(pt.x) + 1, floor(pt.y) + 1);
        set(p11, cv::Vec3b(0, 255 * (1 - norm((pt - p11))), 0));
        set(p12, cv::Vec3b(0, 255 * (1 - norm((pt - p12))), 0));
        set(p21, cv::Vec3b(0, 255 * (1 - norm((pt - p21))), 0));
        set(p22, cv::Vec3b(0, 255 * (1 - norm((pt - p22))), 0));
//        set(p11, cv::Vec3b(0, 0, 255));
    }

}

int main() 
{
    cv::Mat window = cv::Mat(700, 700, CV_8UC3, cv::Scalar(0));
    cv::cvtColor(window, window, cv::COLOR_BGR2RGB);
    cv::namedWindow("Bezier Curve", cv::WINDOW_AUTOSIZE);

    cv::setMouseCallback("Bezier Curve", mouse_handler, nullptr);

    int key = -1;
    while (key != 27) 
    {
        for (auto &point : control_points) 
        {
            cv::circle(window, point, 3, {255, 255, 255}, 3);
        }

        if (control_points.size() == CONTROL_POINT_COUNT)
        {
//            naive_bezier(control_points, window);
            bezier(control_points, window);

            cv::imshow("Bezier Curve", window);
            cv::imwrite("my_bezier_curve.png", window);
            key = cv::waitKey(0);

            return 0;
        }

        cv::imshow("Bezier Curve", window);
        key = cv::waitKey(20);
    }

return 0;
}
