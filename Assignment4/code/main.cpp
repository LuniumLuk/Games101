#include <chrono>
#include <iostream>
#include <opencv2/opencv.hpp>

std::vector<cv::Point2f> control_points;

void mouse_handler(int event, int x, int y, int flags, void *userdata) 
{
    if (event == cv::EVENT_LBUTTONDOWN && control_points.size() < 4) 
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
    if (control_points.size() == 0)
    {
        return cv::Point2f();
    }
    if (control_points.size() == 1)
    {
        return control_points[0];
    }
    // TODO: Implement de Casteljau's algorithm
    std::vector<cv::Point2f> new_points;
    for (int i = 1; i < control_points.size(); ++i)
    {
        cv::Point2f p(
            control_points[i-1].x * t + control_points[i].x * (1.f - t),
            control_points[i-1].y * t + control_points[i].y * (1.f - t)
        );
        new_points.push_back(p);
    }

    return recursive_bezier(new_points, t);
}

// Convert float value in scene to integer value in image space.
inline int ftoi(float x) {
    // Difference of (int)std::floor(x) and static_cast<int>(x):
    // - (int)std::floor(x) cast x toward -INFINITY
    // - static_cast<int>(x) cast x toward 0
    // Although we ignore negative value in image space, -0.5 will still be cast to 0
    // which is not correct theoretically.
    return std::floor(x);
}

// Convert integer value in image space to float value in scene.
inline float itof(int x) {
    return x + 0.5f;
}

inline float pixel_dist(int x, int y, float px, float py)
{
    float dx = px - itof(x);
    float dy = py - itof(y);
    // return std::min(abs(dx) + abs(dy), 1.f);
    return std::min(sqrtf(dx * dx + dy * dy), 1.f);
}

void bezier(const std::vector<cv::Point2f> &control_points, cv::Mat &window) 
{
    // TODO: Iterate through all t = 0 to t = 1 with small steps, and call de Casteljau's 
    // recursive Bezier algorithm.
    for (double t = 0.0; t <= 1.0; t += 0.001) 
    {
        auto point = recursive_bezier(control_points, t);

        int x0 = ftoi(point.x);
        int y0 = ftoi(point.y);

        window.at<cv::Vec3b>(y0, x0)[1] = std::max((float)window.at<cv::Vec3b>(y0, x0)[1], 255 * (1.f - pixel_dist(x0, y0, point.x, point.y)));
        window.at<cv::Vec3b>(y0, x0 + 1)[1] = std::max((float)window.at<cv::Vec3b>(y0, x0 + 1)[1], 255 * (1.f - pixel_dist(x0 + 1, y0, point.x, point.y)));
        window.at<cv::Vec3b>(y0 + 1, x0)[1] = std::max((float)window.at<cv::Vec3b>(y0 + 1, x0)[1], 255 * (1.f - pixel_dist(x0, y0 + 1, point.x, point.y)));
        window.at<cv::Vec3b>(y0, x0 - 1)[1] = std::max((float)window.at<cv::Vec3b>(y0, x0 - 1)[1], 255 * (1.f - pixel_dist(x0 - 1, y0, point.x, point.y)));
        window.at<cv::Vec3b>(y0 - 1, x0)[1] = std::max((float)window.at<cv::Vec3b>(y0 - 1, x0)[1], 255 * (1.f - pixel_dist(x0, y0 - 1, point.x, point.y)));
        // window.at<cv::Vec3b>(point.y, point.x)[1] = 255;
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

        if (control_points.size() == 4) 
        {
            // naive_bezier(control_points, window);
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
