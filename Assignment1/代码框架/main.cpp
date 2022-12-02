#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>

constexpr int IM_SIZE = 512;
constexpr double MY_PI = 3.1415926;

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, -eye_pos[0], 0, 1, 0, -eye_pos[1], 0, 0, 1,
        -eye_pos[2], 0, 0, 0, 1;

    view = translate * view;

    return view;
}

Eigen::Matrix4f get_rotation(Eigen::Vector3f axis, float rotation_angle)
{
    auto radian = rotation_angle * M_PI / 180;
    float sin = sinf(radian);
    float cos = cosf(radian);
    float xx = axis.x() * axis.x();
    float xy = axis.x() * axis.y();
    float xz = axis.x() * axis.z();
    float yy = axis.y() * axis.y();
    float yz = axis.y() * axis.z();
    float zz = axis.z() * axis.z();

    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

    model << xx * (1 - cos) + cos
           , xy * (1 - cos) - axis.z() * sin
           , xz * (1 - cos) + axis.y() * sin
           , 0
           , xy * (1 - cos) + axis.z() * sin
           , yy * (1 - cos) + cos
           , yz * (1 - cos) - axis.x() * sin
           , 0
           , xz * (1 - cos) - axis.y() * sin
           , yz * (1 - cos) + axis.x() * sin
           , zz * (1 - cos) + cos
           , 0
           , 0
           , 0
           , 0
           , 1;
    
    return model;
}

Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the model matrix for rotating the triangle around the Z axis.
    // Then return it.
    auto radian = rotation_angle * M_PI / 180;
    auto sin = sinf(radian);
    auto cos = cosf(radian);
    model << cos, -sin, 0, 0,
             sin,  cos, 0, 0,
               0,    0, 1, 0,
               0,    0, 0, 1;

    return model;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar)
{
    // Students will implement this function

    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the projection matrix for the given parameters.
    // Then return it.

    float zRange = zFar - zNear;
    projection(0, 0) = 1 / (aspect_ratio * tanf(eye_fov / 2));
    projection(1, 1) = 1 / tanf(eye_fov / 2);
    projection(2, 2) = -(zNear + zFar) / zRange;
    projection(2, 3) = -2 * zFar * zNear / zRange;
    projection(3, 2) = -1;
                
    return projection;
}

int main(int argc, const char** argv)
{
    float angle = 0;
    bool auto_rotate = true;
    bool command_line = false;
    std::string filename = "output.png";

    if (argc >= 3) {
        command_line = true;
        angle = std::stof(argv[2]); // -r by default
        if (argc == 4) {
            filename = std::string(argv[3]);
        }
        else
            return 0;
    }

    rst::rasterizer r(IM_SIZE, IM_SIZE);

    Eigen::Vector3f eye_pos = {0, 0, 5};

    std::vector<Eigen::Vector3f> pos{{2, 0, -2}, {0, 2, -2}, {-2, 0, -2}};

    std::vector<Eigen::Vector3i> ind{{0, 1, 2}};

    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);

    int key = 0;
    int frame_count = 0;

    if (command_line) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);
        cv::Mat image(IM_SIZE, IM_SIZE, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);

        cv::imwrite(filename, image);

        return 0;
    }

    while (key != 27) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        // r.set_model(get_model_matrix(angle));
        // Rotate around arbitrary axis.
        Eigen::Vector3f axis{0, 1, 0};
        r.set_model(get_rotation(axis, angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);

        cv::Mat image(IM_SIZE, IM_SIZE, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::imshow("image", image);
        key = cv::waitKey(10);

        // std::cout << "frame count: " << frame_count++ << '\n';

        if (key == ' ') {
            auto_rotate = !auto_rotate;
        }

        if (auto_rotate) {
            angle += 10;
        }
        else {
            if (key == 'a') {
                angle += 10;
            }
            else if (key == 'd') {
                angle -= 10;
            }
        }

    }

    return 0;
}
