//
// Created by LEI XU on 4/27/19.
//

#ifndef RASTERIZER_TEXTURE_H
#define RASTERIZER_TEXTURE_H
#include "global.hpp"
#include <eigen3/Eigen/Eigen>
#include <opencv2/opencv.hpp>
class Texture{
private:
    cv::Mat image_data;

public:
    Texture(const std::string& name)
    {
        image_data = cv::imread(name);
        cv::cvtColor(image_data, image_data, cv::COLOR_RGB2BGR);
        width = image_data.cols;
        height = image_data.rows;
    }

    int width, height;

    Eigen::Vector3f getColorBilinear(float u, float v)
    {
        auto u_img = u * width;
        auto v_img = (1 - v) * height;

        int u0 = floor(u_img);
        int u1 = ceil(u_img);
        int v0 = floor(v_img);
        int v1 = ceil(v_img);

        float ua = u_img - (float)u0;
        float va = v_img - (float)v0;

        auto color_00 = image_data.at<cv::Vec3b>(v0, u0);
        auto color_01 = image_data.at<cv::Vec3b>(v1, u0);
        auto color_10 = image_data.at<cv::Vec3b>(v0, u1);
        auto color_11 = image_data.at<cv::Vec3b>(v1, u1);

        Eigen::Vector3f colorf_00 = {color_00[0], color_00[1], color_00[2]};
        Eigen::Vector3f colorf_01 = {color_01[0], color_01[1], color_01[2]};
        Eigen::Vector3f colorf_10 = {color_10[0], color_10[1], color_10[2]};
        Eigen::Vector3f colorf_11 = {color_11[0], color_11[1], color_11[2]};

        return (colorf_00 * (1.f - va) + colorf_01 * va) * (1.f - ua)
             + (colorf_10 * (1.f - va) + colorf_11 * va) * ua;
    }

    Eigen::Vector3f getColor(float u, float v)
    {
        auto u_img = u * width;
        auto v_img = (1 - v) * height;
        auto color = image_data.at<cv::Vec3b>(v_img, u_img);
        return Eigen::Vector3f(color[0], color[1], color[2]);
    }

};
#endif //RASTERIZER_TEXTURE_H
