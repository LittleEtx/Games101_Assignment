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
    template<typename T>
    inline T clamp(T x, T a, T b)
    {
        return x < a ? a : (x > b ? b : x);
    }
public:
    Texture(const std::string& name)
    {
        image_data = cv::imread(name);
        cv::cvtColor(image_data, image_data, cv::COLOR_RGB2BGR);
        width = image_data.cols;
        height = image_data.rows;
    }

    int width, height;

    Eigen::Vector3f getColor(float u, float v)
    {
        auto u_img = u * width;
        auto v_img = (1 - v) * height;
        auto color = image_data.at<cv::Vec3b>(v_img, u_img);
        return Eigen::Vector3f(color[0], color[1], color[2]);
    }

    Eigen::Vector3f getColorBilinear(float u, float v)
    {
        auto u_img = u * width - 0.5f;
        auto v_img = (1 - v) * height - 0.5f;
        auto u_min = clamp((int)std::floor(u_img), 0, width - 1);
        auto v_min = clamp((int)std::floor(v_img), 0, height - 1);
        auto u_max = clamp((int)std::ceil(u_img), 0, width - 1);
        auto v_max = clamp((int)std::ceil(v_img), 0, height - 1);
        auto u_ratio = u_img - u_min;
        auto v_ratio = v_img - v_min;
        auto color1 = image_data.at<cv::Vec3b>(v_min, u_min) * (1 - u_ratio) + image_data.at<cv::Vec3b>(v_min, u_max) * u_ratio;
        auto color2 = image_data.at<cv::Vec3b>(v_max, u_min) * (1 - u_ratio) + image_data.at<cv::Vec3b>(v_max, u_max) * u_ratio;
        auto color = color1 * (1 - v_ratio) + color2 * v_ratio;
        return Eigen::Vector3f(color[0], color[1], color[2]);
    }

};
#endif //RASTERIZER_TEXTURE_H
