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

    Eigen::Vector3f getColor(float u, float v)
    {
        auto u_img = u * width;
        auto v_img = (1 - v) * height;
        auto color = image_data.at<cv::Vec3b>(v_img, u_img);
        return Eigen::Vector3f(color[0], color[1], color[2]);
    }

    Eigen::Vector3f getColorBilinear(float u, float v)
    {
        auto u_img = u * width;
        auto v_img = (1 - v) * height;
        float u11 = ceil(u_img);
        float v11 = ceil(v_img);
        float u01 = floor(u_img);
        float v01 = floor(v_img);
        auto color1 = image_data.at<cv::Vec3b>(v01, u11);
        auto color2 = image_data.at<cv::Vec3b>(v01, u01);
        auto color3 = image_data.at<cv::Vec3b>(v11, u11);
        auto color4 = image_data.at<cv::Vec3b>(v11, u01);
        float s = (u_img - u01) / (u11 - u01);
        float t = (v_img - v01) / (v11 - v01);
        auto color5 = color4 + s * (color3 - color4);
        auto color6 = color2 + s * (color1 - color2);
        auto final_color = color6 + t * (color5 - color6);
        return Eigen::Vector3f(final_color[0], final_color[1], final_color[2]);
    }
};
#endif //RASTERIZER_TEXTURE_H
