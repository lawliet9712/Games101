// clang-format off
//
// Created by goksu on 4/6/19.
//

#include <algorithm>
#include <vector>
#include "rasterizer.hpp"
#include <opencv2/opencv.hpp>
#include <math.h>


rst::pos_buf_id rst::rasterizer::load_positions(const std::vector<Eigen::Vector3f>& positions)
{
    auto id = get_next_id();
    pos_buf.emplace(id, positions);

    return { id };
}

rst::ind_buf_id rst::rasterizer::load_indices(const std::vector<Eigen::Vector3i>& indices)
{
    auto id = get_next_id();
    ind_buf.emplace(id, indices);

    return { id };
}

rst::col_buf_id rst::rasterizer::load_colors(const std::vector<Eigen::Vector3f>& cols)
{
    auto id = get_next_id();
    col_buf.emplace(id, cols);

    return { id };
}

auto to_vec4(const Eigen::Vector3f& v3, float w = 1.0f)
{
    return Vector4f(v3.x(), v3.y(), v3.z(), w);
}

static bool _insideTriangle(Vector3f P, const Vector3f* _v)
{
    Vector3f AB = _v[1] - _v[0];
    Vector3f AP = P - _v[0];
    auto cross1 = AB.cross(AP);

    Vector3f BC = _v[2] - _v[1];
    Vector3f BP = P - _v[1];
    auto cross2 = BC.cross(BP);

    Vector3f CA = _v[0] - _v[2];
    Vector3f CP = P - _v[2];
    auto cross3 = CA.cross(AP);

    /* 相同符号 */
    if ((cross1.z() > 0 && cross2.z() > 0 && cross3.z() > 0) ||
        (cross1.z() < 0 && cross2.z() < 0 && cross3.z() < 0)) {
        return true;
    }

    return false;
}

/* 采样次数 */
static const float sampleTimes = 2;
static const float totalSamplePoint = sampleTimes * sampleTimes;
static bool insideTriangle(float x, float y, const Vector3f* _v)
{
    Vector3f v[3];
    for (int i = 0; i < 3; i++)
        v[i] = { _v[i].x(),_v[i].y(), 1.0 };
    Vector3f f0, f1, f2;
    f0 = v[1].cross(v[0]);
    f1 = v[2].cross(v[1]);
    f2 = v[0].cross(v[2]);
    Vector3f p(x, y, 1.);
    if ((p.dot(f0) * f0.dot(v[2]) > 0) && (p.dot(f1) * f1.dot(v[0]) > 0) && (p.dot(f2) * f2.dot(v[1]) > 0))
        return true;
    return false;
    // TODO : Implement this function to check if the point (x, y) is inside the triangle represented by _v[0], _v[1], _v[2]
    Vector3f P = Eigen::Vector3f(x, y, 1.0f);
    return _insideTriangle(P, _v);

    /*
    float step = 1.0f / sampleTimes * 0.5f;
    float lefttopx = P.x() - 0.5;
    float lefttopy = P.y() + 0.5;

    float insideTrianglePoints = 0;
    for (int i = 1; i <= sampleTimes; i++) {
        for (int j = 1; j <= sampleTimes; j++) {
            Vector3f SamplePoint = Eigen::Vector3f(lefttopx + i * step, lefttopy - j * step, 1.0f);
            if (_insideTriangle(SamplePoint, _v)) {
                insideTrianglePoints += 1;
            }
        }
    }

    return insideTrianglePoints == 0 ? 0 : insideTrianglePoints / totalSamplePoint;
    */
}

static std::tuple<float, float, float> computeBarycentric2D(float x, float y, const Vector3f* v)
{
    float c1 = (x * (v[1].y() - v[2].y()) + (v[2].x() - v[1].x()) * y + v[1].x() * v[2].y() - v[2].x() * v[1].y()) / (v[0].x() * (v[1].y() - v[2].y()) + (v[2].x() - v[1].x()) * v[0].y() + v[1].x() * v[2].y() - v[2].x() * v[1].y());
    float c2 = (x * (v[2].y() - v[0].y()) + (v[0].x() - v[2].x()) * y + v[2].x() * v[0].y() - v[0].x() * v[2].y()) / (v[1].x() * (v[2].y() - v[0].y()) + (v[0].x() - v[2].x()) * v[1].y() + v[2].x() * v[0].y() - v[0].x() * v[2].y());
    float c3 = (x * (v[0].y() - v[1].y()) + (v[1].x() - v[0].x()) * y + v[0].x() * v[1].y() - v[1].x() * v[0].y()) / (v[2].x() * (v[0].y() - v[1].y()) + (v[1].x() - v[0].x()) * v[2].y() + v[0].x() * v[1].y() - v[1].x() * v[0].y());
    return { c1,c2,c3 };
}

void rst::rasterizer::draw(pos_buf_id pos_buffer, ind_buf_id ind_buffer, col_buf_id col_buffer, Primitive type)
{
    auto& buf = pos_buf[pos_buffer.pos_id];
    auto& ind = ind_buf[ind_buffer.ind_id];
    auto& col = col_buf[col_buffer.col_id];

    float f1 = (50 - 0.1) / 2.0;
    float f2 = (50 + 0.1) / 2.0;

    Eigen::Matrix4f mvp = projection * view * model;
    for (auto& i : ind)
    {
        Triangle t;
        Eigen::Vector4f v[] = {
                mvp * to_vec4(buf[i[0]], 1.0f),
                mvp * to_vec4(buf[i[1]], 1.0f),
                mvp * to_vec4(buf[i[2]], 1.0f)
        };
        //Homogeneous division
        for (auto& vec : v) {
            vec /= vec.w();
        }
        //Viewport transformation
        for (auto& vert : v)
        {
            vert.x() = 0.5 * width * (vert.x() + 1.0);
            vert.y() = 0.5 * height * (vert.y() + 1.0);
            vert.z() = -vert.z() * f1 + f2;
        }

        for (int i = 0; i < 3; ++i)
        {
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
        }

        auto col_x = col[i[0]];
        auto col_y = col[i[1]];
        auto col_z = col[i[2]];

        t.setColor(0, col_x[0], col_x[1], col_x[2]);
        t.setColor(1, col_y[0], col_y[1], col_y[2]);
        t.setColor(2, col_z[0], col_z[1], col_z[2]);

        rasterize_triangle(t);
    }
}

//Screen space rasterization
void rst::rasterizer::rasterize_triangle(const Triangle& t) {
    auto v = t.toVector4();

    // TODO : Find out the bounding box of current triangle.
    // iterate through the pixel and find if the current pixel is inside the triangle
    float xmin = FLT_MAX, xmax = FLT_MIN, ymin = FLT_MAX, ymax = FLT_MIN;
    for (const auto& p : v) {
        xmin = p.x() < xmin ? p.x() : xmin;
        xmax = p.x() > xmax ? p.x() : xmax;
        ymin = p.y() < ymin ? p.y() : ymin;
        ymax = p.y() > ymax ? p.y() : ymax;
    }

    std::cout << "minx=" << xmin << " miny=" << ymin << " maxx=" << xmax << " maxy=" << ymax << std::endl;
    // If so, use the following code to get the interpolated z value.
    for (int x = ceil(xmin); x < floor(xmax); x++) {
        for (int y = ceil(ymin); y < floor(ymax); y++) {
            /* 检查是否在三角形内 */
            float grid_width = 1.0f / sampleTimes;
            float grid_p_offset = grid_width * 0.5f;
            // 左上角开始算
            float lefttopx = x - 0.5f;
            float lefttopy = y - 0.5f;
            bool should_set_color = false;
            for (int i = 0; i < sampleTimes; i++) {
                for (int j = 0; j < sampleTimes; j++) {
                    float samplex = lefttopx + i * grid_width + grid_p_offset;
                    float sampley = lefttopy + j * grid_width + grid_p_offset;
                    if (insideTriangle(samplex, sampley, t.v)) {
                        // 计算重心坐标
                        auto Barycentric2D = computeBarycentric2D(samplex, sampley, t.v);
                        float alpha = std::get<0>(Barycentric2D), beta = std::get<1>(Barycentric2D), gamma = std::get<2>(Barycentric2D);
                        float w_reciprocal = 1.0 / (alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                        float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                        z_interpolated *= w_reciprocal;
                        int sample_ind = get_index(x, y) + i * sampleTimes + j;
                        float sample_z = sample_depth_buf[sample_ind];
                        if (sample_z > z_interpolated) {
                            should_set_color = true;
                            sample_depth_buf[sample_ind] = z_interpolated;
                            sample_frame_buf[sample_ind] = t.getColor() / totalSamplePoint;
                        }
                    }
                }
            }
            int ind = get_index(x, y);
            Vector3f final_color = { 0, 0, 0 };
            for (int i = ind; i < ind + totalSamplePoint; i++) {
                final_color += sample_frame_buf[i];
            }
            Eigen::Vector3f point = Eigen::Vector3f(x, y, 1.0f);
            set_pixel(point, final_color);
        }
    }
}

void rst::rasterizer::set_model(const Eigen::Matrix4f& m)
{
    model = m;
}

void rst::rasterizer::set_view(const Eigen::Matrix4f& v)
{
    view = v;
}

void rst::rasterizer::set_projection(const Eigen::Matrix4f& p)
{
    projection = p;
}

void rst::rasterizer::clear(rst::Buffers buff)
{
    if ((buff & rst::Buffers::Color) == rst::Buffers::Color)
    {
        std::fill(frame_buf.begin(), frame_buf.end(), Eigen::Vector3f{ 0, 0, 0 });
    }
    if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth)
    {
        std::fill(depth_buf.begin(), depth_buf.end(), std::numeric_limits<float>::infinity());
    }
    if ((buff & rst::Buffers::SampleColor) == rst::Buffers::SampleColor)
    {
        std::fill(sample_frame_buf.begin(), sample_frame_buf.end(), Eigen::Vector3f{ 0, 0, 0 });
    }
    if ((buff & rst::Buffers::SampleDepth) == rst::Buffers::SampleDepth)
    {
        std::fill(sample_depth_buf.begin(), sample_depth_buf.end(), std::numeric_limits<float>::infinity());
    }
}

rst::rasterizer::rasterizer(int w, int h) : width(w), height(h)
{
    frame_buf.resize(w * h);
    depth_buf.resize(w * h);
    sample_frame_buf.resize(w * h * totalSamplePoint);
    sample_depth_buf.resize(w * h * totalSamplePoint);
}

int rst::rasterizer::get_index(int x, int y)
{
    return (height - 1 - y) * width + x;
}

void rst::rasterizer::set_pixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color)
{
    //old index: auto ind = point.y() + point.x() * width;
    auto ind = (height - 1 - point.y()) * width + point.x();
    frame_buf[ind] = color;

}

// clang-format on