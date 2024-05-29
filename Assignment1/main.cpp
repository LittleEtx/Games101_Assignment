#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <iostream>
#include <opencv2/opencv.hpp>

constexpr float MY_PI = 3.1415926;

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, -eye_pos[0], 0, 1, 0, -eye_pos[1], 0, 0, 1,
        -eye_pos[2], 0, 0, 0, 1;

    view = translate * view;

    return view;
}

Eigen::Matrix4f get_model_matrix(float roll, float pitch, float yaw)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

    // Create the model matrix for rotating the triangle around the Z axis.
    // Then return it.
    Eigen::Matrix4f rotate_x;
    rotate_x << 1, 0, 0, 0,
                0, std::cos(pitch / 180 * MY_PI), -std::sin(pitch / 180 * MY_PI), 0,
                0, std::sin(pitch / 180 * MY_PI), std::cos(pitch / 180 * MY_PI), 0,
                0, 0, 0, 1;

    Eigen::Matrix4f rotate_y;
    rotate_y << std::cos(yaw / 180 * MY_PI), 0, std::sin(yaw / 180 * MY_PI), 0,
                0, 1, 0, 0,
                -std::sin(yaw / 180 * MY_PI), 0, std::cos(yaw / 180 * MY_PI), 0,
                0, 0, 0, 1;

    Eigen::Matrix4f rotate_z;
    rotate_z << std::cos(roll / 180 * MY_PI), -std::sin(roll / 180 * MY_PI), 0, 0,
                std::sin(roll / 180 * MY_PI), std::cos(roll / 180 * MY_PI), 0, 0,
                0, 0, 1, 0,
                0, 0, 0, 1;

    model = rotate_z * rotate_y * rotate_x * model;

    return model;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar)
{
    // Students will implement this function

    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();

    // Create the projection matrix for the given parameters.
    // Then return it.

    Eigen::Matrix4f perspective;
    perspective << -zNear, 0, 0, 0,
                   0, -zNear, 0, 0,
                   0, 0, -(zNear + zFar), -zNear * zFar,
                   0, 0, 1, 0;

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, 0,
                 0, 1, 0, 0,
                 0, 0, 1, zNear + zFar / 2,
                 0, 0, 0, 1;

    float width = 2 * zNear * std::tan(eye_fov / 2);
    float height = width / aspect_ratio;
    float depth = zFar - zNear;
    Eigen::Matrix4f scale;
    scale << 2 / width, 0, 0, 0,
             0, 2 / height, 0, 0,
             0, 0, 2 / depth, 0,
             0, 0, 0, 1;

    projection = scale * translate * perspective * projection;
    return projection;
}

int main(int argc, const char** argv)
{
    float angle = 0;
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

    rst::rasterizer r(700, 700);

    Eigen::Vector3f eye_pos = {0, 0, 5};

    std::vector<Eigen::Vector3f> pos{{2, 0, -2}, {0, 2, -2}, {-2, 0, -2}};

    std::vector<Eigen::Vector3i> ind{{0, 1, 2}};

    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);

    int key = 0;
    int frame_count = 0;

    if (command_line) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle, 0, 0));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);

        cv::imwrite(filename, image);

        return 0;
    }
    float roll = 0, pitch = 0, yaw = 0;
    while (key != 27) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(roll, pitch, yaw));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);

        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::imshow("image", image);
        key = cv::waitKey(10);

        std::cout << "frame count: " << frame_count++ << ", key pressed: " << key << '\n';

        if (key == 'a') {
            yaw += 10;
        }
        else if (key == 'd') {
            yaw -= 10;
        }
        else if (key == 'w') {
            pitch += 10;
        }
        else if (key == 's') {
            pitch -= 10;
        }
        else if (key == 'q') {
            roll += 10;
        }
        else if (key == 'e') {
            roll -= 10;
        }
    }

    return 0;
}
