//
// Created by goksu on 2/25/20.
//

#include <opencv2/core/mat.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include "Scene.hpp"
#include "Renderer.hpp"


inline float deg2rad(const float& deg) { return deg * M_PI / 180.0; }

const float EPSILON = 0.00001;

// The main render function. This where we iterate over all pixels in the image,
// generate primary rays and cast these rays into the scene. The content of the
// framebuffer is saved to a file.
void Renderer::Render(const Scene& scene)
{
    std::vector<Vector3f> framebuffer(scene.width * scene.height);

    float scale = tan(deg2rad(scene.fov * 0.5));
    float imageAspectRatio = scene.width / (float)scene.height;
    Vector3f eye_pos(-1, 5, 10);
    int m = 0;
    for (uint32_t j = 0; j < scene.height; ++j) {
        for (uint32_t i = 0; i < scene.width; ++i) {
            // generate primary ray direction
            float x = (2 * (i + 0.5f) / (float)scene.width - 1) *
                      imageAspectRatio * scale;
            float y = (1 - 2 * (j + 0.5f) / (float)scene.height) * scale;
            // Find the x and y positions of the current pixel to get the
            // direction vector that passes through it.
            // Also, don't forget to multiply both of them with the variable
            // *scale*, and x (horizontal) variable with the *imageAspectRatio*

            // Don't forget to normalize this direction!
            Vector3f dir = normalize(Vector3f(x, y, -1));
            framebuffer[i + j * scene.width] = scene.castRay(Ray(eye_pos, dir), 0);
        }
        UpdateProgress(j / (float)scene.height);
    }
    UpdateProgress(1.f);

        // use opencv to view the image
    cv::Mat image(scene.height, scene.width, CV_32FC3, framebuffer.data());
    image.convertTo(image, CV_8UC3, 255);
    cv::cvtColor(image, image, cv::COLOR_RGB2BGR);
    cv::imshow("image", image);
    // save image
    cv::imwrite("image.png", image);
    cv::waitKey(0);
}
