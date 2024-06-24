#include <CoreLayer/Math/Math.h>
#include <FunctionLayer/Camera/Pinhole.h>
#include <FunctionLayer/Integrator/Integrator.h>
#include <FunctionLayer/Sampler/Sampler.h>
#include <FunctionLayer/Scene/Scene.h>
#include <FunctionLayer/Texture/Mipmap.h>
#include <ResourceLayer/Factory.h>
#include <ResourceLayer/FileUtil.h>
#include <ResourceLayer/Image.h>
#include <ResourceLayer/JsonUtil.h>
#include <chrono>
#include <fstream>
#include <memory>
#include <mutex>
#include <queue>
#include <regex>
#include <stdio.h>
#include <thread>

#define PBSTR "||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||"
#define PBWIDTH 60

std::mutex sampler_mtx;
// #define PARALLEL

#ifdef PARALLEL
#define T_NUM 16
std::queue<std::thread> T_queue;
#endif

inline void printProgress(float percentage) {
    int val = (int)(percentage * 100);
    int lpad = (int)(percentage * PBWIDTH);
    int rpad = PBWIDTH - lpad;
    printf("\r%3d%% [%.*s%*s]", val, lpad, PBSTR, rpad, "");
    fflush(stdout);
}

void parallelRendering(int &x, int &width, int &y, int &height, int &spp,
                       std::shared_ptr<Camera> &camera,
                       std::shared_ptr<Sampler> &sampler,
                       std::shared_ptr<Integrator> &integrator,
                       std::shared_ptr<Scene> &scene) {
    Vector2f NDC{(float)x / width, (float)y / height};
    Spectrum li(.0f);
    for (int i = 0; i < spp; ++i) {

        sampler_mtx.lock();
        CameraSample samp = CameraSample{sampler->next2D()};
        Ray ray = camera->sampleRayDifferentials(samp, NDC);
        li += integrator->li(ray, *scene, sampler);
        sampler_mtx.unlock();
    }
    camera->film->deposit({x, y}, li / spp);
}

int main(int argc, char **argv) {
    const std::string sceneDir = std::string(argv[1]);
    FileUtil::setWorkingDirectory(sceneDir);
    std::string sceneJsonPath = FileUtil::getFullPath("scene.json");
    std::ifstream fstm(sceneJsonPath);
    Json json = Json::parse(fstm);
    auto scene = std::make_shared<Scene>(json["scene"]);
    auto integrator = Factory::construct_class<Integrator>(json["integrator"]);
    auto sampler = Factory::construct_class<Sampler>(json["sampler"]);
    auto camera = Factory::construct_class<Camera>(json["camera"]);
    int spp = sampler->xSamples * sampler->ySamples;
    int width = camera->film->size[0], height = camera->film->size[1];
    std::cout << fetchOptional(json["camera"], "type", std::string("Non"))
              << "\n";
    if (fetchOptional(json["camera"], "type", std::string(" ")) ==
        std::string("opticalSystem")) {
        camera->autoFocus(*scene);
    }
    auto start = std::chrono::system_clock::now();

    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
#ifdef PARALLEL
            if (T_queue.size() >= T_NUM) {
                T_queue.front().join();
                T_queue.pop();
            }
            T_queue.push(std::move(std::thread(
                parallelRendering, std::ref(x), std::ref(width), std::ref(y),
                std::ref(height), std::ref(spp), std::ref(camera),
                std::ref(sampler), std::ref(integrator), std::ref(scene))));

#else
            Vector2f NDC{(float)x / width, (float)y / height};
            Spectrum li(.0f);
            for (int i = 0; i < spp; ++i) {
                Ray ray = camera->sampleRayDifferentials(
                    CameraSample{sampler->next2D()}, NDC);
                li += integrator->li(ray, *scene, sampler);
            }
            camera->film->deposit({x, y}, li / spp);
#endif
            int finished = x + y * width;
            if (finished % 5 == 0) {
                printProgress((float)finished / (height * width));
            }
        }
    }

#ifdef PARALLEL
    while (!T_queue.empty()) {
        T_queue.front().join();
        T_queue.pop();
    }
#endif

    printProgress(1.f);

    auto end = std::chrono::system_clock::now();

    printf("\nRendering costs %.2fs\n",
           (std::chrono::duration_cast<std::chrono::milliseconds>(end - start))
                   .count() /
               1000.f);

    //* 目前支持输出为png/hdr两种格式
    std::string outputName =
        fetchRequired<std::string>(json["output"], "filename");
    if (std::regex_match(outputName, std::regex("(.*)(\\.png)"))) {
        camera->film->savePNG(outputName.c_str());
    } else if (std::regex_match(outputName, std::regex("(.*)(\\.hdr)"))) {
        camera->film->saveHDR(outputName.c_str());
    } else {
        std::cout << "Only support output as PNG/HDR\n";
    }
}
