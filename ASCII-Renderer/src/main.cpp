#include "ASCIIRenderer.cpp"

using namespace std;

int main() {
    ASCIIRenderer renderer;
    std::string modelPath = "data/teapot.obj";

    try {
        renderer.renderLoop(modelPath);
    }
    catch (const std::exception& e) {
        std::cerr << "Exception: " << e.what() << std::endl;
    }

    return 0;
}