#include <iostream>
#include <string>
#include <vector>
#include <filesystem>
#include <algorithm>
#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>
#include <thread>
#include <chrono>

#define M_PI 3.14159265358979323846

class ASCIIRenderer {
private:
    const int WIDTH = 80;
    const int HEIGHT = 40;
    std::vector<std::vector<char>> canvas;
    std::vector<std::vector<float>> depthBuffer;
    float rotationAngle = 0.0f;
    float rotationSpeed = 0.3f;

    const std::string DEPTH_CHARS = " .:-=+*#%@";

    void initCanvas() {
        for (int y = 0; y < HEIGHT; y++) {
                std::fill(canvas[y].begin(), canvas[y].end(), ' ');
                std::fill(depthBuffer[y].begin(), depthBuffer[y].end(), -1e6f);
        }
    }

    std::pair<float, float> project(aiVector3D v) {
        float scale = 1.0f / (v.z + 3.0f);
        return {
            (WIDTH / 2) + v.x * scale * (WIDTH / 2),
            (HEIGHT / 2) - v.y * scale * (HEIGHT / 2)
        };
    }

    char getShade(aiVector3D normal) {
        aiVector3D lightDir = { 0, 0, -1 };
        float intensity = std::max(0.0f, normal * lightDir);

        intensity = pow(intensity, 2.2f);

        int index = static_cast<int>(intensity * (DEPTH_CHARS.length() - 1));
        return DEPTH_CHARS[index];
    }

    void drawLine(int x0, int y0, int x1, int y1, float depth0, float depth1, char shade) {
        int dx = std::abs(x1 - x0);
        int dy = std::abs(y1 - y0);
        int sx = x0 < x1 ? 1 : -1;
        int sy = y0 < y1 ? 1 : -1;
        int err = dx - dy;

        while (true) {
            float t = (dx > dy) ? (std::abs(x0 - x1) / static_cast<float>(dx))
                : (std::abs(y0 - y1) / static_cast<float>(dy));
            float depth = depth0 * (1.0f - t) + depth1 * t;

            if (x0 >= 0 && x0 < WIDTH && y0 >= 0 && y0 < HEIGHT) {
                if (depth > depthBuffer[y0][x0]) {
                    depthBuffer[y0][x0] = depth;
                    canvas[y0][x0] = shade;
                }
            }

            if (x0 == x1 && y0 == y1) break;
            int e2 = 2 * err;
            if (e2 > -dy) { err -= dy; x0 += sx; }
            if (e2 < dx) { err += dx; y0 += sy; }
        }
    }

    void clearConsole() {
        std::cout << "\x1b[H\x1b[J";
    }

    struct Barycentric {
        float alpha, beta, gamma;
    };

    Barycentric computeBarycentric(float x, float y, float x0, float y0, float x1, float y1, float x2, float y2) {
        float denom = (y1 - y2) * (x0 - x2) + (x2 - x1) * (y0 - y2);
        float alpha = ((y1 - y2) * (x - x2) + (x2 - x1) * (y - y2)) / denom;
        float beta = ((y2 - y0) * (x - x2) + (x0 - x2) * (y - y2)) / denom;
        float gamma = 1.0f - alpha - beta;
        return { alpha, beta, gamma };
    }

    void drawTriangle(float x0, float y0, float z0,
        float x1, float y1, float z1,
        float x2, float y2, float z2, char shade) {
        int minX = std::max(0, static_cast<int>(std::min({ x0, x1, x2 })));
        int minY = std::max(0, static_cast<int>(std::min({ y0, y1, y2 })));
        int maxX = std::min(WIDTH - 1, static_cast<int>(std::max({ x0, x1, x2 })));
        int maxY = std::min(HEIGHT - 1, static_cast<int>(std::max({ y0, y1, y2 })));

        for (int y = minY; y <= maxY; y++) {
            for (int x = minX; x <= maxX; x++) {
                Barycentric bc = computeBarycentric(x, y, x0, y0, x1, y1, x2, y2);
                if (bc.alpha >= 0 && bc.beta >= 0 && bc.gamma >= 0) {
                    float depth = bc.alpha * z0 + bc.beta * z1 + bc.gamma * z2;

                    if (depth > depthBuffer[y][x]) {
                        depthBuffer[y][x] = depth;
                        canvas[y][x] = shade;
                    }
                }
            }
        }
    }

    aiVector3D calculateNormal(aiVector3D v0, aiVector3D v1, aiVector3D v2) {
        aiVector3D edge1 = v1 - v0;
        aiVector3D edge2 = v2 - v0;
        aiVector3D normal = edge1 ^ edge2;
        normal.Normalize();
        if (normal.z > 0) {
            normal.x = -normal.x;
            normal.y = -normal.y;
            normal.z = -normal.z;
        }

        return normal;
    }

    void normalizeModel(aiMesh* mesh, float targetSize = 0.067f) {
        float minX = FLT_MAX, maxX = -FLT_MAX;
        float minY = FLT_MAX, maxY = -FLT_MAX;
        float minZ = FLT_MAX, maxZ = -FLT_MAX;

        for (unsigned int i = 0; i < mesh->mNumVertices; i++) {
            aiVector3D v = mesh->mVertices[i];
            minX = std::min(minX, v.x);
            maxX = std::max(maxX, v.x);
            minY = std::min(minY, v.y);
            maxY = std::max(maxY, v.y);
            minZ = std::min(minZ, v.z);
            maxZ = std::max(maxZ, v.z);
        }

        float rangeX = maxX - minX;
        float rangeY = maxY - minY;
        float rangeZ = maxZ - minZ;
        float maxRange = std::max({ rangeX, rangeY, rangeZ });

        float scale = (WIDTH * targetSize) / maxRange;

        float centerX = (minX + maxX) / 2.0f;
        float centerY = (minY + maxY) / 2.0f;
        float centerZ = (minZ + maxZ) / 2.0f;

        for (unsigned int i = 0; i < mesh->mNumVertices; i++) {
            mesh->mVertices[i].x = (mesh->mVertices[i].x - centerX) * scale;
            mesh->mVertices[i].y = (mesh->mVertices[i].y - centerY) * scale;
            mesh->mVertices[i].z = (mesh->mVertices[i].z - centerZ) * scale;
        }
    }

    aiVector3D rotateY(const aiVector3D& v, float angle) {
        float cosA = cos(angle);
        float sinA = sin(angle);
        return {
            v.x * cosA - v.z * sinA,
            v.y,
            v.x * sinA + v.z * cosA
        };
    }


public:
    ASCIIRenderer()
    {
        canvas.resize(HEIGHT, std::vector<char>(WIDTH, ' '));
        depthBuffer.assign(HEIGHT, std::vector<float>(WIDTH, -1e6f));
    }

    void renderLoop(const std::string& modelPath) {
        while (true) {
            renderModel(modelPath);
            clearConsole();
            printCanvas();
            rotationAngle += rotationSpeed;
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }

    void renderModel(const std::string& modelPath) {
        if (!std::filesystem::exists(modelPath)) {
            std::cerr << "Error: The file does not exist - " << modelPath << std::endl;
            return;
        }

        Assimp::Importer importer;

        unsigned int flags =
            aiProcess_Triangulate |
            aiProcess_GenNormals;

        const aiScene* scene = importer.ReadFile(modelPath, flags);

        if (!scene) {
            std::cerr << "Error loading model: " << importer.GetErrorString() << std::endl;
            return;
        }

        initCanvas();

        for (unsigned int m = 0; m < scene->mNumMeshes; m++) {
            aiMesh* mesh = scene->mMeshes[m];

            normalizeModel(mesh);

            for (unsigned int f = 0; f < mesh->mNumFaces; f++) {
                aiFace face = mesh->mFaces[f];

                if (face.mNumIndices != 3) continue;

                float distanceFromCamera = 7.f;
                aiVector3D center(0, 0, 0);

                aiVector3D v0 = rotateY(mesh->mVertices[face.mIndices[0]] - center, rotationAngle) + center;
                aiVector3D v1 = rotateY(mesh->mVertices[face.mIndices[1]] - center, rotationAngle) + center;
                aiVector3D v2 = rotateY(mesh->mVertices[face.mIndices[2]] - center, rotationAngle) + center;

                v0.y = -v0.y;
                v1.y = -v1.y;
                v2.y = -v2.y;

                aiVector3D offset(0, 0, -distanceFromCamera);
                v0 = v0 + offset;
                v1 = v1 + offset;
                v2 = v2 + offset;

                // calcular la normal del triangulo
                aiVector3D normal = calculateNormal(v0, v1, v2);
                // obtener el caracter ASCII segun la iluminacion
                char shade = getShade(normal);

                // proyectar cada vertice
                auto [x0, y0] = project(v0);
                auto [x1, y1] = project(v1);
                auto [x2, y2] = project(v2);

                // convertir a enteros para dibujar en la matriz
                int ix0 = static_cast<int>(x0);
                int iy0 = static_cast<int>(y0);
                int ix1 = static_cast<int>(x1);
                int iy1 = static_cast<int>(y1);
                int ix2 = static_cast<int>(x2);
                int iy2 = static_cast<int>(y2);


                float depth0 = v0.z;
                float depth1 = v1.z;
                float depth2 = v2.z;

                //drawLine(ix0, iy0, ix1, iy1, v0.z, v1.z, shade);
                //drawLine(ix1, iy1, ix2, iy2, v1.z, v2.z, shade);
                //drawLine(ix2, iy2, ix0, iy0, v2.z, v0.z, shade);
                drawTriangle(x0, y0, depth0, x1, y1, depth1, x2, y2, depth2, shade);
            }
        }
    }

    void printCanvas() {
        for (const auto& row : canvas) {
            for (char pixel : row) {
                std::cout << pixel;
            }
            std::cout << std::endl;
        }
    }
};