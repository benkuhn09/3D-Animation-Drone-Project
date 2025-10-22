// meshFromAssimp.h
#pragma once

#include <vector>
#include <string>
#include <GL/glew.h>

#include "model.h"

namespace Assimp { class Importer; }
struct aiScene;

extern char model_dir[50];


bool Import3DFromFile(const std::string& pFile,
    Assimp::Importer& importer,
    const aiScene*& scene,
    float& scaleFactor);

std::vector<MyMesh> createMeshFromAssimp(const aiScene*& sc, GLuint*& textureIds);