//
// The code comes with no warranties, use it at your own risk.
// You may use it, or parts of it, wherever you want.
// 
// Author: João Madeiras Pereira
//
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include "renderer.h"
#include "mathUtility.h"
#include "shader.h"

#define STB_RECT_PACK_IMPLEMENTATION
#define STB_TRUETYPE_IMPLEMENTATION

#include "stb_rect_pack.h"
#include "stb_truetype.h"

using namespace std;

Renderer::Renderer() {}

bool Renderer::truetypeInit(const std::string& fontFile) {

    // Read the font file
    ifstream inputStream(fontFile.c_str(), std::ios::binary);

    if (inputStream.fail()) {
        printf("\nError opening font file.\n");
        return(false);
    }

    inputStream.seekg(0, std::ios::end);
    auto&& fontFileSize = inputStream.tellg();
    inputStream.seekg(0, std::ios::beg);

    uint8_t* fontDataBuf = new uint8_t[fontFileSize];

    inputStream.read((char*)fontDataBuf, fontFileSize);

    if (!fontDataBuf) {
        cerr << "Failed to load buffer with font data\n";
        return false;
    }

    if (!stbtt_InitFont(&font.info, fontDataBuf, 0)) {
        cerr << "stbtt_InitFont() Failed!\n";
        return false;
    }

    inputStream.close();

    constexpr auto TEX_SIZE = 1024; //Font atlast width and height
    uint8_t* fontAtlasTextureData = new uint8_t[TEX_SIZE * TEX_SIZE];
    //auto pixels = std::make_unique<uint8_t[]>(TEX_SIZE * TEX_SIZE);

    stbtt_pack_context pack_context;
    if (!stbtt_PackBegin(&pack_context, fontAtlasTextureData, TEX_SIZE, TEX_SIZE, 0, 1, nullptr)) {
        cerr << "Failed to start font packing\n";
        return false;
    }

    font.size = 128.f;
    stbtt_pack_range range{
            font.size,
            32,
            nullptr,
            96,
            font.packedChars
    };

    if (!stbtt_PackFontRanges(&pack_context, fontDataBuf, 0, &range, 1)) {
        cerr << "Failed to pack font ranges\n";
        return false;
    }

    stbtt_PackEnd(&pack_context);

    for (int i = 0; i < 96; i++)
    {
        float unusedX, unusedY;

        stbtt_GetPackedQuad(
            font.packedChars,               // Array of stbtt_packedchar
            TEX_SIZE,                           // Width of the font atlas texture
            TEX_SIZE,                           // Height of the font atlas texture
            i,                            // Index of the glyph
            &unusedX, &unusedY,         // current position of the glyph in screen pixel coordinates, (not required as we have a different coordinate system)
            &font.alignedQuads[i],              // stbtt_alligned_quad struct. (this struct mainly consists of the texture coordinates)
            0                        // Allign X and Y position to a integer (doesn't matter because we are not using 'unusedX' and 'unusedY')
        );
    }

    //each glyph quad texture needs just one color channel: 0 in background and 1 for the actual character pixels. Use it for alpha blending
    //It creates a texture object in TexObjArray for storing the fontAtlasTexture
    TexObjArray.texture2D_Loader(TEX_SIZE, TEX_SIZE, fontAtlasTextureData);
    GLuint texID_loc = TexObjArray.getNumTextureObjects() - 1; //position of font atlas textureObj in the textureArray;
    printf("The texture object #%d stores fontAtlasTexture\n", texID_loc + 1);
    font.textureId = TexObjArray.getTextureId(texID_loc);

    // configure VAO/VBO for char (glyph) texture aligned quads
    //each vertex will have just the Position attribute containing 4 floats: (vec2 pos, vec2 tex)
    // -----------------------------------
    glGenVertexArrays(1, &textVAO);
    glGenBuffers(2, textVBO);
    glBindVertexArray(textVAO);
    glBindBuffer(GL_ARRAY_BUFFER, textVBO[0]);
    glBufferData(GL_ARRAY_BUFFER, sizeof(float) * 16, nullptr, GL_DYNAMIC_DRAW);

    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, 0, 0);

    //index buffer
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, textVBO[1]);
    GLuint quadFaceIndex[] = { 0,1,2,2,3,0 };
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(quadFaceIndex), quadFaceIndex, GL_STATIC_DRAW);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);

    return true;
}


bool Renderer::setRenderMeshesShaderProg(const std::string& vertShaderPath, const std::string& fragShaderPath) {

    // Shader for models
    Shader shader;
    shader.init();
    program = shader.getProgramIndex();
    shader.compileShader(Shader::VERTEX_SHADER, vertShaderPath);
    shader.compileShader(Shader::FRAGMENT_SHADER, fragShaderPath);

    // set semantics for the shader variables
    glBindFragDataLocation(program, 0, "colorOut");
    glBindAttribLocation(program, Shader::VERTEX_COORD_ATTRIB, "position");
    glBindAttribLocation(program, Shader::NORMAL_ATTRIB, "normal");
    glBindAttribLocation(program, Shader::TEXTURE_COORD_ATTRIB, "texCoord");

    glLinkProgram(program);

    printf("InfoLog for Model Shaders and Program\n%s\n\n", shader.getAllInfoLogs().c_str());
    if (!shader.isProgramValid())
        printf("GLSL Model Program Not Valid!\n");

    pvm_loc = glGetUniformLocation(program, "m_pvm");
    vm_loc = glGetUniformLocation(program, "m_viewModel");
    normal_loc = glGetUniformLocation(program, "m_normal");
    texMode_loc = glGetUniformLocation(program, "texMode"); // different modes of texturing
    lpos_loc = glGetUniformLocation(program, "l_pos");
    tex_loc[0] = glGetUniformLocation(program, "texmap");
    tex_loc[1] = glGetUniformLocation(program, "texmap1");
    tex_loc[2] = glGetUniformLocation(program, "texmap2");
    tex_loc[3] = glGetUniformLocation(program, "texmap3");
    tex_loc[4] = glGetUniformLocation(program, "texmap4");
    tex_loc[5] = glGetUniformLocation(program, "texmap5");
    tex_loc[6] = glGetUniformLocation(program, "texmap6");
    tex_loc[7] = glGetUniformLocation(program, "texmap7");
    tex_loc[8] = glGetUniformLocation(program, "texmap8");

    return(shader.isProgramLinked() && shader.isProgramValid());
}
Renderer::~Renderer() {
    glDeleteProgram(program);
    glDeleteProgram(textProgram);
    for (auto& mesh : myMeshes) glDeleteVertexArrays(1, &(mesh.vao));
    myMeshes.clear(); myMeshes.shrink_to_fit();
 
}

bool Renderer::setRenderTextShaderProg(const std::string& vertShaderPath, const std::string& fragShaderPath) {
   
    Shader shader;    // Shader for rendering True Type Font (ttf) bitmap text
    shader.init();
    textProgram = shader.getProgramIndex();
    shader.compileShader(Shader::VERTEX_SHADER, vertShaderPath);
    shader.compileShader(Shader::FRAGMENT_SHADER, fragShaderPath);

    glLinkProgram(textProgram);
    printf("InfoLog for Text Rendering Shader\n%s\n\n", shader.getAllInfoLogs().c_str());

    if (!shader.isProgramValid()) {
        printf("GLSL Text Program Not Valid!\n");
        exit(1);
    }

    fontPvm_loc = glGetUniformLocation(textProgram, "pvm");
    textColor_loc = glGetUniformLocation(textProgram, "textColor");

    // static font atlas texture binding
    glUseProgram(textProgram);
    glActiveTexture(GL_TEXTURE16);
    glBindTexture(GL_TEXTURE_2D, font.textureId);
    glUniform1i(glGetUniformLocation(textProgram, "fontAtlasTexture"), 16);

    return(shader.isProgramLinked() && shader.isProgramValid());
}


void Renderer::activateRenderMeshesShaderProg() {   //GLSL program to draw the meshes
    glUseProgram(program);
}


void Renderer::setSpotParam(float* coneDir, const float cutOff) {
    GLint loc;
    loc = glGetUniformLocation(program, "coneDir");
    glUniform4fv(loc, 1, coneDir);
    loc = glGetUniformLocation(program, "spotCosCutOff");
    glUniform1f(loc, cutOff);
}

void Renderer::setSpotLightMode(bool spotLightMode) {
    GLint loc;
    loc = glGetUniformLocation(program, "spotlight_mode");
    if (spotLightMode)
        glUniform1i(loc, 1);
    else
        glUniform1i(loc, 0);
}

void Renderer::setLightPos(float* lightPos) {
    glUniform4fv(lpos_loc, 1, lightPos);
}

void Renderer::setTexUnit(int tuId, int texObjId) {
    glActiveTexture(GL_TEXTURE0 + tuId);
    glBindTexture(GL_TEXTURE_2D, TexObjArray.getTextureId(texObjId));
    glUniform1i(tex_loc[tuId], tuId);
}

void Renderer::setFogParams(int depthFog, const float fogColor[3], float fogDensity) {
    glUseProgram(program);

    GLint locDepth = glGetUniformLocation(program, "depthFog");
    GLint locColor = glGetUniformLocation(program, "fogColor");
    GLint locDensity = glGetUniformLocation(program, "fogDensity");

    if (locDepth != -1) glUniform1i(locDepth, depthFog);
    if (locColor != -1) glUniform3fv(locColor, 1, fogColor);
    if (locDensity != -1) glUniform1f(locDensity, fogDensity);
}

void Renderer::setDirectionalLight(float* direction, float r, float g, float b, bool on) {
    GLint loc = glGetUniformLocation(program, "directionalLight.direction");
    glUniform3fv(loc, 1, direction);

    loc = glGetUniformLocation(program, "directionalLight.color");
    glUniform3f(loc, r, g, b);

    loc = glGetUniformLocation(program, "directionalLight.on");
    glUniform1i(loc, on ? 1 : 0);
}

void Renderer::setPointLights(const float lightEye[NUMBER_POINT_LIGHTS][4],
    const float lightColor[NUMBER_POINT_LIGHTS][3],
    bool lightsOn) {
    for (int i = 0; i < NUMBER_POINT_LIGHTS; i++) {
        std::string posName = "pointLights[" + std::to_string(i) + "].position";
        GLint locPos = glGetUniformLocation(program, posName.c_str());
        glUniform3f(locPos, lightEye[i][0], lightEye[i][1], lightEye[i][2]);

        std::string colorName = "pointLights[" + std::to_string(i) + "].color";
        GLint locColor = glGetUniformLocation(program, colorName.c_str());
        glUniform3f(locColor, lightColor[i][0], lightColor[i][1], lightColor[i][2]);

        std::string onName = "pointLights[" + std::to_string(i) + "].on";
        GLint locOn = glGetUniformLocation(program, onName.c_str());
        glUniform1i(locOn, lightsOn ? 1 : 0);
    }
}

void Renderer::setSpotLights(const float spotPosEye[][4],
    const float spotDirEye[][4],
    const float spotColor[][3],
    bool spotOn,
    float spotCutOff) {
    glUseProgram(program);

    GLint locCut = glGetUniformLocation(program, "spotCosCutOff");
    if (locCut != -1) glUniform1f(locCut, spotCutOff);

    for (int i = 0; i < NUMBER_SPOT_LIGHTS; ++i) {
        std::string base = "spotLights[" + std::to_string(i) + "]";

        GLint locPos = glGetUniformLocation(program, (base + ".position").c_str());
        if (locPos != -1) glUniform3f(locPos, spotPosEye[i][0], spotPosEye[i][1], spotPosEye[i][2]);

        GLint locDir = glGetUniformLocation(program, (base + ".direction").c_str());
        if (locDir != -1) glUniform3f(locDir, spotDirEye[i][0], spotDirEye[i][1], spotDirEye[i][2]);

        GLint locColor = glGetUniformLocation(program, (base + ".color").c_str());
        if (locColor != -1) glUniform3f(locColor, spotColor[i][0], spotColor[i][1], spotColor[i][2]);

        GLint locOn = glGetUniformLocation(program, (base + ".on").c_str());
        if (locOn != -1) glUniform1i(locOn, spotOn ? 1 : 0);
    }
}

void Renderer::renderMesh(const dataMesh& data) {
    GLint loc;

    // be aware to activate previously the Model shader program
    glUniformMatrix4fv(vm_loc, 1, GL_FALSE, data.vm);
    glUniformMatrix4fv(pvm_loc, 1, GL_FALSE, data.pvm);
    glUniformMatrix3fv(normal_loc, 1, GL_FALSE, data.normal);

    // send the material
    loc = glGetUniformLocation(program, "mat.ambient");
    glUniform4fv(loc, 1, myMeshes[data.meshID].mat.ambient);
    loc = glGetUniformLocation(program, "mat.diffuse");
    glUniform4fv(loc, 1, myMeshes[data.meshID].mat.diffuse);
    loc = glGetUniformLocation(program, "mat.specular");
    glUniform4fv(loc, 1, myMeshes[data.meshID].mat.specular);
    loc = glGetUniformLocation(program, "mat.shininess");
    glUniform1f(loc, myMeshes[data.meshID].mat.shininess);

    // Render mesh
    glUniform1i(texMode_loc, data.texMode);
 


    glBindVertexArray(myMeshes[data.meshID].vao);
    glDrawElements(myMeshes[data.meshID].type, myMeshes[data.meshID].numIndexes, GL_UNSIGNED_INT, nullptr);
    glBindVertexArray(0);
}

void Renderer::renderText(const TextCommand& text) {

    glUseProgram(textProgram);   //use GLSL program for text rendering
    glUniformMatrix4fv(fontPvm_loc, 1, GL_FALSE, text.pvm);
    glUniform4fv(textColor_loc, 1, text.color);
    glBindVertexArray(textVAO);

    float localPosition[2] = { text.position[0], text.position[1] };   //screen coordinates

    for (auto ch : text.str) {

        if (ch > 32 && ch < 127) {
            // Retrieve the data that is used to render a glyph of character 'ch'
            stbtt_packedchar packedChar = font.packedChars[ch - 32];
            stbtt_aligned_quad alignedQuad = font.alignedQuads[ch - 32];

            // The units of the fields of the above structs are in pixels,

            float glyphSize[2] = { (float)(packedChar.x1 - packedChar.x0) * text.size,(float)(packedChar.y1 - packedChar.y0) * text.size };
            float glyphBoundingBoxBottomLeft[2] = { localPosition[0] + (packedChar.xoff * text.size), (localPosition[1] - packedChar.yoff2) * text.size };

            // The order of vertices of a quad goes top-right, top-left, bottom-left, bottom-right
            //each vertex will have just the Position attribute containing 4 floats: (vec2 pos, vec2 tex), in total 16 floats
            float glyphVertices[16] = { glyphBoundingBoxBottomLeft[0] + glyphSize[0], glyphBoundingBoxBottomLeft[1] + glyphSize[1], alignedQuad.s1, alignedQuad.t0,
                glyphBoundingBoxBottomLeft[0], glyphBoundingBoxBottomLeft[1] + glyphSize[1], alignedQuad.s0, alignedQuad.t0,
                glyphBoundingBoxBottomLeft[0], glyphBoundingBoxBottomLeft[1], alignedQuad.s0, alignedQuad.t1,
                glyphBoundingBoxBottomLeft[0] + glyphSize[0], glyphBoundingBoxBottomLeft[1], alignedQuad.s1, alignedQuad.t1 };

            // update content of VBO memory
            glBindBuffer(GL_ARRAY_BUFFER, textVBO[0]);
            glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(glyphVertices), glyphVertices);
            glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);

            // Update the position to render the next glyph specified by packedChar->xadvance.
            localPosition[0] += packedChar.xadvance * text.size;
        }
        // Handle newlines seperately.
        else if (ch == '\n')
        {
            // advance y by fontSize, reset x-coordinate
            localPosition[1] -= 1.0 * font.size * text.size;
            localPosition[0] = text.position[0];
        }
        else if (ch == ' ')
        {
            // advance x by fontSize, keep y-coordinate
            localPosition[0] += 0.2 * font.size * text.size;
        }
    }
}


