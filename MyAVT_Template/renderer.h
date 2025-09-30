//
// The code comes with no warranties, use it at your own risk.
// You may use it, or parts of it, wherever you want.
// 
// Author: João Madeiras Pereira
//

#include <vector>
#include "texture.h"
#include "model.h"
#include "stb_truetype.h"

/* Lighting Constants */
constexpr int NUMBER_POINT_LIGHTS = 6;
constexpr int NUMBER_SPOT_LIGHTS = 2;

struct dataMesh {
  int meshID = 0;  //mesh ID in the myMeshes array
  float *pvm, *vm, *normal;  //matrices pointers
  int texMode = 0;  //type of shading-> 0:no texturing; 1:modulate diffuse color with texel color; 2:diffuse color is replaced by texel color; 3: multitexturing
};

enum class Align {
    Left,
    Center,
    Right,
    Top = Right,
    Bottom = Left,
};

struct TextCommand {
    std::string str{};
    float position[2];  //screen coordinates
    float size = 1.f;
    float color[4] = {1.f,1.f,1.f, 1.f};
    float *pvm;
    Align align_x = Align::Center, align_y = Align::Center;
};

class Renderer {
public:
  Renderer();
  ~Renderer();

  bool truetypeInit(const std::string &ttf_filepath);  //Initialization of TRUETYPE  for text rendering

  //Setup render meshes GLSL program
  bool setRenderMeshesShaderProg(const std::string &vertShaderPath, const std::string &fragShaderPath);

    // setup text font rasterizer GLSL program
  bool setRenderTextShaderProg(const std::string &vertShaderPath, const std::string &fragShaderPath);

  void activateRenderMeshesShaderProg();

  void renderMesh(const dataMesh &data);

  void renderText(const TextCommand &text);

  void setLightPos(float *lightPos);

  void setSpotParam(float *coneDir, float cutOff);

  void setSpotLightMode(bool spotLightMode);

  void setTexUnit(int tuId, int texObjId);

  void setFogParams(int depthFog, const float fogColor[3], float fogDensity);

  void setDirectionalLight(float* direction, float r, float g, float b, bool on);

  void setPointLights(const float lightPos[NUMBER_POINT_LIGHTS][4],
      const float lightColor[NUMBER_POINT_LIGHTS][3],
      bool lightsOn);
  
  void setSpotLights(const float spotPosEye[][4],
      const float spotDirEye[][4],
      const float spotColor[][3],
      bool spotOn,
      float spotCutOff);

  //Vector with meshes
  std::vector<struct MyMesh> myMeshes;

  /// Object of class Texture that manage an array of Texture Objects 
  Texture TexObjArray;

private:

  //Render meshes GLSL program
  GLuint program;

  // Text font rasterizer GLSL program
  GLuint textProgram;

  GLint pvm_loc, vm_loc, normal_loc, lpos_loc, texMode_loc;
  GLint tex_loc[MAX_TEXTURES];

  //render font GLSL program variable locations and VAO
  GLint fontPvm_loc, textColor_loc;
  GLuint textVAO, textVBO[2];

    struct Font {
        float size;
        GLuint textureId;    //font atlas texture object ID stored in TexObjArray
        stbtt_fontinfo info;
        stbtt_packedchar packedChars[96];
        stbtt_aligned_quad alignedQuads[96];
    } font{};
};