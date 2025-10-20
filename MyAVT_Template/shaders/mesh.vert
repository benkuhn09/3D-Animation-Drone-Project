#version 430

uniform mat4 m_pvm;
uniform mat4 m_viewModel;
//uniform mat4 m_viewModelInv;
uniform mat3 m_normal;

in vec4 position;
in vec4 normal;
in vec4 texCoord;
in vec4 tangent;

out Data {
    vec3 normal;
    vec3 eye;
    vec3 fragPos;   // view-space position (xyz)
    vec2 tex_coord;
    vec4 posEye;    // preserve for fog (eye-space vec4)
    vec3 worldDir;
    vec3 T;
    vec3 B;
    vec3 N;
} DataOut;

uniform int texMode; 

void main () {
    // transform to view (eye) space
    vec4 pos = m_viewModel * position;

    vec3 Nview = normalize(m_normal * normal.xyz);

    DataOut.normal    = Nview;
    DataOut.eye       = vec3(-pos);
    DataOut.fragPos   = pos.xyz;
    DataOut.tex_coord = texCoord.st;
    DataOut.posEye    = pos;
    DataOut.worldDir  = normalize(pos.xyz);

    if (texMode == 14) {
        vec3 Tview = normalize(m_normal * tangent.xyz);
        vec3 Bview = tangent.w * normalize(cross(Nview, Tview)); // handedness from tangent.w
        DataOut.T = Tview;
        DataOut.B = Bview;
        DataOut.N = Nview;
    } else {
        DataOut.T = vec3(1,0,0);
        DataOut.B = vec3(0,1,0);
        DataOut.N = Nview;
    }

    gl_Position = m_pvm * position;
}
