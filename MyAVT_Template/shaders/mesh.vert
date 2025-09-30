#version 430

uniform mat4 m_pvm;
uniform mat4 m_viewModel;
uniform mat3 m_normal;

in vec4 position;
in vec4 normal;
in vec4 texCoord;

out Data {
    vec3 normal;
    vec3 eye;
    vec3 fragPos;   // view-space position (xyz)
    vec2 tex_coord;
    vec4 posEye;    // preserve for fog (eye-space vec4)
} DataOut;

void main () {
    // transform to view (eye) space
    vec4 pos = m_viewModel * position;

    DataOut.normal = normalize(m_normal * normal.xyz);
    DataOut.eye = vec3(-pos);            // same as you had
    DataOut.fragPos = pos.xyz;           // new: fragment position in view-space
    DataOut.tex_coord = texCoord.st;
    DataOut.posEye = pos;                // keep this for fog calculations

    gl_Position = m_pvm * position;
}
