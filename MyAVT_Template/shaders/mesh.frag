#version 430

#define NUMBER_POINT_LIGHTS 6
#define NUM_SPOT_LIGHTS 2

struct Materials {
    vec4 diffuse;
    vec4 ambient;
    vec4 specular;
    vec4 emissive;
    float shininess;
    int texCount;
};

struct DirectionalLight {
    vec3 direction; // in view space (pointing from surface towards light = -lightDir)
    vec3 color;
    bool on;
};

struct PointLight {
    vec3 position;  // in view space
    vec3 color;
    bool on;
};

struct SpotLight {
    vec3 position;  // in view space
    vec3 direction; // in view space (cone direction)
    vec3 color;
    bool on;
};

in Data {
    vec3 normal;
    vec3 eye;
    vec3 fragPos;
    vec2 tex_coord;
    vec4 posEye;   // for fog
} DataIn;

uniform Materials mat;

uniform sampler2D texmap;
uniform sampler2D texmap1;
uniform sampler2D texmap2;
uniform sampler2D texmap3;
uniform sampler2D texmap4;
uniform sampler2D texmap5;
uniform sampler2D texmap6;
uniform sampler2D texmap7;
uniform sampler2D texmap8;
uniform sampler2D texmap9;

uniform int texMode;

// Lighting uniforms (teammate's system)
uniform DirectionalLight directionalLight;
uniform PointLight pointLights[NUMBER_POINT_LIGHTS];
uniform SpotLight spotLights[NUM_SPOT_LIGHTS];

// Fog uniforms (preserved from your shader)
uniform int depthFog;
uniform vec3 fogColor;
uniform float fogDensity;

out vec4 colorOut;

void main() {
    // Material / shading placeholders
    vec3 n = normalize(DataIn.normal);
    vec3 e = normalize(DataIn.eye);

    vec3 ambient = vec3(0.2);
    vec3 diffuse = vec3(0.0);
    vec3 specular = vec3(0.0);

    // ---------- Directional Light (no attenuation) ----------
    if (directionalLight.on) {
        // teammate uses l = normalize(-direction)
        vec3 l = normalize(-directionalLight.direction);
        float diff = max(dot(n, l), 0.0);
        diffuse += directionalLight.color * diff * mat.diffuse.rgb;

        if (diff > 0.0) {
            vec3 h = normalize(l + e);
            float specFactor = pow(max(dot(h, n), 0.0), mat.shininess);
            specular += directionalLight.color * specFactor * mat.specular.rgb;
        }

        ambient += directionalLight.color * mat.ambient.rgb;
    }

    // ---------- Point Lights (attenuated) ----------
    for (int i = 0; i < NUMBER_POINT_LIGHTS; i++) {
        if (pointLights[i].on) {
            vec3 l = normalize(pointLights[i].position - DataIn.fragPos);
            float diff = max(dot(n, l), 0.0);

            float dist = length(pointLights[i].position - DataIn.fragPos);
            // attenuation constants from teammate: constant=1.0, linear=0.09, quadratic=0.032
            float att = 1.0 / (1.0 + 0.09 * dist + 0.032 * dist * dist);

            diffuse += att * diff * pointLights[i].color * mat.diffuse.rgb;

            if (diff > 0.0) {
                vec3 h = normalize(l + e);
                float specFactor = pow(max(dot(h, n), 0.0), mat.shininess);
                specular += att * specFactor * pointLights[i].color * mat.specular.rgb;
            }

            ambient += att * pointLights[i].color * mat.ambient.rgb;
        }
    }

    // ---------- Spot Lights ----------
    for (int i = 0; i < NUM_SPOT_LIGHTS; i++) {
        if (spotLights[i].on) {
            vec3 l = normalize(spotLights[i].position - DataIn.fragPos);
            float spotCos = dot(-l, normalize(spotLights[i].direction));
            float spotCutoff = 0.85; // tweak from CPU if needed
            if (spotCos > spotCutoff) {
                float dist = length(spotLights[i].position - DataIn.fragPos);
                float att = 1.0 / (1.0 + 0.1 * dist + 0.01 * dist * dist);

                float diff = max(dot(n, l), 0.0);
                diffuse += att * diff * spotLights[i].color * mat.diffuse.rgb;

                if (diff > 0.0) {
                    vec3 h = normalize(l + e);
                    float specFactor = pow(max(dot(h, n), 0.0), mat.shininess);
                    specular += att * specFactor * spotLights[i].color * mat.specular.rgb;
                }

                // small ambient contribution for spots
                ambient += 0.1 * att * spotLights[i].color * mat.ambient.rgb;
            }
        }
    }

    vec3 lighting = ambient + diffuse + specular;

    // ---------- Texture modes (preserved, but use "lighting" for shading) ----------
    vec4 texel, texel1;
    vec4 baseColor;
    float alpha = mat.diffuse.a;

    if (texMode == 0) {
        // no texturing -> use lighting color and material emissive if you want
        baseColor = vec4(lighting, 1.0);
        alpha *= texel.a;
    }
    else if (texMode == 1) { // modulate diffuse with texel
        texel = texture(texmap2, DataIn.tex_coord); // lightwood
        baseColor = vec4(lighting * texel.rgb, 1.0);
        alpha *= texel.a;
    }
    else if (texMode == 2) {
        texel = texture(texmap5, DataIn.tex_coord); // skyscraper_night
        baseColor = vec4(lighting * texel.rgb, 1.0);
        alpha *= texel.a;
    }
    else if (texMode == 3) {
        texel = texture(texmap6, DataIn.tex_coord); // skyscraper_plain
        baseColor = vec4(lighting * texel.rgb, 1.0);
        alpha *= texel.a;
    }
    else if (texMode == 4) {
        texel = texture(texmap7, DataIn.tex_coord); // skyscraper_glass
        baseColor = vec4(lighting * texel.rgb, 1.0);
        alpha *= texel.a;
    }
    else if (texMode == 5) {
        texel = texture(texmap8, DataIn.tex_coord); // skyscraper_residential
        baseColor = vec4(lighting * texel.rgb, 1.0);
        alpha *= texel.a;
    }
    else if (texMode == 6) { // fallback
        texel = texture(texmap3, DataIn.tex_coord);
        texel1 = texture(texmap4, DataIn.tex_coord);
        baseColor = vec4(lighting * (texel.rgb * texel1.rgb), 1.0);
        alpha *= texel.a;
    }
    else {
        texel = texture(texmap9, DataIn.tex_coord); // smoke particle
        baseColor = vec4(lighting * texel.rgb, 1.0);
        alpha *= texel.a;
    }

    // ---------- Fog (preserve your existing fog system) ----------
    float dist;
    if (depthFog == 0) {
        dist = abs(DataIn.posEye.z);
    } else {
        dist = length(DataIn.posEye);
    }

    float fogAmount = exp(-dist * fogDensity);
    fogAmount = clamp(fogAmount, 0.0, 1.0);

    vec3 finalColor = mix(fogColor, baseColor.rgb, fogAmount);
    colorOut = vec4(finalColor, alpha);
}
