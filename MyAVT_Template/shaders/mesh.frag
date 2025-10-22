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
    vec3 worldDir; 
    vec3 T;
    vec3 B;
    vec3 N;
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
uniform sampler2D texmap10;
uniform samplerCube skybox;

uniform int texMode;

// Lighting uniforms
uniform DirectionalLight directionalLight;
uniform PointLight pointLights[NUMBER_POINT_LIGHTS];
uniform SpotLight spotLights[NUM_SPOT_LIGHTS];

// Fog uniforms
uniform int depthFog;
uniform vec3 fogColor;
uniform float fogDensity;

out vec4 colorOut;

void main() {

    

    //computing normal (regular or fake, depending on texMode)
    vec3 n;
    if (texMode == 14) {
        vec3 n_ts = normalize(2.0 * texture(texmap10, DataIn.tex_coord).rgb - 1.0);
        n = normalize(mat3(DataIn.T, DataIn.B, DataIn.N) * n_ts);
        // UNCOMMENT BELOW TO VERIFY NORMAL MAPPING
        //colorOut = vec4(0.5 * (n + 1.0), 1.0); return;
    } else {
        n = normalize(DataIn.normal);
    }

    vec3 e = normalize(DataIn.eye);

    //lighting
    vec3 ambient = vec3(0.2);
    vec3 diffuse = vec3(0.0);
    vec3 specular = vec3(0.0);

    // Directional light
    if (directionalLight.on) {
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

    // Point lights
    for (int i = 0; i < NUMBER_POINT_LIGHTS; i++) {
        if (pointLights[i].on) {
            vec3 l = normalize(pointLights[i].position - DataIn.fragPos);
            float diff = max(dot(n, l), 0.0);

            float dist = length(pointLights[i].position - DataIn.fragPos);
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

    // Spot lights
    for (int i = 0; i < NUM_SPOT_LIGHTS; i++) {
        if (spotLights[i].on) {
            vec3 l = normalize(spotLights[i].position - DataIn.fragPos);
            float spotCos = dot(-l, normalize(spotLights[i].direction));
            float spotCutoff = 0.85;
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

                ambient += 0.1 * att * spotLights[i].color * mat.ambient.rgb;
            }
        }
    }

    vec3 lighting = ambient + diffuse + specular;


    //Texture Application

    vec4 texel, texel1;
    vec4 baseColor;
    float alpha = mat.diffuse.a;

    if (texMode == 0) {
        baseColor = vec4(lighting, 1.0);
    }
    else if (texMode == 1) {
        texel = texture(texmap2, DataIn.tex_coord);
        baseColor = vec4(lighting * texel.rgb, 1.0);
        alpha *= texel.a;
    }
    else if (texMode == 2) {
        texel = texture(texmap5, DataIn.tex_coord);
        baseColor = vec4(lighting * texel.rgb, 1.0);
        alpha *= texel.a;
    }
    else if (texMode == 3) {
        texel = texture(texmap6, DataIn.tex_coord);
        baseColor = vec4(lighting * texel.rgb, 1.0);
        alpha *= texel.a;
    }
    else if (texMode == 4) {
        texel = texture(texmap7, DataIn.tex_coord);
        baseColor = vec4(lighting * texel.rgb, 1.0);
        alpha *= texel.a;
    }
    else if (texMode == 5) {
        texel = texture(texmap8, DataIn.tex_coord);
        baseColor = vec4(lighting * texel.rgb, 1.0);
        alpha *= texel.a;
    }
    else if (texMode == 6) {
        texel = texture(texmap3, DataIn.tex_coord);
        texel1 = texture(texmap4, DataIn.tex_coord);
        baseColor = vec4(lighting * (texel.rgb * texel1.rgb), 1.0);
        alpha *= texel.a;
    }
    else if (texMode == 7) {
        vec4 t = texture(texmap9, DataIn.tex_coord);
        vec3 rgb = t.rgb * t.a * mat.diffuse.rgb;
        float a = t.a * mat.diffuse.a;
        baseColor = vec4(rgb, a);
    }
    else if (texMode == 8) {
        vec4 t = texture(texmap9, DataIn.tex_coord);
        vec3 rgb = t.rgb * mat.diffuse.rgb;
        baseColor = vec4(rgb, 1.0);
    }
    else if (texMode == 9) {
        baseColor = vec4(0.0, 0.0, 0.0, 1.0);
    }
    else if (texMode == 10) {
        vec3 dir = normalize(DataIn.worldDir);
        baseColor = texture(skybox, dir);
        alpha = 1.0;
    }
    else if (texMode == 11) {
        vec3 n = normalize(DataIn.normal);
        vec3 e = normalize(DataIn.eye);
        vec3 I = normalize(-e);
        vec3 R = reflect(I, n);

        vec3 env = texture(skybox, R).rgb;
        vec4 glassTex = texture(texmap7, DataIn.tex_coord);
        vec3 tint = glassTex.rgb * mat.diffuse.rgb;

        float F0 = 0.04;
        float cosTheta = clamp(dot(n, e), 0.0, 1.0);
        float Fresnel = F0 + (1.0 - F0) * pow(1.0 - cosTheta, 5.0);
        float reflectivity = 2.5;

        vec3 litBase = lighting * tint;
        vec3 rgb = mix(litBase, env, reflectivity * Fresnel);

        float a = mat.diffuse.a * (glassTex.a > 0.0 ? glassTex.a : 1.0);
        baseColor = vec4(rgb, a);
        alpha = a;
    }
    else if (texMode == 13) {
        vec3 n = normalize(DataIn.normal);
        vec3 e = normalize(DataIn.eye);
        vec3 I = normalize(-e);
        vec3 R = reflect(I, n);

        vec3 env = texture(skybox, R).rgb;
        vec3 litBase = lighting * mat.diffuse.rgb;

        float F0 = 0.12;
        float cosTheta = clamp(dot(n, e), 0.0, 1.0);
        float Fresnel = F0 + (1.0 - F0) * pow(1.0 - cosTheta, 5.0);
        float reflectivity = 1.2;

        vec3 rgb = mix(litBase, env, reflectivity * Fresnel);
        baseColor = vec4(rgb, mat.diffuse.a);
    }
    else if (texMode == 14) {
        // Apply bump-mapped lighting as before
        vec4 tA = texture(texmap3, DataIn.tex_coord);
        vec4 tB = texture(texmap4, DataIn.tex_coord);
        vec3 rgb = lighting * (tA.rgb * tB.rgb);

        // Combine texture alpha (tA.a) with material alpha from CPU
        float aTex = tA.a;
        float aMat = mat.diffuse.a;  
        float aOut = aTex * aMat;

        baseColor = vec4(rgb, aOut);
        alpha = aOut;
    }
    else {
        texel = texture(texmap9, DataIn.tex_coord);
        baseColor = vec4(lighting * texel.rgb, 1.0);
        alpha *= texel.a;
    }

    //fog
    float dist = (depthFog == 0) ? abs(DataIn.posEye.z) : length(DataIn.posEye);
    float fogAmount = clamp(exp(-dist * fogDensity), 0.0, 1.0);

    vec3 finalColor = mix(fogColor, baseColor.rgb, fogAmount);
    colorOut = vec4(finalColor, alpha);
}
