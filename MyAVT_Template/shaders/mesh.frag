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
    vec3 worldDir; // new
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
uniform samplerCube skybox;

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
    else if (texMode == 7) {
    vec4 t = texture(texmap9, DataIn.tex_coord);
    vec3 rgb = t.rgb * t.a * mat.diffuse.rgb;
    float a  = t.a * mat.diffuse.a;
    baseColor = vec4(rgb, a);
}
    
    else if (texMode == 8) {
        vec4 t = texture(texmap9, DataIn.tex_coord);
        vec3 rgb = t.rgb * mat.diffuse.rgb;  // <-- no * t.a
        baseColor = vec4(rgb, 1.0);          // alpha unused with GL_ONE, GL_ONE
    }
    else if (texMode == 9) {
        baseColor = vec4(0.0, 0.0, 0.0, 1.0);
        alpha = mat.diffuse.a;
    }
    else if (texMode == 10) {
        // Skybox mode
        vec3 dir = normalize(DataIn.worldDir);
        baseColor = texture(skybox, dir);
        alpha = 1.0;
    }
    else if (texMode == 11) {
        // --- Glass + env-mapped reflection ---
        vec3 n = normalize(DataIn.normal);          // view-space normal
        vec3 e = normalize(DataIn.eye);             // view-space vector from surface -> eye
        vec3 I = normalize(-e);                     // incident vector (from eye -> surface)
        vec3 R = reflect(I, n);                     // reflection dir in view space

        // Sample the environment
        vec3 env = texture(skybox, R).rgb;

        // Optional: sample your glass texture as a tint/pattern (windows etc.)
        // If your glass buildings use texmap7, keep this; otherwise remove.
        vec4 glassTex = texture(texmap7, DataIn.tex_coord);
        vec3 tint = glassTex.rgb * mat.diffuse.rgb;

        // Schlick Fresnel: F0 ~ 0.02–0.08 for dielectrics (pick what you like)
        float F0 = 0.04; // can promote to a uniform if you want artistic control
        float cosTheta = clamp(dot(n, e), 0.0, 1.0);
        float Fresnel = F0 + (1.0 - F0) * pow(1.0 - cosTheta, 5.0);

        // Amount of reflection (artist control): multiply Fresnel by a scalar
        float reflectivity = 2.5; // 0..1, tweak to taste

        // Lit base (diffuse/spec/ambient you computed above)
        vec3 litBase = lighting * tint;

        // Blend env reflection over lit base
        vec3 rgb = mix(litBase, env, reflectivity * Fresnel);

        // Alpha: keep your glass translucency; if glassTex has alpha, use it too
        float a = mat.diffuse.a * (glassTex.a > 0.0 ? glassTex.a : 1.0);

        baseColor = vec4(rgb, a);
        alpha = a;
    }
    else if (texMode == 13) {
        // --- Metallic / flying objects: lit base + environment reflection ---
        vec3 n = normalize(DataIn.normal);
        vec3 e = normalize(DataIn.eye);
        vec3 I = normalize(-e);
        vec3 R = reflect(I, n);

        // Sample environment
        vec3 env = texture(skybox, R).rgb;

        // Base lighting (same as texMode 0)
        vec3 litBase = lighting * mat.diffuse.rgb;

        // Fresnel (stronger for metals)
        float F0 = 0.12;                       // more reflective base value
        float cosTheta = clamp(dot(n, e), 0.0, 1.0);
        float Fresnel = F0 + (1.0 - F0) * pow(1.0 - cosTheta, 5.0);

        // Reflection mix
        float reflectivity = 1.2;              // adjust intensity to taste
        vec3 rgb = mix(litBase, env, reflectivity * Fresnel);

        // Metallic objects are opaque
        float a = mat.diffuse.a;
        baseColor = vec4(rgb, a);
        alpha = a;
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
