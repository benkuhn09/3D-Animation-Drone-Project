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
    vec3 direction;
    vec3 color;
    bool on;
};

struct PointLight {
    vec3 position;
    vec3 color;
    bool on;
};

struct SpotLight {
    vec3 position;
    vec3 direction;
    vec3 color;
    bool on;
};

in Data {
	vec3 normal;
	vec3 eye;
	//vec3 lightDir;
	vec3 fragPos;
	vec2 tex_coord;
} DataIn;

uniform Materials mat;

uniform sampler2D texmap;
uniform sampler2D texmap1;
uniform sampler2D texmap2;

uniform int texMode;
uniform bool spotlight_mode;
uniform vec4 coneDir;
uniform float spotCosCutOff;

uniform DirectionalLight directionalLight;
uniform PointLight pointLights[NUMBER_POINT_LIGHTS];
uniform SpotLight spotLights[NUM_SPOT_LIGHTS];

out vec4 colorOut;

void main() {
	vec4 texel, texel1;
	vec4 finalColor = vec4(0.0);
    

	vec4 spec = vec4(0.0);
	float intensity = 0.0f;
	float intSpec = 0.0f;

	float att = 0.0;
	float spotExp = 60.0;

	vec3 n = normalize(DataIn.normal);
	//vec3 l = normalize(DataIn.lightDir);
	vec3 e = normalize(DataIn.eye);
	vec3 sd = normalize(coneDir.xyz);

	vec3 ambient = vec3(0.0);
    vec3 diffuse = vec3(0.0);
    vec3 specular = vec3(0.0);

	/* //lumiere du prof -- a supprimer 
	if(spotlight_mode == true)  {  //Scene iluminated by a spotlight
		float spotCos = dot(-l, sd);
		if(spotCos > spotCosCutOff)  {	//inside cone?
			att = pow(spotCos, spotExp);
			intensity = max(dot(n,l), 0.0) * att;
			if (intensity > 0.0) {
				vec3 h = normalize(l + e);
				intSpec = max(dot(h,n), 0.0);
				spec = mat.specular * pow(intSpec, mat.shininess) * att;
			}
		}
	}
	else {				//Scene iluminated by a pointlight
		intensity = max(dot(n,l), 0.0);
		if (intensity > 0.0) {
			vec3 h = normalize(l + e);
			intSpec = max(dot(h,n), 0.0);
			spec = mat.specular * pow(intSpec, mat.shininess);
		}
	}*/

	if (directionalLight.on) {
        // La direction de la lumière directionnelle est déjà en coordonnées de vue.
        // On la normalise pour être sûr.
        vec3 l = normalize(-directionalLight.direction);
        
        // Calcul de la contribution diffuse
        float diff = max(dot(n, l), 0.0);
        diffuse += directionalLight.color * diff * mat.diffuse.rgb;

        // Calcul de la contribution spéculaire
        if (diff > 0.0) {
            vec3 h = normalize(l + e);
            float specFactor = pow(max(dot(h, n), 0.0), mat.shininess);
            specular += directionalLight.color * specFactor * mat.specular.rgb;
        }
        
        // La lumière directionnelle n'a pas d'atténuation basée sur la distance
        ambient += directionalLight.color * mat.ambient.rgb;
    }

	// Point lights
	for (int i = 0; i < NUMBER_POINT_LIGHTS; i++) {
		if (pointLights[i].on) {
			// vecteur lumière (en view space)
			vec3 l = normalize(pointLights[i].position - DataIn.fragPos);
			float diff = max(dot(n, l), 0.0);

			// distance entre fragment et la source
			float dist = length(pointLights[i].position - DataIn.fragPos);
			float att = 1.0 / (1.0 + 0.09 * dist + 0.032 * dist * dist);

			diffuse += att * diff * pointLights[i].color * mat.diffuse.rgb;

			if (diff > 0.0) {
				vec3 h = normalize(l + e); // e reste le vecteur vue
				float specFactor = pow(max(dot(h, n), 0.0), mat.shininess);
				specular += att * specFactor * pointLights[i].color * mat.specular.rgb;
			}

			ambient += att * pointLights[i].color * mat.ambient.rgb;
		}
	}

	for (int i = 0; i < NUM_SPOT_LIGHTS; i++) {
		if (spotLights[i].on) {
			// vecteur du fragment vers la lumière
			vec3 l = normalize(spotLights[i].position - DataIn.fragPos);

			// angle avec le cône
			float spotCos = dot(-l, normalize(spotLights[i].direction));
			if (spotCos > spotCosCutOff) {
				// atténuation en fonction de la distance
				float dist = length(spotLights[i].position - DataIn.fragPos);
				float att = 1.0 / (1.0 + 0.1*dist + 0.01*dist*dist);

				// diffuse
				float diff = max(dot(n, l), 0.0);
				diffuse += att * diff * spotLights[i].color * mat.diffuse.rgb;

				// spéculaire
				if (diff > 0.0) {
					vec3 h = normalize(l + e);
					float specFactor = pow(max(dot(h, n), 0.0), mat.shininess);
					specular += att * specFactor * spotLights[i].color * mat.specular.rgb;
				}

				// lumière ambiante faible pour les spots
				ambient += 0.1 * att * spotLights[i].color * mat.ambient.rgb;
			}
		}
	}


	// Couleur finale
	vec3 lighting = ambient + diffuse + specular;

	if (texMode == 0) {
		// Pas de texture
		colorOut = vec4(lighting, 1.0);
	}
	else if (texMode == 1) {
		texel = texture(texmap2, DataIn.tex_coord);
		colorOut = vec4(lighting * texel.rgb, 1.0);
	}
	else if (texMode == 2) {
		texel = texture(texmap, DataIn.tex_coord);
		colorOut = vec4(lighting * texel.rgb, 1.0);
	}
	else {
		texel  = texture(texmap2, DataIn.tex_coord);
		texel1 = texture(texmap1, DataIn.tex_coord);
		colorOut = vec4(lighting * (texel.rgb * texel1.rgb), 1.0);
	}

}