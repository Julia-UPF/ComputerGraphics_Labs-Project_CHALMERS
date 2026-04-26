#version 420

/* WARNNING: the following must match the values in lab6_main.cpp
	 */
#define SOLUTION_USE_BUILTIN_SHADOW_TEST 1

// required by GLSL spec Sect 4.5.3 (though nvidia does not, amd does)
precision highp float;

///////////////////////////////////////////////////////////////////////////////
// Material
///////////////////////////////////////////////////////////////////////////////
uniform vec3 material_color;
uniform float material_metalness;
uniform float material_fresnel;
uniform float material_shininess;
uniform vec3 material_emission;

uniform int has_color_texture;
layout(binding = 0) uniform sampler2D colorMap;
uniform int has_emission_texture;
layout(binding = 5) uniform sampler2D emissiveMap;

///////////////////////////////////////////////////////////////////////////////
// Environment
///////////////////////////////////////////////////////////////////////////////
layout(binding = 6) uniform sampler2D environmentMap;
layout(binding = 7) uniform sampler2D irradianceMap;
layout(binding = 8) uniform sampler2D reflectionMap;
uniform float environment_multiplier;

///////////////////////////////////////////////////////////////////////////////
// Light source
///////////////////////////////////////////////////////////////////////////////
uniform vec3 point_light_color = vec3(1.0, 1.0, 1.0);
uniform float point_light_intensity_multiplier = 50.0;

///////////////////////////////////////////////////////////////////////////////
// Constants
///////////////////////////////////////////////////////////////////////////////
#define PI 3.14159265359

///////////////////////////////////////////////////////////////////////////////
// Input varyings from vertex shader
///////////////////////////////////////////////////////////////////////////////
in vec2 texCoord;
in vec3 viewSpaceNormal;
in vec3 viewSpacePosition;

in vec4 shadowMapCoord;

///////////////////////////////////////////////////////////////////////////////
// Input uniform variables
///////////////////////////////////////////////////////////////////////////////
uniform mat4 viewInverse;
uniform vec3 viewSpaceLightPosition;

uniform vec3 viewSpaceLightDir;
uniform float spotOuterAngle;
uniform float spotInnerAngle;
uniform int useSpotLight;
uniform int useSoftFalloff;


//layout(binding = 10) uniform sampler2D shadowMapTex;
layout(binding = 10) uniform sampler2DShadow shadowMapTex;

///////////////////////////////////////////////////////////////////////////////
// Output color
///////////////////////////////////////////////////////////////////////////////
layout(location = 0) out vec4 fragmentColor;





vec3 calculateDirectIllumiunation(vec3 wo, vec3 n, vec3 base_color)
{
	// Diffuse term
	vec3 light_direction = viewSpaceLightPosition - viewSpacePosition;
	float light_distance = length(light_direction);
	vec3 L_i = point_light_color * point_light_intensity_multiplier*(1/(light_distance*light_distance));
	vec3 wi = normalize(light_direction);
	if(dot(wi, n) <= 0){
		return vec3(0.0);
	}

	vec3 diffuse_term = base_color*(1/PI)*dot(wi, n)*L_i;

	//Reflected term
	vec3 wh = normalize(wi + wo);
	float wh_dot_wo = max(0.0001, dot(wh, wo));
	float n_dot_wh = max(0.0001, dot(n, wh));
	float n_dot_wo = max(0.0001, dot(n, wo));
	float n_dot_wi = max(0.0001, dot(n, wi));

	float F = material_fresnel + (1-material_fresnel)*pow(1-wh_dot_wo, 5);
	float D = ((material_shininess+2)/(2*PI))*pow(n_dot_wh, material_shininess);
	float G = min(1.0, min(2.0*n_dot_wh*n_dot_wo/wh_dot_wo, 2.0*n_dot_wh*n_dot_wi/wh_dot_wo));
	float denominator = 4.0 * clamp(n_dot_wo * n_dot_wi, 0.0001, 1.0);	

	float brdf = (F*D*G)/denominator;

	//Material
	vec3 dielectric_term = brdf*n_dot_wi*L_i+(1-F)*diffuse_term;
	vec3 metal_term = brdf*base_color*n_dot_wi*L_i;
	vec3 direct_illum = material_metalness*metal_term+(1-material_metalness)*dielectric_term;

	return direct_illum;
}

vec3 calculateIndirectIllumination(vec3 wo, vec3 n, vec3 base_color)
{
	//Lookup the irradiance from the irradiance map and calculate the diffuse reflection
	
	vec3 n_world_space = vec3(viewInverse * vec4(n, 0.0));
	float theta = acos(max(-1.0f, min(1.0f, n_world_space.y)));
	float phi = atan(n_world_space.z, n_world_space.x);
	if(phi < 0.0f)
		phi = phi + 2.0f * PI;

	vec2 lookup = vec2(phi / (2.0 * PI), 1 - theta / PI);
	vec3 L_i = environment_multiplier * texture(irradianceMap, lookup).rgb;

	vec3 diffuse_term = base_color * (1.0 / PI) * L_i;

	//Look up in the reflection map from the perfect specular direction and calculate the dielectric and metal terms.
	vec3 wi = normalize(reflect(-wo, n));
	float roughness = sqrt(sqrt(2/(material_shininess+2)));

	vec3 wr = normalize(vec3(viewInverse * vec4(wi, 0.0)));
	theta = acos(max(-1.0f, min(1.0f, wr.y)));
	phi = atan(wr.z, wr.x);
	if(phi < 0.0f)
		phi = phi + 2.0f * PI;

	// Use these to lookup the color in the environment map
	lookup = vec2(phi / (2.0 * PI), 1 - theta / PI);
	L_i = environment_multiplier * textureLod(reflectionMap, lookup, roughness*0.7).rgb;

	vec3 wh = normalize(wi + wo);
	float wh_dot_wo = max(0.0001, dot(wh, wo));
	float F = material_fresnel + (1-material_fresnel)*pow(1-wh_dot_wo, 5);
	vec3 dielectric_term = F*L_i +(1-F)*diffuse_term;
	vec3 metal_term = base_color*F*L_i;

	vec3 indirect_illum = material_metalness*metal_term+(1-material_metalness)*dielectric_term;
	return indirect_illum;
}

void main()
{
	float visibility = 1.0;
	float attenuation = 1.0;
		
	//float depth = texture(shadowMapTex, shadowMapCoord.xy / shadowMapCoord.w).r;
	//visibility = (depth >= (shadowMapCoord.z / shadowMapCoord.w)) ? 1.0 : 0.0;
	visibility = textureProj(shadowMapTex, shadowMapCoord);

	vec3 posToLight = normalize(viewSpaceLightPosition - viewSpacePosition);
	float cosAngle = dot(posToLight, -viewSpaceLightDir);

	if(useSpotLight == 1)
	{
		vec3 posToLight = normalize(viewSpaceLightPosition - viewSpacePosition);
		float cosAngle = dot(posToLight, -viewSpaceLightDir);

		if(useSoftFalloff == 0)
		{
			// Spotlight with hard border:
			attenuation = (cosAngle > spotOuterAngle) ? 1.0 : 0.0;
		}
		else
		{
			// Spotlight with soft border:
			attenuation = smoothstep(spotOuterAngle, spotInnerAngle, cosAngle);
		}

		visibility *= attenuation;
	}

	vec3 wo = -normalize(viewSpacePosition);
	vec3 n = normalize(viewSpaceNormal);

	vec3 base_color = material_color;
	if(has_color_texture == 1)
	{
		base_color = texture(colorMap, texCoord).rgb;
	}

	// Direct illumination
	vec3 direct_illumination_term = visibility * calculateDirectIllumiunation(wo, n, base_color);

	// Indirect illumination
	vec3 indirect_illumination_term = calculateIndirectIllumination(wo, n, base_color);

	///////////////////////////////////////////////////////////////////////////
	// Add emissive term. If emissive texture exists, sample this term.
	///////////////////////////////////////////////////////////////////////////
	vec3 emission_term = material_emission;
	if(has_emission_texture == 1)
	{
		emission_term = texture(emissiveMap, texCoord).rgb;
	}

	vec3 shading = direct_illumination_term + indirect_illumination_term + emission_term;

	fragmentColor = vec4(shading, 1.0);

	return;
}
