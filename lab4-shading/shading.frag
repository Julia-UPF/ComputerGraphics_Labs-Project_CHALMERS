#version 420

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

///////////////////////////////////////////////////////////////////////////////
// Input uniform variables
///////////////////////////////////////////////////////////////////////////////
uniform mat4 viewInverse;
uniform vec3 viewSpaceLightPosition;

///////////////////////////////////////////////////////////////////////////////
// Output color
///////////////////////////////////////////////////////////////////////////////
layout(location = 0) out vec4 fragmentColor;


vec3 calculateDirectIllumiunation(vec3 wo, vec3 n, vec3 base_color)
{
	vec3 direct_illum = base_color;
	///////////////////////////////////////////////////////////////////////////
	// Task 1.2 - Calculate the radiance Li from the light, and the direction
	//            to the light. If the light is backfacing the triangle,
	//            return vec3(0);
	///////////////////////////////////////////////////////////////////////////
	vec3 light_direction = viewSpaceLightPosition - viewSpacePosition;
	const float light_distance = length(light_direction);
	vec3 L_i = point_light_color * point_light_intensity_multiplier*(1/(light_distance*light_distance));
	vec3 wi = normalize(light_direction);
	if(dot(wi, n) <= 0){
		return vec3(0.0);
	}

	///////////////////////////////////////////////////////////////////////////
	// Task 1.3 - Calculate the diffuse term and return that as the result
	///////////////////////////////////////////////////////////////////////////
	vec3 diffuse_term = base_color*(1/PI)*dot(wi, n)*L_i;

	///////////////////////////////////////////////////////////////////////////
	// Task 2 - Calculate the Torrance Sparrow BRDF and return the light
	//          reflected from that instead
	///////////////////////////////////////////////////////////////////////////
	vec3 wh = normalize(wi + wo);
	float wh_dot_wo = max(0.0001, dot(wh, wo));
	float n_dot_wh = max(0.0001, dot(n, wh));
	float n_dot_wo = max(0.0001, dot(n, wo));
	float n_dot_wi = max(0.0001, dot(n, wi));

	const float F = material_fresnel + (1-material_fresnel)*pow(1-wh_dot_wo, 5);
	const float D = ((material_shininess+2)/(2*PI))*pow(n_dot_wh, material_shininess);
	const float G = min(1.0, min(2.0*n_dot_wh*n_dot_wo/wh_dot_wo, 2.0*n_dot_wh*n_dot_wi/wh_dot_wo));
	float denominator = 4.0 * clamp(n_dot_wo * n_dot_wi, 0.0001, 1.0);	

	const float brdf = (F*D*G)/denominator;

	///////////////////////////////////////////////////////////////////////////
	// Task 3 - Make your shader respect the parameters of our material model.
	///////////////////////////////////////////////////////////////////////////
	vec3 dielectric_term = brdf*n_dot_wi*L_i+(1-F)*diffuse_term;
	vec3 metal_term = brdf*base_color*n_dot_wi*L_i;
	direct_illum = material_metalness*metal_term+(1-material_metalness)*dielectric_term;

	return direct_illum;
}

vec3 calculateIndirectIllumination(vec3 wo, vec3 n, vec3 base_color)
{
	vec3 indirect_illum = vec3(0.f);
	///////////////////////////////////////////////////////////////////////////
	// Task 5 - Lookup the irradiance from the irradiance map and calculate
	//          the diffuse reflection
	///////////////////////////////////////////////////////////////////////////
	vec3 n_world_space = vec3(viewInverse * vec4(n, 0.0));
	float theta = acos(max(-1.0f, min(1.0f, n_world_space.y)));
	float phi = atan(n_world_space.z, n_world_space.x);
	if(phi < 0.0f)
		phi = phi + 2.0f * PI;

	vec2 lookup = vec2(phi / (2.0 * PI), 1 - theta / PI);
	vec3 L_i = environment_multiplier * texture(irradianceMap, lookup).rgb;

	vec3 diffuse_term = base_color * (1.0 / PI) * L_i;

	///////////////////////////////////////////////////////////////////////////
	// Task 6 - Look up in the reflection map from the perfect specular
	//          direction and calculate the dielectric and metal terms.
	///////////////////////////////////////////////////////////////////////////
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

	indirect_illum = material_metalness*metal_term+(1-material_metalness)*dielectric_term;


	return indirect_illum;
}


void main()
{
	///////////////////////////////////////////////////////////////////////////
	// Task 1.1 - Fill in the outgoing direction, wo, and the normal, n. Both
	//            shall be normalized vectors in view-space.
	///////////////////////////////////////////////////////////////////////////
	vec3 wo = normalize(-viewSpacePosition);
	vec3 n = normalize(viewSpaceNormal);

	vec3 base_color = material_color;
	if(has_color_texture == 1)
	{
		base_color *= texture(colorMap, texCoord).rgb;
	}

	vec3 direct_illumination_term = vec3(0.0);
	{ // Direct illumination
		direct_illumination_term = calculateDirectIllumiunation(wo, n, base_color);
	}

	vec3 indirect_illumination_term = vec3(0.0);
	{ // Indirect illumination
		indirect_illumination_term = calculateIndirectIllumination(wo, n, base_color);
	}

	///////////////////////////////////////////////////////////////////////////
	// Task 1.4 - Make glowy things glow!
	///////////////////////////////////////////////////////////////////////////
	vec3 emission_term = material_emission;

	vec3 final_color = direct_illumination_term + indirect_illumination_term + emission_term;

	// Check if we got invalid results in the operations
	if(any(isnan(final_color)))
	{
		final_color.rgb = vec3(1.f, 0.f, 1.f);
	}

	fragmentColor.rgb = final_color;
}
