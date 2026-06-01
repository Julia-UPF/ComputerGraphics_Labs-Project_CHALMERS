#include "Pathtracer.h"
#include <memory>
#include <iostream>
#include <map>
#include <algorithm>
#include "material.h"
#include "embree.h"
#include "sampling.h"
#include "labhelper.h"

using namespace std;
using namespace glm;
using namespace labhelper;

namespace pathtracer
{
///////////////////////////////////////////////////////////////////////////////
// Global variables
///////////////////////////////////////////////////////////////////////////////
Settings settings;
Environment environment;
Image rendered_image;
PointLight point_light;
std::vector<DiscLight> disc_lights;

//For post-processing
std::vector<vec3> normal_buffer;		//indexing will be idx = y*width + x
std::vector<float> depth_buffer;		//color buffer in rendered_image.data
Image filtered_image;


///////////////////////////////////////////////////////////////////////////
// Restart rendering of image
///////////////////////////////////////////////////////////////////////////
void restart()
{
	// No need to clear image,
	rendered_image.number_of_samples = 0;
}

int getSampleCount()
{
	return std::max(rendered_image.number_of_samples - 1, 0);
}

///////////////////////////////////////////////////////////////////////////
// On window resize, window size is passed in, actual size of pathtraced
// image may be smaller (if we're subsampling for speed)
///////////////////////////////////////////////////////////////////////////
void resize(int w, int h)
{
	rendered_image.width = w / settings.subsampling;
	rendered_image.height = h / settings.subsampling;
	rendered_image.data.resize(rendered_image.width * rendered_image.height);
	normal_buffer.resize(rendered_image.width * rendered_image.height);
	depth_buffer.resize(rendered_image.width * rendered_image.height);
	restart();
}

///////////////////////////////////////////////////////////////////////////
/// Return the radiance from a certain direction wi from the environment
/// map.
///////////////////////////////////////////////////////////////////////////
vec3 Lenvironment(const vec3& wi)
{
	const float theta = acos(std::max(-1.0f, std::min(1.0f, wi.y)));
	float phi = atan(wi.z, wi.x);
	if(phi < 0.0f)
		phi = phi + 2.0f * M_PI;
	vec2 lookup = vec2(phi / (2.0 * M_PI), 1 - theta / M_PI);
	return environment.multiplier * environment.map.sample(lookup.x, lookup.y);
}

///////////////////////////////////////////////////////////////////////////
/// Calculate the radiance going from one point (r.hitPosition()) in one
/// direction (-r.d), through path tracing.
///////////////////////////////////////////////////////////////////////////
vec3 Li(Ray& primary_ray)
{
	bool inside = false;		//Added to keep track of whether the ray is inside a medium or not, for refraction
	vec3 sigma_t = vec3(0.0f);	//Added to keep track of absorption in the medium for refraction
	vec3 L = vec3(0.0f);
	vec3 path_throughput = vec3(1.0);
	Ray current_ray = primary_ray;

	for (int bounce = 0; bounce < settings.max_bounces; ++bounce) {		//ADDED for multiple bounces
		///////////////////////////////////////////////////////////////////
		// Get the intersection information from the ray
		///////////////////////////////////////////////////////////////////
		Intersection hit = getIntersection(current_ray);
		if (inside) {
			float distance = length(hit.position - current_ray.o);
			vec3 transmittance = exp(-sigma_t * distance);
			path_throughput *= transmittance;		//Calculate absorption/scattering in the medium for refraction
		}

		///////////////////////////////////////////////////////////////////
		// Create a Material tree for evaluating brdfs and calculating
		// sample directions.
		///////////////////////////////////////////////////////////////////

		/*Diffuse diffuse(hit.material->m_color);								//Full (without refractions)
		MicrofacetBRDF microfacet(hit.material->m_shininess);
		DielectricBSDF dielectric(&microfacet, &diffuse, hit.material->m_fresnel);
		MetalBSDF metal(&microfacet, hit.material->m_color, hit.material->m_fresnel);
		BSDFLinearBlend metal_blend(hit.material->m_metalness, &metal, &dielectric);
		BSDF& mat = metal_blend; */

		//GlassBTDF glass(hit.material->m_ior);		//Glass only
		//BTDF& mat = glass;

		Diffuse diffuse(hit.material->m_color);								//Full (with refractions)
		GlassBTDF glass(hit.material->m_ior);
		BTDFLinearBlend glass_blend(hit.material->m_transparency, &glass, &diffuse);
		MicrofacetBRDF microfacet(hit.material->m_shininess);
		DielectricBSDF dielectric(&microfacet, &glass_blend, hit.material->m_fresnel);
		MetalBSDF metal(&microfacet, hit.material->m_color, hit.material->m_fresnel);
		BSDFLinearBlend metal_blend(hit.material->m_metalness, &metal, &dielectric);
		BSDF& mat = metal_blend;


		//Diffuse diffuse(hit.material->m_color);		//Diffuse only
		//BTDF& mat = diffuse;

		/*Diffuse diffuse(hit.material->m_color);			//Dielectric only
		MicrofacetBRDF microfacet(hit.material->m_shininess);
		DielectricBSDF dielectric(&microfacet, &diffuse, hit.material->m_fresnel);
		BSDF& mat = dielectric;*/


		///////////////////////////////////////////////////////////
		// Direct illumination with Area Lighs
		///////////////////////////////////////////////////////////
		vec3 direct_illum = vec3(0.0f);
		for (const DiscLight& light: disc_lights) {
			vec3 light_normal = normalize(light.direction);
			//Sample random point
			float r = sqrt(randf()) * light.radius;
			float theta = 2 * M_PI * randf();

			float x = r * cos(theta);
			float y = r * sin(theta);

			vec3 tangent = normalize(perpendicular(light_normal));
			vec3 bitangent = normalize(cross(light_normal, tangent));

			vec3 light_pos = light.position + x * tangent + y * bitangent;


			//shadow
			Ray shadow_ray;				//Added for shadows
			shadow_ray.o = hit.position + hit.geometry_normal * EPSILON; //ray doesn't start exactly on the surface to avoid self-intersection
			shadow_ray.d = normalize(light_pos - hit.position);

			if (occluded(shadow_ray))
				continue;

			float cos_surface = std::max(0.0f, dot(hit.shading_normal, shadow_ray.d));
			float cos_light = std::max(0.0f, dot(light_normal, -shadow_ray.d));

			vec3 Li = light.color * light.intensity_multiplier;

			const float distance_to_light = length(light_pos - hit.position);
			const float falloff_factor = cos_surface*cos_light / (distance_to_light * distance_to_light);
			vec3 contribution = mat.f(shadow_ray.d, hit.wo, hit.shading_normal)*Li*falloff_factor;

			direct_illum += contribution;

		}

		L += path_throughput * direct_illum;


		///////////////////////////////////////////////////////////////////
		// Calculate Direct Illumination from point light.
		///////////////////////////////////////////////////////////////////
		Ray shadow_ray;				//Added for shadows
		shadow_ray.o = hit.position + hit.geometry_normal * EPSILON; //ray doesn't start exactly on the surface to avoid self-intersection
		shadow_ray.d = normalize(point_light.position - hit.position);
											//if occluded, the point is in shadow and we don't add any contribution from the light source
		const float distance_to_light = length(point_light.position - hit.position);
		const float falloff_factor = 1.0f / (distance_to_light * distance_to_light);
		vec3 Li = point_light.intensity_multiplier * point_light.color * falloff_factor;
		vec3 wi = normalize(point_light.position - hit.position);
		vec3 direct_illumination = mat.f(wi, hit.wo, hit.shading_normal) * Li * std::max(0.0f, dot(wi, hit.shading_normal));
		
		if (!occluded(shadow_ray)) {
			L += path_throughput * direct_illumination;
		}
		

		L += path_throughput * hit.material->m_emission;			//if ray hits emissive object, add this light
		


		//SAMPLE NEXT RAY
		WiSample sample = mat.sample_wi(hit.wo, hit.shading_normal);

		if (sample.pdf < EPSILON || path_throughput == vec3(0.0f))
			return L;

		float cosineterm = abs(dot(sample.wi, hit.shading_normal));
		path_throughput = path_throughput * (sample.f * cosineterm) / sample.pdf;	//prob of this next ray


		Ray new_ray;
		new_ray.d = sample.wi;
		if(dot(sample.wi, hit.geometry_normal) < 0.0f)
		{
			new_ray.o = hit.position - hit.geometry_normal * EPSILON;
			inside = true;		//If the ray goes inside the object, we set inside to true
			sigma_t = (vec3(1.0f) - hit.material->m_color)*0.1f;		//Set sigma to the opposite of color of the material for refraction and scale it so that it's not too strong (the 0.1f is just a value I found to give good results, it can be changed to get more or less absorption in the medium for refraction)
			//it could also be set depending on the material but for simplicity (not change all materials to have a sigma value) I just set it depending on the color of the material, so that darker materials have more absorption and lighter materials have less absorption for refraction
		}
		else
		{
			new_ray.o = hit.position + hit.geometry_normal * EPSILON;
			inside = false;		//If the ray goes outside the object, we set inside to false
			sigma_t = vec3(0.0f);		//No absorption outside the object for refraction
		}
		current_ray = new_ray;

		if (!intersect(current_ray))
			return L + path_throughput * Lenvironment(current_ray.d);


	}
	// Return the final outgoing radiance for the primary ray
	return L;
}

///////////////////////////////////////////////////////////////////////////
/// Used to homogenize points transformed with projection matrices
///////////////////////////////////////////////////////////////////////////
inline static glm::vec3 homogenize(const glm::vec4& p)
{
	return glm::vec3(p * (1.f / p.w));
}

///////////////////////////////////////////////////////////////////////////
/// Trace one path per pixel and accumulate the result in an image
///////////////////////////////////////////////////////////////////////////
void tracePaths(const glm::mat4& V, const glm::mat4& P)
{
	// Stop here if we have as many samples as we want
	if((int(rendered_image.number_of_samples) > settings.max_paths_per_pixel)
	   && (settings.max_paths_per_pixel != 0))
	{
		return;
	}
	vec3 camera_pos = vec3(glm::inverse(V) * vec4(0.0f, 0.0f, 0.0f, 1.0f));
	std::fill(normal_buffer.begin(), normal_buffer.end(), vec3(0.0f)); //fill normal_buffer with initiaal vec3(0) normals
	std::fill(depth_buffer.begin(), depth_buffer.end(), 1e30f); //fill depth_buffer with initiaal large number (best would be with far plane values but i couldn't find it)


	// Trace one path per pixel (the omp parallel stuf magically distributes the
	// pathtracing on all cores of your CPU).
	int num_rays = 0;
	vector<vec4> local_image(rendered_image.width * rendered_image.height, vec4(0.0f));

#pragma omp parallel for
	for(int y = 0; y < rendered_image.height; y++)
	{
		for(int x = 0; x < rendered_image.width; x++)
		{
			int pixel_idx = y * rendered_image.width + x;
			vec3 color;
			Ray primaryRay;
			primaryRay.o = camera_pos;
			// Create a ray that starts in the camera position and points toward
			// the current pixel on a virtual screen.
			float random1 = randf();		//ADDED to get a random number in the range [0,1] (anti-aliasing) 
			float random2 = randf();
			vec2 screenCoord = vec2((float(x) + random1)/ float(rendered_image.width),
			                        (float(y) + random2)/ float(rendered_image.height));
			// Calculate direction
			vec4 viewCoord = vec4(screenCoord.x * 2.0f - 1.0f, screenCoord.y * 2.0f - 1.0f, 1.0f, 1.0f);
			vec3 p = homogenize(inverse(P * V) * viewCoord);
			primaryRay.d = normalize(p - camera_pos);

			if (settings.use_depth_field) {
				//find camera space basis
				vec3 forward = normalize(primaryRay.d);		//direction camera is facing
				vec3 camera_right = normalize(cross(forward, vec3(0.0f, 1.0f, 0.0f)));	//right vector of the camera, perpendicular to the forward direction and the world up vector (0,1,0)
				vec3 camera_up = normalize(cross(camera_right, forward));	//up vector of the camera, perpendicular to the forward direction and the camera right vector

				vec3 focal_point = camera_pos + forward * settings.focus_distance;	//point in space where the camera is focused, so that objects at this distance will be in focus

				//sample point on lens
				float r = sqrt(randf()) * settings.aperture_radius;
				float theta = 2 * M_PI * randf();

				float lens_x = r * cos(theta);
				float lens_y = r * sin(theta);

				vec3 lens_pos = camera_pos + lens_x * camera_right + lens_y * camera_up;

				primaryRay.o = lens_pos;
				primaryRay.d = normalize(focal_point - lens_pos);
			}


			// Intersect ray with scene
			if(intersect(primaryRay))
			{
				// If it hit something, evaluate the radiance from that point
				color = Li(primaryRay);

				//store normal & depth (for denoising...)
				Intersection first_hit = getIntersection(primaryRay);
				normal_buffer[pixel_idx] = first_hit.shading_normal;
				depth_buffer[pixel_idx] = length(first_hit.position - camera_pos);

			}
			else
			{
				// Otherwise evaluate environment
				color = Lenvironment(primaryRay.d);
			}
			//Firefly clamping
			float luminance = dot(color, vec3(0.2126f, 0.7152f, 0.0722f));
			float max_luminance = 10.0f;
			if (luminance > max_luminance && settings.firefly_clamping)
				color *= max_luminance / luminance;


			// Accumulate the obtained radiance to the pixels color
			float n = float(rendered_image.number_of_samples);
			rendered_image.data[pixel_idx] =
			    rendered_image.data[pixel_idx] * (n / (n + 1.0f))
			    + (1.0f / (n + 1.0f)) * color;
		}
	}
	rendered_image.number_of_samples += 1;
}


void applyBilateralFilter()
{
	int width = rendered_image.width;
	int height = rendered_image.height;

	filtered_image.width = width;
	filtered_image.height = height;
	filtered_image.data.resize(width * height);

	int pixel_step = 1; //3x3 filter
	float sigma_color = 0.05f; //tune this for more or less smoothing
	float sigma_space = 1.25f; //tune this for more or less smoothing
	float sigma_depth = 0.15f; //tune this for more or less smoothing
	float sigma_normal = 0.1f; //tune this for more or less smoothing

#pragma omp parallel for 
	for (int y = 0; y < height; y++) {
		for (int x = 0; x < width; x++) {
			vec3 sum = vec3(0.0f);
			float weight_sum = 0.0f;

			//find pixel's color, normal and depth to compare with neighbors (pixel in question is in the center of sqare filter)
			int center_idx = y * width + x;

			vec3 center_color = rendered_image.data[center_idx];
			vec3 center_normal = normal_buffer[center_idx];
			float center_depth = depth_buffer[center_idx];

			//loop through neighbors in the filter
			for (int dy = -pixel_step; dy <= pixel_step; dy++) {//from -2 to 2 if pixel_step is 2, so we get a 5x5 filter
				for (int dx = -pixel_step; dx <= pixel_step; dx++) {
					int neighbor_x = x + dx;
					int neighbor_y = y + dy;

					//check if neighbor is within image bounds
					if (neighbor_x < 0 || neighbor_x >= width || neighbor_y < 0 || neighbor_y >= height)
						continue; //continue to next neighbor if out of bounds

					int neighbor_idx = neighbor_y * width + neighbor_x;

					//get neighbor's color, normal and depth
					vec3 neighbor_color = rendered_image.data[neighbor_idx];
					vec3 neighbor_normal = normal_buffer[neighbor_idx];
					float neighbor_depth = depth_buffer[neighbor_idx];

					vec3 color_diff = neighbor_color - center_color;
					vec3 normal_diff = neighbor_normal - center_normal;
					float depth_diff = neighbor_depth - center_depth;
					vec2 space_diff = vec2(dx, dy);


					//calculate weights based on color, space, depth and normal differences
					float w_color = exp(-dot(color_diff, color_diff) / 2.0f * sigma_color * sigma_color);		//all weights are gaussian: 1/(2sigma^2)*e^(-x^2/(2sigma^2)) where x is the difference between the neighbor and the center pixel for the respective attribute (color, space, depth or normal) and sigma is a parameter (variance) that can be tuned to get more or less smoothing
					float w_space = exp(-dot(space_diff, space_diff) / 2.0f * sigma_space * sigma_space);
					float w_depth = exp(-depth_diff * depth_diff / 2.0f * sigma_depth * sigma_depth);
					float w_normal = exp(-dot(normal_diff, normal_diff) / 2.0f * sigma_normal * sigma_normal);

					w_color = 1.0f;
					//compute final weight and accumulate
					float weight = w_color * w_space * w_depth * w_normal;
					sum += neighbor_color * weight;
					weight_sum += weight;
				}
			}

			vec3 denoised_color = sum / std::max(weight_sum, 0.00001f);		//make sure not to devide by 0
			vec3 original_color = rendered_image.data[center_idx];

			float alpha = 0.25f;		//tune this for more or less smoothing, alpha is the blending factor between the original color and the denoised color, so that we can keep some of the details of the original image while still getting a smoother result

			filtered_image.data[center_idx] = vec4(mix(original_color, denoised_color, alpha), 1.0f);		//make sure not to devide by 0

		}
	}
}
}; // namespace pathtracer
