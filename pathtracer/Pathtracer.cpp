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

		///////////////////////////////////////////////////////////////////
		// Calculate Direct Illumination from light.
		///////////////////////////////////////////////////////////////////
		Ray nray;				//Added for shadows
		nray.o = hit.position + hit.geometry_normal * EPSILON; //ray doesn't start exactly on the surface to avoid self-intersection
		nray.d = normalize(point_light.position - hit.position);
											//if occluded, the point is in shadow and we don't add any contribution from the light source
		const float distance_to_light = length(point_light.position - hit.position);
		const float falloff_factor = 1.0f / (distance_to_light * distance_to_light);
		vec3 Li = point_light.intensity_multiplier * point_light.color * falloff_factor;
		vec3 wi = normalize(point_light.position - hit.position);
		vec3 direct_illumination = mat.f(wi, hit.wo, hit.shading_normal) * Li * std::max(0.0f, dot(wi, hit.shading_normal));
		
		if (!occluded(nray)) {
			L += path_throughput * direct_illumination;
		}
		
		L += path_throughput * hit.material->m_emission;
		WiSample sample = mat.sample_wi(hit.wo, hit.shading_normal);

		if (sample.pdf < EPSILON || path_throughput == vec3(0))
			return L;

		float cosineterm = abs(dot(sample.wi, hit.shading_normal));
		path_throughput = path_throughput * (sample.f * cosineterm) / sample.pdf;


		Ray new_ray;
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
		new_ray.d = sample.wi;
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
	// Trace one path per pixel (the omp parallel stuf magically distributes the
	// pathtracing on all cores of your CPU).
	int num_rays = 0;
	vector<vec4> local_image(rendered_image.width * rendered_image.height, vec4(0.0f));

#pragma omp parallel for
	for(int y = 0; y < rendered_image.height; y++)
	{
		for(int x = 0; x < rendered_image.width; x++)
		{
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
			// Intersect ray with scene
			if(intersect(primaryRay))
			{
				// If it hit something, evaluate the radiance from that point
				color = Li(primaryRay);
			}
			else
			{
				// Otherwise evaluate environment
				color = Lenvironment(primaryRay.d);
			}
			// Accumulate the obtained radiance to the pixels color
			float n = float(rendered_image.number_of_samples);
			rendered_image.data[y * rendered_image.width + x] =
			    rendered_image.data[y * rendered_image.width + x] * (n / (n + 1.0f))
			    + (1.0f / (n + 1.0f)) * color;
		}
	}
	rendered_image.number_of_samples += 1;
}
}; // namespace pathtracer
