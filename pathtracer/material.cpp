#include "material.h"
#include "sampling.h"
#include "labhelper.h"

using namespace labhelper;

namespace pathtracer
{
WiSample sampleHemisphereCosine(const vec3& wo, const vec3& n)
{
	mat3 tbn = tangentSpace(n);
	vec3 sample = cosineSampleHemisphere();
	WiSample r;
	r.wi = tbn * sample;
	if(dot(r.wi, n) > 0.0f)
		r.pdf = max(0.0f, dot(r.wi, n)) / M_PI;
	return r;
}

///////////////////////////////////////////////////////////////////////////
// A Lambertian (diffuse) material
///////////////////////////////////////////////////////////////////////////
vec3 Diffuse::f(const vec3& wi, const vec3& wo, const vec3& n) const
{
	if(dot(wi, n) <= 0.0f)
		return vec3(0.0f);
	if(!sameHemisphere(wi, wo, n))
		return vec3(0.0f);
	return (1.0f / M_PI) * color;
}

WiSample Diffuse::sample_wi(const vec3& wo, const vec3& n) const
{
	WiSample r = sampleHemisphereCosine(wo, n);
	r.f = f(r.wi, wo, n);
	return r;
}

vec3 MicrofacetBRDF::f(const vec3& wi, const vec3& wo, const vec3& n) const
{
	if (!sameHemisphere(wi, wo, n))		//reject directions below the surface
		return vec3(0.0f);
	vec3 wh = normalize(wi + wo);

	vec3 N;
	if (dot(wo, n) > 0.0f)
	{
		N = n;
	}
	else
	{
		N = -n;
	}
	
	float n_dot_wi = max(0.0001f, dot(N, wi));
	float n_dot_wo = max(0.0001f, dot(N, wo));
	float wo_dot_wh = max(0.0001f, dot(wo, wh));
	float n_dot_wh = max(0.0001f, dot(N, wh));


	float D = (shininess + 2) / (2 * M_PI) * pow(n_dot_wh, shininess);
	float G = min(1.0f, min(2 * n_dot_wh * n_dot_wo / wo_dot_wh, 2 * n_dot_wh * n_dot_wi / wo_dot_wh));
	float denominator = 4 * n_dot_wi * n_dot_wo;
	vec3 BRDF = vec3(D * G / denominator);

	return BRDF;
}

WiSample MicrofacetBRDF::sample_wi(const vec3& wo, const vec3& n) const
{
	//Generate half angle wh								//ADDED to sample a half angle wh according to the distribution of the microfacet normals
	vec3 N;
	if (dot(wo, n) > 0.0f)
	{
		N = n;
	}
	else
	{
		N = -n;
	}
	WiSample r;
	vec3 tangent = normalize(perpendicular(N));
	vec3 bitangent = normalize(cross(tangent, N));
	float phi = 2.0f * M_PI * randf();
	float cos_theta = pow(randf(), 1.0f / (shininess + 1));
	float sin_theta = sqrt(max(0.0f, 1.0f - cos_theta * cos_theta));
	vec3 wh = normalize(sin_theta * cos(phi) * tangent +
						sin_theta * sin(phi) * bitangent +
						cos_theta * N);

	vec3 wi = reflect(-wo, wh);
	
	if (dot(wi, N) <= 0.0f || dot(wo, wh) <= 0.0f) {		//reject bad samples
		r.wi = vec3(0.0);
		r.pdf = 0.0;
		r.f = vec3(0.0);
		return r;
	}

	float p_wh = (this->shininess + 1.0f) * pow(max(0.0f, dot(N, wh)), this->shininess) / (2.0f * M_PI);	
	float wh_dot_wo = max(0.0001f, dot(wh, wo));
	float p_wi = p_wh / (4.0f * wh_dot_wo);
	r.wi = wi;

	r.f = f(r.wi, wo, N);
	r.pdf = p_wi;



	return r;
}


float BSDF::fresnel(const vec3& wi, const vec3& wo) const			//ADDED to calculate the fresnel term for the dielectric bsdf
{
	float R0 = this->R0;
	vec3 wh = normalize(wi + wo);
	float wh_dot_wo = dot(wh, wo);			//needed?
	float F = R0 + (1.0f - R0) * pow(1.0f - wh_dot_wo, 5);
	return clamp(F, 0.0f, 1.0f);
}


vec3 DielectricBSDF::f(const vec3& wi, const vec3& wo, const vec3& n) const		//ADDED to calculate the value of the dielectric bsdf for specific directions
{
	vec3 BSDF;
	float fresnel = this->fresnel(wi, wo);
	BSDF = fresnel * reflective_material->f(wi, wo, n) + (1.0f - fresnel) * transmissive_material->f(wi, wo, n);

	return BSDF;
}

WiSample DielectricBSDF::sample_wi(const vec3& wo, const vec3& n) const
{
	vec3 N;
	if (dot(wo, n) > 0.0f)
	{
		N = n;
	}
	else
	{
		N = -n;
	}
	WiSample r;
	float F = this->fresnel(r.wi, wo);

	if(randf()<0.5f)
	{
		r = reflective_material->sample_wi(wo, N);
		r.pdf *= 0.5f;
		r.f *= F;
	}
	else
	{
		r = transmissive_material->sample_wi(wo, n);	//original normal so that transmissive_material can determine what side of the object is the outside
		r.pdf *= 0.5f;
		r.f *= (1.0f - F);
	}

	return r;
}

vec3 MetalBSDF::f(const vec3& wi, const vec3& wo, const vec3& n) const		//ADDED to calculate the value of the metal bsdf for specific directions
{
	vec3 BSDF;
	float fresnel = this->fresnel(wi, wo);
	BSDF = fresnel * reflective_material->f(wi, wo, n) * color;		//multiplied by color because metal absorbs some light

	return BSDF;
}

WiSample MetalBSDF::sample_wi(const vec3& wo, const vec3& n) const
{
	WiSample r;
	r = reflective_material->sample_wi(wo, n);
	float fresnel = this->fresnel(r.wi, wo);
	r.f = r.f*fresnel*color;
	return r;
}


vec3 BSDFLinearBlend::f(const vec3& wi, const vec3& wo, const vec3& n) const		//ADDED to calculate the value of the linear blend bsdf for specific directions
{
	vec3 blend = w * bsdf0->f(wi, wo, n) + (1.0f - w) * bsdf1->f(wi, wo, n);
	return blend;
}

WiSample BSDFLinearBlend::sample_wi(const vec3& wo, const vec3& n) const
{
	WiSample r;
	if (randf() < w)
	{
		r = bsdf0->sample_wi(wo, n);
	}
	else
	{
		r = bsdf1->sample_wi(wo, n);
	}

	return r;
}


#if SOLUTION_PROJECT == PROJECT_REFRACTIONS
///////////////////////////////////////////////////////////////////////////
// A perfect specular refraction.
///////////////////////////////////////////////////////////////////////////
vec3 GlassBTDF::f(const vec3& wi, const vec3& wo, const vec3& n) const
{
	if(sameHemisphere(wi, wo, n))
	{
		return vec3(0);
	}
	else
	{
		return vec3(1);
	}
}

WiSample GlassBTDF::sample_wi(const vec3& wo, const vec3& n) const
{
	WiSample r;

	float eta;
	glm::vec3 N;
	if(dot(wo, n) > 0.0f)
	{
		N = n;
		eta = 1.0f / ior;
	}
	else
	{
		N = -n;
		eta = ior;
	}

	// Alternatively:
	// d = dot(wo, N)
	// k = d * d (1 - eta*eta)
	// wi = normalize(-eta * wo + (d * eta - sqrt(k)) * N)

	// or

	// d = dot(n, wo)
	// k = 1 - eta*eta * (1 - d * d)
	// wi = - eta * wo + ( eta * d - sqrt(k) ) * N

	float w = dot(wo, N) * eta;
	float k = 1.0f + (w - eta) * (w + eta);
	if(k < 0.0f)
	{
		// Total internal reflection
		r.wi = reflect(-wo, N);
	}
	else
	{
		k = sqrt(k);
		r.wi = normalize(-eta * wo + (w - k) * N);
	}
	r.pdf = abs(dot(r.wi, n));
	r.f = vec3(1.0f, 1.0f, 1.0f);

	return r;
}

vec3 BTDFLinearBlend::f(const vec3& wi, const vec3& wo, const vec3& n) const
{
	return w * btdf0->f(wi, wo, n) + (1.0f - w) * btdf1->f(wi, wo, n);
}

WiSample BTDFLinearBlend::sample_wi(const vec3& wo, const vec3& n) const
{
	if(randf() < w)
	{
		WiSample r = btdf0->sample_wi(wo, n);
		return r;
	}
	else
	{
		WiSample r = btdf1->sample_wi(wo, n);
		return r;
	}
}

#endif
} // namespace pathtracer
