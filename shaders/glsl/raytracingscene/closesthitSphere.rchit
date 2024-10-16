#version 460
#extension GL_EXT_ray_tracing : require
#extension GL_EXT_nonuniform_qualifier : enable

struct RayPayload {
	vec3 color;
	float distance;
	vec3 normal;
	float reflector;
};

layout(location = 0) rayPayloadInEXT RayPayload rayPayload;
layout(location = 2) rayPayloadEXT bool shadowed;
hitAttributeEXT vec2 attribs;

layout(binding = 0, set = 0) uniform accelerationStructureEXT topLevelAS;

struct Sphere {
	vec3 center;
	float radius;
	vec4 color;
};
layout(binding = 7, set = 0) buffer Spheres { Sphere s[]; } spheres;

void main()
{
	Sphere sphere = spheres.s[gl_PrimitiveID];

	vec3 worldPos = gl_WorldRayOriginEXT + gl_WorldRayDirectionEXT * gl_HitTEXT;
	vec3 worldNrm = normalize(worldPos - sphere.center);

	// Basic lighting
	vec3 lightVector = normalize(vec3(1.0, 1.0, 1.0));
	float dot_product = max(dot(lightVector, worldNrm), 0.2);
	rayPayload.color = sphere.color.rgb * dot_product;

	rayPayload.distance = gl_RayTmaxEXT;
	rayPayload.normal = worldNrm;
	rayPayload.reflector = gl_PrimitiveID == 1 ? 1.0 : 0.0;
}