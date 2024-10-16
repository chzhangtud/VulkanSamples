/* Copyright (c) 2023, Sascha Willems
 *
 * SPDX-License-Identifier: MIT
 *
 */

#version 460
#extension GL_EXT_ray_tracing : enable

struct RayPayload {
	vec3 color;
	float distance;
	vec3 normal;
	float reflector;
};

layout(location = 0) rayPayloadInEXT RayPayload rayPayload;

layout (binding = 6, set = 0) uniform samplerCube samplerEnvMap;

void main()
{
    rayPayload.color = texture(samplerEnvMap, normalize(gl_WorldRayDirectionEXT)).rgb;
    //rayPayload.color = vec3(gl_WorldRayDirectionEXT * 0.5 + 0.5);
}