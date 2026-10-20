#version 150

// AeroCore 0.29: finished parts (PartShading). The vertex stage passes the raw colour, normal and position on; the
// lighting is per pixel (rendertype_aerocore_part.fsh). Positions are camera-relative world axes for things in the
// world, screen space for a GUI view (an orthographic projection), as Minecraft's entity shaders get them.
// A mesh kept on the GPU (PartGpu) comes in its own frame: PartPose takes it to camera-relative world axes, PartNormalMat
// its normals, and PartLight is the part's packed light; for vertices written each frame they are the identity and -1
// (the light from each vertex).

#moj_import <fog.glsl>

in vec3 Position;
in vec4 Color;
in vec2 UV0;
in ivec2 UV1;
in ivec2 UV2;
in vec3 Normal;

uniform sampler2D Sampler1;
uniform sampler2D Sampler2;

uniform mat4 ModelViewMat;
uniform mat4 ProjMat;
uniform int FogShape;
uniform mat4 PartPose;
uniform mat4 PartNormalMat;
uniform ivec2 PartLight;

out float vertexDistance;
out vec4 vertexColor;
out vec4 lightMapColor;
out vec4 overlayColor;
out vec2 texCoord0;
out vec3 normal;
out vec3 position;
out float skyLight;
out float blockLight;

void main() {
    vec3 world = (PartPose * vec4(Position, 1.0)).xyz;
    ivec2 light = PartLight.x >= 0 ? PartLight : UV2;
    gl_Position = ProjMat * ModelViewMat * vec4(world, 1.0);
    vertexDistance = fog_distance(world, FogShape);
    vertexColor = Color;
    lightMapColor = texelFetch(Sampler2, light / 16, 0);
    overlayColor = texelFetch(Sampler1, UV1, 0);
    texCoord0 = UV0;
    normal = mat3(PartNormalMat) * Normal;
    position = world;
    skyLight = clamp(float(light.y) / 240.0, 0.0, 1.0);
    blockLight = clamp(float(light.x) / 240.0, 0.0, 1.0);   // 0.39
}
