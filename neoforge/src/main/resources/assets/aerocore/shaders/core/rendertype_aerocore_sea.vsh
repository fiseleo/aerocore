#version 150

// AeroCore sea surface (WaveSurfaceRenderer). The mesh carries the resolved waves; the vertex attributes are packed:
//   Color   r foam, g light returned from the bed (exp(-2Kd)), b spectrum gain / 3, a terrain horizon / (pi/2)
//   UV0     surface x, z minus the region origin, m
//   UV1     biome water tint: x = R * 128 + G / 2, y = (B / 2) * 256 + log2(cutoff wavelength, m)
//   UV2     lightmap
//   Normal  x, z of the mesh normal; y the ripple gain (wind sheltering)

#moj_import <light.glsl>
#moj_import <fog.glsl>

in vec3 Position;
in vec4 Color;
in vec2 UV0;
in ivec2 UV1;
in ivec2 UV2;
in vec3 Normal;

uniform sampler2D Sampler2;

uniform mat4 ModelViewMat;
uniform mat4 ProjMat;
uniform int FogShape;

out float vertexDistance;
out vec3 relativePosition;
out vec2 surfaceXZ;
out vec4 surfaceData;
out vec3 normalData;
out vec3 tint;
out float cutoff;
out vec4 lightColor;
out float skyVisible;

void main() {
    gl_Position = ProjMat * ModelViewMat * vec4(Position, 1.0);
    vertexDistance = fog_distance(Position, FogShape);
    relativePosition = Position;
    surfaceXZ = UV0;
    surfaceData = Color;
    normalData = Normal;
    tint = vec3(float(UV1.x / 128), float((UV1.x % 128) * 2), float((UV1.y / 256) * 2)) / 255.0;
    cutoff = exp2(float(UV1.y % 256));
    lightColor = minecraft_sample_lightmap(Sampler2, UV2);
    skyVisible = clamp(float(UV2.y) / 240.0, 0.0, 1.0);
}
