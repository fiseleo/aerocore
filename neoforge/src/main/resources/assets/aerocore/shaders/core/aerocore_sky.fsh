#version 150

// AeroCore daylight sky (SkyRenderer, SkyModel): Preetham, Shirley and Smits (1999) clear sky - Perez distribution of
// luminance and CIE chromaticity scaled to the zenith - blended toward the CIE overcast sky by the rain, converted
// from xyY to linear sRGB, exposed for the hemisphere's mean (Exposure, relative to the zenith), dimmed by Daylight
// (the game's own day curve) and the storm, tone mapped on luminance (out-of-gamut colours desaturated, highlights rolled
// toward white; SkyRenderer.display), then sRGB-encoded. Directions below the horizon take the horizon's colour.
uniform mat4 InverseViewProjection;
uniform vec3 PerezA;
uniform vec3 PerezB;
uniform vec3 PerezC;
uniform vec3 PerezD;
uniform vec3 PerezE;
uniform vec3 Zenith;
uniform vec3 Normalizer;
uniform vec3 SunDirection;
uniform float Exposure;
uniform float Daylight;
uniform float Overcast;
uniform float Storm;

in vec2 clip;

out vec4 fragColor;

vec3 perez(float theta, float gamma) {
    float cg = cos(gamma);
    return (1.0 + PerezA * exp(PerezB / cos(theta))) * (1.0 + PerezC * exp(PerezD * gamma) + PerezE * cg * cg);
}

vec3 linearRgb(float x, float y, float luminance) {
    float yy = max(y, 1.0e-6), bigX = x / yy * luminance, bigZ = (1.0 - x - y) / yy * luminance;
    return vec3(3.2406 * bigX - 1.5372 * luminance - 0.4986 * bigZ,
                -0.9689 * bigX + 1.8758 * luminance + 0.0415 * bigZ,
                0.0557 * bigX - 0.2040 * luminance + 1.0570 * bigZ);
}

vec3 display(vec3 lin) {
    float l = dot(lin, vec3(0.2126, 0.7152, 0.0722));
    if (l <= 0.0) {
        return vec3(0.0);
    }
    float m = min(lin.r, min(lin.g, lin.b));
    vec3 c = m < 0.0 ? l + (lin - l) * (l / (l - m)) : lin;
    c *= (1.0 - exp(-l)) / l;
    float top = max(c.r, max(c.g, c.b));
    if (top > 1.0) {
        float w = min(top - 1.0, 1.0);
        c = c / top * (1.0 - w) + w;
    }
    return c;
}

float encode(float v) {
    v = clamp(v, 0.0, 1.0);
    return v <= 0.0031308 ? 12.92 * v : 1.055 * pow(v, 1.0 / 2.4) - 0.055;
}

void main() {
    vec4 world = InverseViewProjection * vec4(clip, 1.0, 1.0);
    vec3 dir = normalize(world.xyz / world.w);
    float theta = acos(clamp(dir.y, 0.0017, 1.0));
    float gamma = acos(clamp(dot(dir, SunDirection), -1.0, 1.0));
    vec3 yxy = Zenith * perez(theta, gamma) / Normalizer;
    vec3 clear = linearRgb(yxy.y, yxy.z, yxy.x / Zenith.x);
    // The overcast sky in the same display units as the clear sky's mean (0.55 of it at its zenith).
    vec3 overcast = vec3((1.0 + 2.0 * cos(theta)) / 3.0) * 0.55 * 0.5 / Exposure;
    vec3 mapped = display(mix(clear, overcast, Overcast) * Exposure * Daylight * (1.0 - 0.5 * Storm));
    fragColor = vec4(encode(mapped.r), encode(mapped.g), encode(mapped.b), 1.0);
}
