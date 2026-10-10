#version 150

// AeroCore sea surface, per pixel (WaveSurfaceRenderer, WaterOptics; see docs/SEA.md, "Picture").
// Normal: the mesh's resolved slope plus every wave the mesh leaves out (the spectrum's waves shorter than four vertex
// spacings, then the wind ripples), each faded out once it spans fewer than about five pixels; what is faded goes into
// the slope variance of the sun glitter (Cox-Munk), so a pixel shows the mean of the facets it cannot resolve.
// Colour: Fresnel reflection of the sky (horizon colour to sky colour with the reflected ray's elevation) or of the
// banks where the reflected ray meets the surveyed terrain, the sun's glitter, the water body and the bed through
// exp(-2Kd) (as alpha); from below, the sky inside Snell's window. Foam is lit like a block face.

#moj_import <fog.glsl>

uniform vec4 ColorModulator;
uniform float FogStart;
uniform float FogEnd;
uniform vec4 FogColor;

// kx, kz (rad/m), amplitude (m), phase at the region origin now (rad); the spectrum first, then the ripples.
uniform vec4 Waves[96];
uniform int WaveCount;
uniform int SpectrumCount;
uniform vec3 SkyColor;
uniform vec3 HorizonColor;
uniform vec3 ShoreColor;
uniform vec3 SunDirection;
uniform float SunStrength;
// Development views (WaveSurfaceRenderer.debugView): 0 the picture; 1 spectrum gain / 3 (red), ripple gain (green),
// terrain horizon x 4 / (pi/2) (blue); 2 mesh cutoff wavelength (red 1 m ... white 8 m) and the pixel footprint (green, x 20 /m);
// 3 bed light (red), mesh slope x 5 (green), foam (blue); 4 drawn slope x 3 (red), glitter variance x 10 (green), lightmap (blue).
uniform int DebugView;
// Seconds for the foam lace's slow change, wrapped at 2 pi x 1000 s (its rates are multiples of 0.001 rad/s).
uniform float FoamTime;

in float vertexDistance;
in vec3 relativePosition;
in vec2 surfaceXZ;
in vec4 surfaceData;
in vec3 normalData;
in vec3 tint;
in float cutoff;
in vec4 lightColor;
in float skyVisible;

out vec4 fragColor;

const float PI = 3.14159265;
const float N_WATER = 1.333;
// Display choices, as in WaveSurfaceRenderer: the water body as a share of the biome tint, the sun over the sky
// radiance (sr), how white full foam draws.
const float BODY_SHADE = 0.42;
const float SUN_OVER_SKY = 25.0;
const float FOAM_WHITE = 0.85;
// Slope variance left when every wave is resolved: the sun's own disc (0.27 deg) and ripples under 1.7 cm.
const float MIN_VARIANCE = 2.0e-4;

float fresnel(float ci) {
    ci = clamp(ci, 0.0, 1.0);
    float ct = sqrt(max(0.0, 1.0 - (1.0 - ci * ci) / (N_WATER * N_WATER)));
    float rs = (ci - N_WATER * ct) / (ci + N_WATER * ct), rp = (N_WATER * ci - ct) / (N_WATER * ci + ct);
    return 0.5 * (rs * rs + rp * rp);
}

float fresnelFromWater(float ci) {
    ci = clamp(ci, 0.0, 1.0);
    float st2 = N_WATER * N_WATER * (1.0 - ci * ci);
    if (st2 >= 1.0) {
        return 1.0;
    }
    float ct = sqrt(1.0 - st2);
    float rs = (N_WATER * ci - ct) / (N_WATER * ci + ct), rp = (ci - N_WATER * ct) / (ci + N_WATER * ct);
    return 0.5 * (rs * rs + rp * rp);
}

// Foam lace. A foam value is the share of the surface covered (SeaFoam); real foam covers it as a reticulated network
// of bubble rafts and streaks with clear holes, so the pixel is white where a lace field lies under the threshold that
// whitens that share. The field: Worley cell edges (F2 - F1) at 0.45 m and 0.17 m, roughened by value noise and
// thinned by a 1.1 m patchiness; the threshold for a coverage c is a cubic fit to the field's quantiles (within 0.04;
// white share within 0.01 of c for c 0.05 to 0.95, checked offline). Feature points drift slowly (foam rafts reshape).
// Where a pixel spans more than the lace's detail, the mean (c itself) is drawn. A presentation model, not bubble physics.
float hash3(vec3 p) {
    p = fract(p * 0.3183099 + 0.1);
    p *= 17.0;
    return fract(p.x * p.y * p.z * (p.x + p.y + p.z));
}

float valueNoise(vec3 x) {
    vec3 i = floor(x), f = fract(x);
    f = f * f * (3.0 - 2.0 * f);
    return mix(mix(mix(hash3(i), hash3(i + vec3(1.0, 0.0, 0.0)), f.x), mix(hash3(i + vec3(0.0, 1.0, 0.0)), hash3(i + vec3(1.0, 1.0, 0.0)), f.x), f.y),
               mix(mix(hash3(i + vec3(0.0, 0.0, 1.0)), hash3(i + vec3(1.0, 0.0, 1.0)), f.x), mix(hash3(i + vec3(0.0, 1.0, 1.0)), hash3(i + vec3(1.0, 1.0, 1.0)), f.x), f.y), f.z);
}

float worleyEdge(vec2 p, float t) {
    vec2 cell = floor(p);
    float f1 = 9.0, f2 = 9.0;
    for (int dx = -1; dx <= 1; dx++) {
        for (int dz = -1; dz <= 1; dz++) {
            vec2 c = cell + vec2(float(dx), float(dz));
            float a = hash3(vec3(c, 0.0)), b = hash3(vec3(c + vec2(17.3, 31.7), 1.0));
            vec2 feature = c + 0.5 + 0.4 * vec2(sin(6.2831853 * a + 0.25 * t), cos(6.2831853 * b + 0.21 * t));
            float d = length(p - feature);
            if (d < f1) { f2 = f1; f1 = d; } else if (d < f2) { f2 = d; }
        }
    }
    return f2 - f1;
}

float foamLace(vec2 xz, float coverage, float footprint) {
    if (coverage <= 0.001) {
        return 0.0;
    }
    float detail = 1.0 - smoothstep(0.03, 0.12, footprint);
    if (detail <= 0.0) {
        return coverage;
    }
    const float CELL = 0.45;
    float t = FoamTime;
    float edge = 0.7 * worleyEdge(xz / CELL, t) + 0.3 * worleyEdge(xz / (0.38 * CELL) + vec2(19.7, 5.3), 1.3 * t);
    float rough = valueNoise(vec3(xz / (0.25 * CELL), 0.5 * t));
    vec3 q = vec3(xz / (2.5 * CELL), 0.1 * t);
    float patchiness = 0.5 * valueNoise(q) + 0.3 * valueNoise(q * 2.03 + 7.1) + 0.2 * valueNoise(q * 4.01 + 3.7);
    float field = edge + 0.15 * (rough - 0.5) + 0.7 * (0.5 - patchiness);
    float c = clamp(coverage, 0.0, 1.0), threshold = ((1.2496 * c - 1.6369) * c + 1.0729) * c - 0.0331;
    float lace = clamp((threshold - field) / 0.04 + 0.5, 0.0, 1.0);
    return mix(coverage, lace, detail);
}

// Smith's masking or shadowing term Lambda for Gaussian slopes of total variance v seen at cos(theta) from the mean
// normal: Walter et al. (2007, EGSR) rational fit, a = cot(theta) / sigma with sigma^2 = v / 2 per direction.
float smithLambda(float cosTheta, float v) {
    float c = clamp(cosTheta, 1.0e-4, 1.0), a = c / (sqrt(0.5 * v) * sqrt(1.0 - c * c) + 1.0e-6);
    return a < 1.6 ? (1.0 - 1.259 * a + 0.396 * a * a) / (3.535 * a + 2.181 * a * a) : 0.0;
}

void main() {
    vec3 toEye = -relativePosition;
    vec3 V = toEye / max(length(toEye), 1.0e-4);
    float nx = normalData.x, nz = normalData.z;
    vec3 meshNormal = vec3(nx, sqrt(max(1.0 - nx * nx - nz * nz, 0.0025)), nz);
    float sx = -meshNormal.x / meshNormal.y, sz = -meshNormal.z / meshNormal.y;
    float footprint = max(max(length(dFdx(surfaceXZ)), length(dFdy(surfaceXZ))), 1.0e-5);
    float gain = surfaceData.b * 3.0, rippleGain = normalData.y;
    float variance = MIN_VARIANCE;
    for (int i = 0; i < WaveCount; i++) {
        vec4 w = Waves[i];
        float k = length(w.xy), wavelength = 6.2831853 / k, a;
        if (i < SpectrumCount) {
            if (wavelength >= cutoff) {
                continue;
            }
            a = w.z * gain;
        } else {
            a = w.z * rippleGain;
        }
        float keep = smoothstep(2.0 * footprint, 5.0 * footprint, wavelength), slope = a * k;
        if (keep > 0.0) {
            float d = -keep * a * sin(dot(w.xy, surfaceXZ) + w.w);
            sx += d * w.x;
            sz += d * w.y;
        }
        variance += (1.0 - keep * keep) * 0.5 * slope * slope;
    }
    vec3 N = normalize(vec3(-sx, 1.0, -sz));
    vec3 body = tint * BODY_SHADE * lightColor.rgb;
    float foam = foamLace(surfaceXZ, surfaceData.r, footprint) * FOAM_WHITE;
    vec4 color;
    if (dot(V, meshNormal) >= 0.0) {
        // Normals of unresolved slopes can face away near grazing; hold those at a grazing reflection.
        float cosView = max(dot(N, V), 0.02), R = fresnel(cosView);
        vec3 reflected = reflect(-V, N);
        float elevation = asin(clamp(reflected.y, -1.0, 1.0)), horizon = surfaceData.a * 1.5707963;
        vec3 sky = mix(HorizonColor, SkyColor, smoothstep(0.0, 0.5, elevation)) * skyVisible;
        float banks = horizon > 0.002 ? 1.0 - smoothstep(horizon - 0.02, horizon + 0.02, elevation) : 0.0;
        vec3 environment = mix(sky, ShoreColor, banks);
        float through = surfaceData.g, alpha = max(R + (1.0 - R) * (1.0 - through), 1.0e-3);
        vec3 c = R * environment + (1.0 - R) * (1.0 - through) * body;
        float glint = 0.0, cosSun = dot(N, SunDirection);
        if (SunStrength > 0.0 && cosSun > 0.0) {
            vec3 H = normalize(V + SunDirection);
            float ct = clamp(dot(H, N), 1.0e-3, 1.0), tan2 = (1.0 - ct * ct) / (ct * ct);
            float density = exp(-tan2 / variance) / (PI * variance);
            // Facets hidden from the eye or the sun by the waves in front of them (Smith), which keeps the glitter
            // finite toward the horizon where Cox and Munk's formula alone grows as 1 / cos(view).
            float shadowing = 1.0 / (1.0 + smithLambda(cosView, variance) + smithLambda(cosSun, variance));
            glint = SUN_OVER_SKY * SunStrength * skyVisible * (1.0 - banks) * shadowing * fresnel(dot(H, V)) * density / (4.0 * ct * ct * ct * ct * cosView);
        }
        float glare = glint / (1.0 + glint);
        vec3 rgb = c / alpha + glare * vec3(1.0, 0.97, 0.9);
        alpha = min(1.0, alpha + glare);
        rgb = mix(rgb, lightColor.rgb, foam);
        color = vec4(min(rgb, vec3(1.0)), alpha + (1.0 - alpha) * foam);
    } else {
        float ri = fresnelFromWater(abs(dot(N, V)));
        vec3 rgb = mix((1.0 - ri) * SkyColor * skyVisible + ri * body, lightColor.rgb, foam);
        color = vec4(rgb, 0.85 + 0.15 * foam);
    }
    fragColor = linear_fog(color * ColorModulator, vertexDistance, FogStart, FogEnd, FogColor);
    if (DebugView == 1) {
        fragColor = vec4(gain / 3.0, rippleGain, min(surfaceData.a * 4.0, 1.0), 1.0);
    } else if (DebugView == 2) {
        fragColor = vec4(log2(cutoff) / 3.0, min(footprint * 20.0, 1.0), 0.0, 1.0);
    } else if (DebugView == 3) {
        fragColor = vec4(surfaceData.g, min(length(meshNormal.xz) / meshNormal.y * 5.0, 1.0), surfaceData.r, 1.0);
    } else if (DebugView == 4) {
        fragColor = vec4(min(length(vec2(sx, sz)) * 3.0, 1.0), min(variance * 10.0, 1.0), lightColor.b, 1.0);
    }
}
