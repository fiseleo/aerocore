#version 150

// AeroCore 0.29: finished parts (PartShading). Minecraft's two-light diffuse (so parts sit with other entities) on
// the texture times the vertex colour, plus a key light's highlight (Blinn-Phong, Schlick Fresnel) and the sky
// reflected (a gradient: metals reflect it strongly, glossy paint a little). Gloss per pixel is the texture's alpha
// (0.5 + 0.5 gloss, so vanilla cutout shaders never drop a pixel); metalness and a gloss scale come per finish
// (PartSurface). In the world the key light is the sun (or the moon), seen as much as the sky is (the sky light);
// a GUI view (orthographic) is a studio: its first light and fixed studio colours.
// 0.30: a lacquer (clearcoat) over paint, pearl paint's colour flip, cut-out pixels (lettering) and glass growing
// opaque toward grazing views (Fresnel), per finish (PartSurface z, w; PartSurface2 x, y).
// 0.38, only for finishes that ask (PartSurface3, PartTint; the others draw exactly as before):
// - a normal map (Sampler3: red and green the tangent-space normal's x along +u and y along +v, z rebuilt; blue a
//   cavity term darkening crevices), its frame built per pixel from the derivatives of position and UV (no tangents in
//   the vertex format), strength x;
// - reflective glass (y): what it reflects is added on top of what shows through, not faded with the glass's alpha
//   (out alpha = a + (1 - a) R, colour = (a base + reflection) / out alpha, R the dielectric Fresnel reflectance), so
//   even clear glass shows the sky at a slant; drawn opaque far away (MeshBaker.shut, 0.55 of its colour), its colour
//   is scaled by z;
// - a tint and the gloss scale (PartSurface y), for "shading": 2.
// 0.39, only while beautified shading is on (PartGlobal x; off, every statement below runs as in 0.38): block light
// (torches, lamps) warms what it lights at night and gives glossy surfaces a warm sheen as from overhead lights
// (BlockLightColor, PartGlobal y its strength, z how much day it is).

#moj_import <fog.glsl>

uniform sampler2D Sampler0;
uniform sampler2D Sampler3;

uniform vec4 ColorModulator;
uniform float FogStart;
uniform float FogEnd;
uniform vec4 FogColor;
uniform mat4 ProjMat;
uniform vec3 Light0_Direction;
uniform vec3 Light1_Direction;

uniform vec4 PartSurface;     // x metalness 0-1, y gloss scale, z clearcoat 0-1, w pearl flip 0-1
uniform vec4 PartSurface2;    // x cutout alpha (0: none), y glass Fresnel 0-1
uniform vec4 PartSurface3;    // x normal map strength (0: none), y reflective glass (0 or 1), z opaque glass brightness factor
uniform vec3 PartTint;
uniform vec3 SunDirection;    // world, toward the sun (or the moon at night)
uniform vec3 SunColor;        // its colour times its strength (0 when below the horizon)
uniform vec3 SkyColor;
uniform vec3 HorizonColor;
uniform vec3 GroundColor;
uniform vec4 PartGlobal;      // 0.39: x beautified shading (0 or 1), y block light sheen, z share of the day 0-1
uniform vec3 BlockLightColor;

in float vertexDistance;
in vec4 vertexColor;
in vec4 lightMapColor;
in vec4 overlayColor;
in vec2 texCoord0;
in vec3 normal;
in vec3 position;
in float skyLight;
in float blockLight;

out vec4 fragColor;

vec3 environment(float up, float roughness, vec3 sky, vec3 horizon, vec3 ground) {
    vec3 c = up > 0.0 ? mix(horizon, sky, smoothstep(0.0, 0.7, up)) : mix(horizon, ground, smoothstep(0.0, 0.25, -up));
    // rough surfaces see a blur of the whole sky
    vec3 blurred = 0.45 * sky + 0.35 * horizon + 0.2 * ground;
    return mix(c, blurred, roughness * roughness);
}

void main() {
    vec4 tex = texture(Sampler0, texCoord0);
    if (PartSurface2.x > 0.0 && tex.a < PartSurface2.x) { discard; }
    vec3 albedo = tex.rgb * vertexColor.rgb * PartTint;
    float gloss = clamp((tex.a - 0.5) * 2.0 * PartSurface.y, 0.0, 1.0);
    float metal = clamp(PartSurface.x, 0.0, 1.0);
    bool studio = ProjMat[3][3] > 0.5;
    vec3 n = normalize(normal);
    vec3 v = studio ? vec3(0.0, 0.0, 1.0) : normalize(-position);
    if (dot(n, v) < 0.0) { n = -n; }   // a two-sided face seen from behind
    // 0.38: the normal map, in a frame from the screen-space derivatives of position and UV (Mikkelsen's cotangent frame)
    float cavity = 1.0;
    if (PartSurface3.x > 0.0) {
        vec3 map = texture(Sampler3, texCoord0).rgb;
        vec3 dp1 = dFdx(position), dp2 = dFdy(position);
        vec2 duv1 = dFdx(texCoord0), duv2 = dFdy(texCoord0);
        vec3 dp2perp = cross(dp2, n), dp1perp = cross(n, dp1);
        vec3 t = dp2perp * duv1.x + dp1perp * duv2.x;
        vec3 b = dp2perp * duv1.y + dp1perp * duv2.y;
        float span = max(dot(t, t), dot(b, b));
        if (span > 1e-24) {
            float inv = inversesqrt(span);
            vec2 xy = (map.rg * 2.0 - 1.0) * PartSurface3.x;
            float z = sqrt(max(0.0, 1.0 - dot(xy, xy)));
            vec3 bent = normalize(t * inv * xy.x + b * inv * xy.y + n * z);
            if (dot(bent, v) > 0.02) { n = bent; }
            cavity = mix(1.0, map.b, min(PartSurface3.x, 1.0));
        }
    }
    float ndv0 = max(dot(n, v), 0.0);
    // pearl: the paint's colour turns toward its rotated hue at grazing views
    float flip = clamp(PartSurface.w, 0.0, 1.0) * (1.0 - ndv0) * (1.0 - ndv0);
    albedo = mix(albedo, albedo.brg * 0.85 + 0.08, flip);

    // Minecraft's diffuse: two fixed lights and an ambient floor, times the light map (block and sky light)
    float l0 = max(0.0, dot(Light0_Direction, n)), l1 = max(0.0, dot(Light1_Direction, n));
    float diffuse = min(1.0, (l0 + l1) * 0.6 + 0.4);
    vec3 base = albedo * (1.0 - 0.55 * metal) * diffuse * lightMapColor.rgb * cavity;

    // Fresnel: dielectrics reflect 4 % head on, metals their own colour
    vec3 f0 = mix(vec3(0.04), albedo, metal);
    float ndv = max(dot(n, v), 0.0);
    vec3 fresnel = f0 + (vec3(1.0) - f0) * pow(1.0 - ndv, 5.0);

    // the key light's highlight
    vec3 s = studio ? normalize(Light0_Direction) : SunDirection;
    vec3 light = studio ? vec3(0.9) : SunColor;
    float seen = studio ? 1.0 : skyLight * skyLight;
    vec3 h = normalize(s + v);
    float shininess = exp2(10.0 * gloss + 1.0);
    float lobe = pow(max(dot(n, h), 0.0), shininess) * (shininess + 8.0) / 32.0;
    vec3 specular = fresnel * lobe * light * seen * step(0.0, dot(n, s)) * (0.2 + 0.8 * gloss);

    // 0.39 beautified: block light warms what it lights (less by day) and puts a warm sheen on glossy surfaces
    if (PartGlobal.x > 0.5 && !studio) {
        float bl = blockLight * blockLight, night = 1.0 - 0.8 * PartGlobal.z;
        base *= mix(vec3(1.0), BlockLightColor * 1.15, 0.25 * bl * night);
        vec3 hb = normalize(vec3(0.0, 1.0, 0.0) + v);
        float sheen = pow(max(dot(n, hb), 0.0), 0.5 * shininess) * (0.5 * shininess + 8.0) / 32.0;
        specular += fresnel * BlockLightColor * (bl * blockLight * night * sheen * (0.2 + 0.8 * gloss) * PartGlobal.y);
    }

    // the surroundings reflected (a GUI's y points down the screen)
    vec3 r = reflect(-v, n);
    float up = studio ? -r.y : r.y;
    vec3 sky = studio ? environment(up, 1.0 - gloss, vec3(0.78), vec3(0.42), vec3(0.12))
                      : environment(up, 1.0 - gloss, SkyColor, HorizonColor, GroundColor);
    float amount = (metal * 0.6 + (1.0 - metal) * 0.3 * gloss) * (studio ? 1.0 : skyLight);
    vec3 reflected = fresnel * sky * amount;

    // a lacquer over the paint: a sharp white highlight and a clear reflection, 4 % head on
    float coat = clamp(PartSurface.z, 0.0, 1.0);
    if (coat > 0.0) {
        float fc = 0.04 + 0.96 * pow(1.0 - ndv, 5.0);
        float sharp = pow(max(dot(n, h), 0.0), 900.0) * 28.0;
        vec3 clearSky = studio ? environment(up, 0.05, vec3(0.78), vec3(0.42), vec3(0.12)) : environment(up, 0.05, SkyColor, HorizonColor, GroundColor);
        specular += coat * fc * sharp * light * seen * step(0.0, dot(n, s));
        reflected += coat * fc * clearSky * (studio ? 1.0 : skyLight);
    }

    float alpha = vertexColor.a;
    vec3 color;
    if (PartSurface3.y > 0.0) {
        // 0.38 reflective glass: the clear sky and the sun's glint added over what shows through (see the notes above)
        float rf = 0.04 + 0.96 * pow(1.0 - ndv, 5.0);
        vec3 clearSky = studio ? environment(up, 0.03, vec3(0.78), vec3(0.42), vec3(0.12)) : environment(up, 0.03, SkyColor, HorizonColor, GroundColor);
        vec3 mirror = rf * clearSky * (studio ? 1.0 : skyLight) + specular;
        if (alpha >= 0.999) {
            color = base * PartSurface3.z + mirror;   // drawn opaque far away: a pane of the tint over a dim inside
        } else {
            float outA = alpha + (1.0 - alpha) * rf;
            color = (alpha * base + mirror) / max(outA, 0.001);
            alpha = outA;
        }
        color *= ColorModulator.rgb;
    } else {
        color = (base + specular + reflected) * ColorModulator.rgb;
        // glass: more opaque, and more of a mirror, toward grazing views
        float glassFresnel = clamp(PartSurface2.y, 0.0, 1.0);
        if (glassFresnel > 0.0) { alpha = mix(alpha, 1.0, glassFresnel * pow(1.0 - ndv, 3.0)); }
    }
    color = mix(overlayColor.rgb, color, overlayColor.a);
    fragColor = linear_fog(vec4(color, alpha * ColorModulator.a), vertexDistance, FogStart, FogEnd, FogColor);
}
