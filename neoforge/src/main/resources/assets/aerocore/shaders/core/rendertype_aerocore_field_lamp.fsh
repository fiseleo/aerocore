#version 150

// AeroCore 0.38: field lights. Added light: no fog (how far a light is seen is Allard's law in FieldLamps) and no
// alpha test (the alpha is 1/255 so Fabulous graphics keep the layer; vanilla's position_tex_color drops it below 0.1).

uniform sampler2D Sampler0;
uniform vec4 ColorModulator;

in vec2 texCoord0;
in vec4 vertexColor;

out vec4 fragColor;

void main() {
    fragColor = texture(Sampler0, texCoord0) * vertexColor * ColorModulator;
}
