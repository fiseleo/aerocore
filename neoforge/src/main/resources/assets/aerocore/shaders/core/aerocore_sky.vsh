#version 150

// A full-screen triangle in clip space; the fragment finds its view ray from the inverse view-projection.
in vec3 Position;

out vec2 clip;

void main() {
    clip = Position.xy;
    gl_Position = vec4(Position.xy, 1.0, 1.0);
}
