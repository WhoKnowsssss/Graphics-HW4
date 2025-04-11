#version 330

// The camera's position in world-space
uniform vec3 u_cam_pos;

// Color
uniform vec4 u_color;

// Properties of the single point light
uniform vec3 u_light_pos;
uniform vec3 u_light_intensity;

// We also get the uniform texture we want to use.
uniform sampler2D u_texture_1;

// These are the inputs which are the outputs of the vertex shader.
in vec4 v_position;
in vec4 v_normal;

// This is where the final pixel color is output.
// Here, we are only interested in the first 3 dimensions (xyz).
// The 4th entry in this vector is for "alpha blending" which we
// do not require you to know about. For now, just set the alpha
// to 1.
out vec4 out_color;

void main() {
  vec3 position = v_position.xyz;
  vec3 normal = normalize(v_normal.xyz);

  vec3 light_dir = u_light_pos - position;
  float distance = length(light_dir);
  light_dir = normalize(light_dir);
  float dot_nl = max(0.0, dot(normal, light_dir));

  vec3 diffuse_coefficient = vec3(1.0, 1.0, 1.0);
  vec3 diffuse = diffuse_coefficient * (u_light_intensity / (distance * distance)) * dot_nl;

  out_color.rgb = diffuse;
  out_color.a = 1.0;
}
