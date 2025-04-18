#version 330

uniform vec3 u_cam_pos;
uniform vec3 u_light_pos;
uniform vec3 u_light_intensity;

uniform sampler2D u_texture_1;

in vec4 v_position;
in vec4 v_normal;
in vec2 v_uv;

out vec4 out_color;

void main() {
  // Sample the texture at the UV coordinates
  vec4 tex_color = texture(u_texture_1, v_uv);
  
  // Output the sampled color
  out_color = tex_color;
  
  // Ensure alpha is set to 1
  out_color.a = 1.0;
}
