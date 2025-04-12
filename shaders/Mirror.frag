#version 330


uniform vec3 u_cam_pos;

uniform samplerCube u_texture_cubemap;

in vec4 v_position;
in vec4 v_normal;
in vec4 v_tangent;

out vec4 out_color;

void main() {
  // YOUR CODE HERE

  vec3 view_dir = normalize(u_cam_pos - v_position.xyz);
  vec3 reflect_dir = reflect(view_dir, v_normal.xyz);
  vec3 reflect_color = texture(u_texture_cubemap, reflect_dir).rgb;
  out_color.rgb = reflect_color;
  out_color.a = 1.0;
}
