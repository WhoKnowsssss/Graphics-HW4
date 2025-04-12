#version 330

uniform vec3 u_cam_pos;
uniform vec3 u_light_pos;
uniform vec3 u_light_intensity;

uniform vec4 u_color;

uniform sampler2D u_texture_2;
uniform vec2 u_texture_2_size;

uniform float u_normal_scaling;
uniform float u_height_scaling;

in vec4 v_position;
in vec4 v_normal;
in vec4 v_tangent;
in vec2 v_uv;

out vec4 out_color;

float h(vec2 uv) {
  // You may want to use this helper function...
  return texture(u_texture_2, uv).r;
}

void main() {
  vec3 position = v_position.xyz;
  vec3 normal = normalize(v_normal.xyz);
  
  vec3 light_dir = u_light_pos - position;
  float distance = length(light_dir);
  light_dir = normalize(light_dir);
  
  vec3 view_dir = normalize(u_cam_pos - position);
  
  vec3 half_vector = normalize(light_dir + view_dir);

  // TBN matrix
  vec3 tangent = normalize(v_tangent.xyz);
  vec3 bitangent = normalize(cross(normal, tangent));
  mat3 tbn = mat3(tangent, bitangent, normal);

  // Get the bump map normal
  float du = 1.0 / u_texture_2_size.x;
  float dv = 1.0 / u_texture_2_size.y;
  float h_uv = h(v_uv);
  float h_uv_du = h(v_uv + vec2(du, 0));
  float h_uv_dv = h(v_uv + vec2(0, dv));
  float delta_u = (h_uv_du - h_uv) * u_height_scaling * u_normal_scaling;
  float delta_v = (h_uv_dv - h_uv) * u_height_scaling * u_normal_scaling;
  vec3 local_normal = normalize(vec3(delta_u, delta_v, 1));
  vec3 displaced_normal = normalize(tbn * local_normal);

  vec3 ambient_coefficient = vec3(0.1, 0.1, 0.1);  // ka
  vec3 diffuse_coefficient = vec3(0.7, 0.7, 0.7);  // kd
  vec3 specular_coefficient = vec3(1.0, 1.0, 1.0); // ks
  vec3 ambient_light = vec3(0.2, 0.2, 0.2);        // Ia
  float p = 32.0;                          // p
  
  float dot_nl = max(0.0, dot(displaced_normal, light_dir));
  float dot_nh = max(0.0, dot(displaced_normal, half_vector));
  
  vec3 ambient = ambient_coefficient * ambient_light;
  vec3 diffuse = diffuse_coefficient * (u_light_intensity / (distance * distance)) * dot_nl;
  vec3 specular = specular_coefficient * (u_light_intensity / (distance * distance)) 
                  * pow(dot_nh, p);
  
  vec3 final_color = ambient + diffuse + specular;
  
  final_color *= u_color.rgb;
  
  out_color.rgb = final_color;
  out_color.a = 1.0;
}

