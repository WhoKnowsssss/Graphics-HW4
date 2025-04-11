#version 330

uniform vec4 u_color;
uniform vec3 u_cam_pos;
uniform vec3 u_light_pos;
uniform vec3 u_light_intensity;

in vec4 v_position;
in vec4 v_normal;
in vec2 v_uv;

out vec4 out_color;

void main() {
  vec3 position = v_position.xyz;
  vec3 normal = normalize(v_normal.xyz);
  
  vec3 light_dir = u_light_pos - position;
  float distance = length(light_dir);
  light_dir = normalize(light_dir);
  
  vec3 view_dir = normalize(u_cam_pos - position);
  
  vec3 half_vector = normalize(light_dir + view_dir);
  
  vec3 ambient_coefficient = vec3(0.1, 0.1, 0.1);  // ka
  vec3 diffuse_coefficient = vec3(0.7, 0.7, 0.7);  // kd
  vec3 specular_coefficient = vec3(1.0, 1.0, 1.0); // ks
  vec3 ambient_light = vec3(0.2, 0.2, 0.2);        // Ia
  float p = 32.0;                          // p
  
  float dot_nl = max(0.0, dot(normal, light_dir));
  float dot_nh = max(0.0, dot(normal, half_vector));
  
  vec3 ambient = ambient_coefficient * ambient_light;
  vec3 diffuse = diffuse_coefficient * (u_light_intensity / (distance * distance)) * dot_nl;
  vec3 specular = specular_coefficient * (u_light_intensity / (distance * distance)) 
                  * pow(dot_nh, p);
  
  vec3 final_color = ambient + diffuse + specular;
  
  final_color *= u_color.rgb;
  
  out_color.rgb = final_color;
  out_color.a = 1.0;
}

