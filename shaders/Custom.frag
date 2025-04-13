#version 330

// (Every uniform is available here.)

uniform mat4 u_view_projection;
uniform mat4 u_model;

uniform float u_normal_scaling;
uniform float u_height_scaling;

uniform vec3 u_cam_pos;
uniform vec3 u_light_pos;
uniform vec3 u_light_intensity;

// Feel free to add your own textures. If you need more than 4,
// you will need to modify the skeleton.
uniform sampler2D u_texture_1;
uniform sampler2D u_texture_2;
uniform sampler2D u_texture_3;
uniform sampler2D u_texture_4;

// Environment map! Take a look at GLSL documentation to see how to
// sample from this.
uniform samplerCube u_texture_cubemap;

in vec4 v_position;
in vec4 v_normal;
in vec4 v_tangent;
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
  
  vec3 ambient_coefficient = vec3(0.5, 0.5, 0.5);  // ka
  vec3 diffuse_coefficient = vec3(0.7, 0.7, 0.7);  // kd
  vec3 specular_coefficient = vec3(1.0, 1.0, 1.0); // ks
  vec3 ambient_light = vec3(0.8, 0.8, 0.8);        // Ia
  float p = 32.0;                          // p
  
  float dot_nl = max(0.0, dot(normal, light_dir));
  float dot_nh = max(0.0, dot(normal, half_vector));
  
  vec3 ambient = ambient_coefficient * ambient_light;
  vec3 diffuse = diffuse_coefficient * (u_light_intensity / (distance * distance)) * dot_nl;
  vec3 specular = specular_coefficient * (u_light_intensity / (distance * distance)) 
                  * pow(dot_nh, p);
  
  vec3 final_color = ambient + diffuse + specular;
  
  vec4 tex_color = texture(u_texture_1, v_uv);

  final_color *= tex_color.rgb;
  
  out_color.rgb = final_color;
  out_color.a = 1.0;
}
