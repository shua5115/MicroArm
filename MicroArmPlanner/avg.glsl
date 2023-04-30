uniform sampler2D texture;

varying vec4 vertColor;
varying vec4 vertTexCoord;

uniform vec3 size;
uniform vec3 dir; // vector in the direction the average should be taken, with length of how wide the average should be

void main() {
  vec2 uv = vertTexCoord.xy;
  float dir_length = length(dir.xy); // number of steps
  vec2 step = vec2(1.0, 1.0)/size.xy; // size of each pixel in uv coords
  step *= dir.xy/dir_length;
  uv -= step*dir_length*0.5; 
  vec3 sum = vec3(0.0, 0.0, 0.0);
  for(float i = 0.0; i <= dir_length; i+=1.0) {
	sum += texture2D(texture, uv).xyz;
	uv += step;
  }
  sum /= dir_length;
  gl_FragColor = vec4(sum, 1);
}