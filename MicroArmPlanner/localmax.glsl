uniform sampler2D texture;

varying vec4 vertColor;
varying vec4 vertTexCoord;

uniform vec3 texSize;
uniform float radius;

const float small = 0.000001;

void main() {
  vec2 s = vec2(radius, radius)/texSize.xy;
  // assuming texture is just black and white, so only using red channel
  float center = texture2D(texture, vertTexCoord.xy).x;
  vec4 adj = vec4(
    texture2D(texture, vertTexCoord.xy + vec2(-s.x, 0)).x, // left
	texture2D(texture, vertTexCoord.xy + vec2(s.x, 0)).x, // right
	texture2D(texture, vertTexCoord.xy + vec2(0, -s.y)).x, // up
	texture2D(texture, vertTexCoord.xy + vec2(0, s.y)).x // down
  );
  vec4 delta = vec4(center, center, center, center) - adj;
  // if this pixel is a local maximum, then all differences between the center - adjacent pixels will be >= 0
  vec4 isgt0 = step(vec4(small, small, small, small), delta); // for every component, if that component is positive
  float val = isgt0.x*isgt0.y*isgt0.z*isgt0.w; // if every component is positive
  gl_FragColor = vec4(val, val, val, 1);
}