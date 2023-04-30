uniform sampler2D texture;

varying vec4 vertColor;
varying vec4 vertTexCoord;

uniform vec3 minHSV;
uniform vec3 maxHSV;

vec3 rgb2hsv(vec3 c)
{
    vec4 K = vec4(0.0, -1.0 / 3.0, 2.0 / 3.0, -1.0);
    vec4 p = mix(vec4(c.bg, K.wz), vec4(c.gb, K.xy), step(c.b, c.g));
    vec4 q = mix(vec4(p.xyw, c.r), vec4(c.r, p.yzx), step(p.x, c.r));

    float d = q.x - min(q.w, q.y);
    float e = 1.0e-10;
    return vec3(abs(q.z + (q.w - q.y) / (6.0 * d + e)), d / (q.x + e), q.x);
}

void main() {
  vec3 texCol = rgb2hsv(vec3(texture2D(texture, vertTexCoord.xy)));
  
  vec3 mask = step(minHSV, texCol) * step(texCol, maxHSV); // if minColor < texCol and textCol < maxCol
  if(minHSV.x > maxHSV.x) {
	mask.x = step(0.5, step(maxHSV.x, texCol.x) + step(texCol.x, minHSV.x));
  }
  float val = mask.x*mask.y*mask.z; // if every component satisfies the above condition ^^^^
  gl_FragColor = vec4(val, val, val, 1);
}