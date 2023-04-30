#ifdef GL_ES
precision mediump float;
precision mediump int;
#endif

uniform sampler2D texture;

varying vec4 vertColor;
varying vec4 vertTexCoord;

const float Pi = 6.28318530718; // Pi*2

uniform float Directions; // 16.0 is good
uniform float Quality; // 3.0 is good
uniform float Size; // 8.0 is good
uniform vec3 Resolution;

void main() {
  //pixel color
  vec4 Color = vec4(0);//texture2D(texture, vertTexCoord.xy);
  
  vec2 Radius = vec2(Size)/Resolution.xy;
  // Blur calculations
  for( float d=0.0; d<Pi; d+=Pi/Directions) {
	for(float i=1.0/Quality; i<=1.0; i+=1.0/Quality) {
	  vec4 toAdd = texture2D(texture, vertTexCoord.xy + (vec2(cos(d), sin(d)) * Radius * i));
	  // weigh average by gaussian function (constants result in an integral of ~1)
	  toAdd *= (1/Directions) * exp(-(i * i) * 3.05798304434);
	  Color += toAdd;
    }
  }
  
  gl_FragColor = Color;
}