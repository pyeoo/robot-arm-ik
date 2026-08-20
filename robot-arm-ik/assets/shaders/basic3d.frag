// basic3d.frag
#version 330 core

in float vViewDepth;
in vec3 vWorldNormal;
out vec4 fragColor;

uniform vec3 uColor;
uniform vec3 uFogColor;
uniform float uFogNear;
uniform float uFogFar;
uniform vec3 uLightDir;

void main(){
    vec3 n = normalize(vWorldNormal);
    float diffuse = max(dot(n, normalize(uLightDir)), 0.15);  // 0.15 = ambient floor so unlit faces aren't pure black

    float t = clamp((vViewDepth - uFogNear) / (uFogFar - uFogNear), 0.0, 1.0);
    vec3 litColor = uColor * diffuse;
    vec3 finalColor = mix(litColor, uFogColor, t);
    fragColor = vec4(finalColor, 1.0);
}
