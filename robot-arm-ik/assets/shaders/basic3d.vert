// basic3d.vert
#version 330 core
layout (location = 0) in vec3 aPos;
layout (location = 1) in vec3 aNormal; 

uniform mat4 uModel;     
uniform mat4 uView;
uniform mat4 uProjection;

out float vViewDepth;
out vec3 vNormal;        
out vec3 vWorldNormal;

void main(){
    vec4 worldPos = uModel * vec4(aPos, 1.0);
    vec4 viewPos = uView * worldPos;
    vViewDepth = -viewPos.z;
    vWorldNormal = mat3(uModel) * aNormal;   
    gl_Position = uProjection * viewPos;
}