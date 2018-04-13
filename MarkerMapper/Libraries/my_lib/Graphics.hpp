//
//  Graphics.hpp
//  ARTest
//
//  Created by 郭子乐 on 2018/1/1.
//  Copyright © 2018年 郭子乐. All rights reserved.
//

#ifndef Graphics_hpp
#define Graphics_hpp

#include <iostream>
#include <cmath>
#include <assert.h>
#include <vector>

#include "Shader.h"

#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

using namespace std;

namespace Graphics{
    
    class Grid{
    public:
        int size;
        float width;
        glm::vec3 color;
        
        Grid(int size=10, float width=1.0f, glm::vec3 color=glm::vec3(1.0f, 1.0f, 1.0f), string vshader="Shaders/shader1.vs", string fshader="Shaders/shader1.frag") : shader(vshader.c_str(), fshader.c_str()){
            assert(size<=10);
            
            this->size = size;
            this->width = width;
            this->color = color;
            
            float step = width/size;
            vertexNum = size*4+4;
            
            for (int i=0; i<size+1; i++) {
                vertices[i*12] = 0.0f;
                vertices[i*12+1] = 0.0f;
                vertices[i*12+2] = step*i;
                vertices[i*12+3] = color[0];
                vertices[i*12+4] = color[1];
                vertices[i*12+5] = color[2];
                vertices[i*12+6] = width;
                vertices[i*12+7] = 0.0f;
                vertices[i*12+8] = step*i;
                vertices[i*12+9] = color[0];
                vertices[i*12+10] = color[1];
                vertices[i*12+11] = color[2];
            }
            for (int i=0; i<size+1; i++) {
                vertices[size*12+12+i*12] = step*i;
                vertices[size*12+12+i*12+1] = 0.0f;
                vertices[size*12+12+i*12+2] = 0.0f;
                vertices[size*12+12+i*12+3] = color[0];
                vertices[size*12+12+i*12+4] = color[1];
                vertices[size*12+12+i*12+5] = color[2];
                vertices[size*12+12+i*12+6] = step*i;
                vertices[size*12+12+i*12+7] = 0.0f;
                vertices[size*12+12+i*12+8] = width;
                vertices[size*12+12+i*12+9] = color[0];
                vertices[size*12+12+i*12+10] = color[1];
                vertices[size*12+12+i*12+11] = color[2];
            }
            
            glGenVertexArrays(1, &VAO);
            glGenBuffers(1, &VBO);
            glBindVertexArray(VAO);
            glBindBuffer(GL_ARRAY_BUFFER, VBO);
            glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);
            glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6*sizeof(GLfloat), (GLvoid*)0);
            glEnableVertexAttribArray(0);
            glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6*sizeof(GLfloat), (GLvoid*)(3*sizeof(GLfloat)));
            glEnableVertexAttribArray(1);
        }
        
        ~Grid(){
            glDeleteVertexArrays(1, &VAO);
            glDeleteBuffers(1, &VBO);
        }
        
        void draw(glm::mat4 viewMat, glm::mat4 projectionMat){
            shader.use();
            
            glm::mat4 modelMat, MVP;
            
            modelMat = glm::rotate(glm::mat4(1.0), glm::radians(0.0f), glm::vec3(0.0f, 1.0f, 0.0f));
            MVP = projectionMat*viewMat*modelMat;
            glUniformMatrix4fv(glGetUniformLocation(shader.ID, "MVP"), 1, GL_FALSE, glm::value_ptr(MVP));
            
            glBindVertexArray(VAO);
            glDrawArrays(GL_LINES, 0, vertexNum);
            
            modelMat = glm::rotate(glm::mat4(1.0), glm::radians(90.0f), glm::vec3(0.0f, 0.0f, 1.0f));
            MVP = projectionMat*viewMat*modelMat;
            glUniformMatrix4fv(glGetUniformLocation(shader.ID, "MVP"), 1, GL_FALSE, glm::value_ptr(MVP));
            
            glBindVertexArray(VAO);
            glDrawArrays(GL_LINES, 0, vertexNum);
            
            modelMat = glm::rotate(glm::mat4(1.0), glm::radians(-90.0f), glm::vec3(1.0f, 0.0f, 0.0f));
            MVP = projectionMat*viewMat*modelMat;
            glUniformMatrix4fv(glGetUniformLocation(shader.ID, "MVP"), 1, GL_FALSE, glm::value_ptr(MVP));
            
            glBindVertexArray(VAO);
            glDrawArrays(GL_LINES, 0, vertexNum);
        }
        
    private:
        GLfloat vertices[22*12];
        int vertexNum;
        
        GLuint VAO, VBO;
        Shader shader;
    };
    
    class Axis{
    public:
        float width;
        
        Axis(float axisWidth=1.3f, string vshader="Shaders/shader1.vs", string fshader="Shaders/shader1.frag") : width(axisWidth), shader(vshader.c_str(), fshader.c_str()){
            GLfloat tVertices[36] = {
                0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f,
                axisWidth, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f,
                0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f,
                0.0f, axisWidth, 0.0f, 0.0f, 1.0f, 0.0f,
                0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f,
                0.0f, 0.0f, axisWidth, 0.0f, 0.0f, 1.0f
            };
            for (int i=0; i<36; i++) {
                vertices[i] = tVertices[i];
            }
            
            glGenVertexArrays(1, &VAO);
            glGenBuffers(1, &VBO);
            glBindVertexArray(VAO);
            glBindBuffer(GL_ARRAY_BUFFER, VBO);
            glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);
            glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6*sizeof(GLfloat), (GLvoid*)0);
            glEnableVertexAttribArray(0);
            glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6*sizeof(GLfloat), (GLvoid*)(3*sizeof(GLfloat)));
            glEnableVertexAttribArray(1);
        }
        
        ~Axis(){
            glDeleteVertexArrays(1, &VAO);
            glDeleteBuffers(1, &VBO);
        }
        
        void draw(glm::mat4 viewMat, glm::mat4 projectionMat){
            shader.use();
            
            glm::mat4 modelMat, MVP;
            
            modelMat = glm::mat4(1.0);
            MVP = projectionMat*viewMat*modelMat;
            glUniformMatrix4fv(glGetUniformLocation(shader.ID, "MVP"), 1, GL_FALSE, glm::value_ptr(MVP));
            
            glBindVertexArray(VAO);
            glDrawArrays(GL_LINES, 0, 6);
        }
        
    private:
        GLfloat vertices[6*3*2];
        GLuint VAO, VBO;
        Shader shader;
    };
    
    class Points{
    public:
        int MaxVertexNum;
        glm::vec3 LineColor;
        
        Points(glm::vec3 lineColor=glm::vec3(1.0f, 1.0f, 0.0f), int maxVertexNum=60*60*10) : shader("Shaders/shader2.vs", "Shaders/shader2.frag"), MaxVertexNum(maxVertexNum), LineColor(lineColor){
            glGenVertexArrays(1, &VAO);
            glGenBuffers(1, &VBO);
            glBindVertexArray(VAO);
            glBindBuffer(GL_ARRAY_BUFFER, VBO);
            glBufferData(GL_ARRAY_BUFFER, 3*maxVertexNum*sizeof(GLfloat), NULL, GL_DYNAMIC_DRAW);
            glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3*sizeof(GLfloat), (GLvoid*)0);
            glEnableVertexAttribArray(0);
            
            shader.use();
            shader.setVec3("ourColor", LineColor);
        }
        ~Points(){
            glDeleteBuffers(1, &VBO);
            glDeleteVertexArrays(1, &VAO);
        }
        void addPoint(glm::vec3 pos){
            assert(vertices.size()<=MaxVertexNum);
            
            vertices.push_back(pos);
            
            GLfloat vertex[3] = {
                pos.x, pos.y, pos.z
            };
            
            glBindBuffer(GL_ARRAY_BUFFER, VBO);
            glBufferSubData(GL_ARRAY_BUFFER, 3*vertices.size()*sizeof(GLfloat), sizeof(vertex), vertex);
        }
        void draw(glm::mat4 viewMat, glm::mat4 projectionMat){
            if (vertices.empty()) return;
            
            shader.use();
            
            glm::mat4 modelMat, MVP;
            
            modelMat = glm::mat4(1.0);
            MVP = projectionMat*viewMat*modelMat;
            //glUniformMatrix4fv(glGetUniformLocation(shader.ID, "MVP"), 1, GL_FALSE, glm::value_ptr(MVP));
            shader.setMat4("MVP", MVP);
            
            glBindVertexArray(VAO);
            glDrawArrays(GL_LINE_STRIP, 0, vertices.size());
        }
        int getVertexNum(){
            return vertices.size();
        }
        void clear(){
            vertices.clear();
        }
        
    private:
        std::vector<glm::vec3> vertices;
        
        GLuint VAO, VBO;
        Shader shader;
    };

}
#endif /* Graphics_hpp */
