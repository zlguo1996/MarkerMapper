//
//  Tools.hpp
//  ARTest
//
//  Created by 郭子乐 on 2017/12/28.
//  Copyright © 2017年 郭子乐. All rights reserved.
//

#ifndef Tools_hpp
#define Tools_hpp


#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

using namespace std;
using namespace cv;

void getViewMatrixFromRvecTvec(float* rvec, float* tvec, Mat& viewMatrix);
void getViewMatrixFromRvecTvec(InputArray& rvec, InputArray& tvec, Mat& viewMatrix);
void getRvecTvecFromViewMatrix(InputArray& viewMatrix, Mat& rvec, Mat& tvec);

//cv - gl : invert axis y,z
void cvToGl(const Mat& cv, Mat& gl);
void glToCv(const Mat& gl, Mat& cv);

// cv::mat - glm::mat
void cvToGlm(const Mat& cv, glm::mat4& glm);
void glmToCv(const glm::mat4& glm, Mat& cv);

// 通过element/index构建新数组
template <typename VT, typename ET>
VT* orderByElement(const VT* const vertexArray, unsigned int vaLength,
                   const ET* const elementArray, unsigned int eaLength,
                   unsigned int size,   // num of components
                   unsigned int stride, // num of components in one block
                   unsigned offset)     // offset of first components in one block
{
    assert((size+offset)<stride);
    
    VT newOrder[eaLength*size];
    
    for (int i=0; i<eaLength; i++) {
        for (int j=0; j<size; j++) {
            newOrder[i*size+j] = vertexArray[elementArray[i]*stride+offset+j];
        }
    }
    
    return newOrder;
}

// vector和数组的相互转化
template <typename T>
vector<Point3_<T>> arrayToVectorN3(const T* const vertexArray, unsigned int vaLength){
    assert(vaLength%3==0);
    
    vector<Point3_<T>> vec;
    for (int i=0; i<vaLength/3; i++) {
        int index = i*3;
        Point3_<T> point(vertexArray[index], vertexArray[index+1], vertexArray[index+2]);
        vec.push_back(point);
    }
    
    return vec;
}

template <typename T>
vector<vector<Point3_<T>>> arrayToVectorNM3(const T* const vertexArray, unsigned int vaLength, int M){
    assert(vaLength%(3*M)==0);
    
    vector<vector<Point3_<T>>> vec;
    for (int i=0; i<vaLength/(3*M); i++) {
        vector<Point3_<T>> vec2;
        for (int j=0; j<M; j++) {
            int index = i*M*3+j*3;
            Point3_<T> point(vertexArray[index], vertexArray[index+1], vertexArray[index+2]);
            vec2.push_back(point);
        }
        vec.push_back(vec2);
    }
    
    return vec;
}

// vector类型转化
template <typename SrcT, typename DstT>
void convertVectorType(const vector<SrcT>& src, vector<DstT>& dst){
    assert(dst.empty());
    for(auto i:src){
        dst.push_back(DstT(i));
    }
}

// 打印数值
template <typename T>
void printArray(T* pointer, int num){
    for (int i=0; i<num-1; i++){
        cout << pointer[i] << ",";
    }
    cout << pointer[num-1] << endl;
}
template <typename T>
void printVector(string str, const vector<T>& vec){
    cout << str << ": ";
    for(auto i : vec){
        cout << i << " ";
    }
    cout << endl;
}

#endif /* Tools_hpp */
