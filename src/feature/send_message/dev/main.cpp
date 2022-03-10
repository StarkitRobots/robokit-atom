#include "matrixx.h"
#include <iostream>
int main (){
    std :: vector <double> a =  {1,3,4,1};
    std :: vector <std :: vector <double>>mat {{-3,2,-1,3},{100,-100,100,100},{4,-5,1,-3}};
     Matrix<3,4,double> matrix;
    
    for (int i = 0; i < 3; ++ i){
         for (int j = 0; j < 4; ++j){
            matrix[i][j] = mat[i][j];
         }
         
     }
          matrix.gaussMethod();
    //std::cout << matrix.det();
     for (int i = 0; i < 3; ++ i){
         for (int j = 0; j < 4; ++j){
            std :: cout << matrix[i][j] <<" ";
         }
         std :: cout << std :: endl;
     }
    return 0;
}