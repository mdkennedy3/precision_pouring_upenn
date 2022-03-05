#pragma once

#include <iostream>
#include <string>
#include <vector>
#include <Eigen/Dense> //MAP is in this (dense includes everything except sparse matricies)

class GPClass
{
  public: 
   // GPClass();
   template <class T> T InputTrainingData(const T &input_data,const T &output_data);

   template <class T, class U> T  TestData(T &test_input, U &variance); 

  private:
    template <class T> T kernel_operator_gaussian(T x1, T x2);
    double kernel_l_val = 1.0; //for k(x1,x2) = exp(dx^2 / l^2)
    template <typename T> T input_data;
    template <typename T> T output_data;
    //Variance functions
    template <typename T> T K_train_train;
    template <typename T> T K_test_train;
    template <typename T> T K_test_test;
   
};



template <class T, class T> T GPClass::InputTrainingData(T input_data, T output_data)
{
//store training data
}



template <class T, class T> T GPClass::kernel_operator_gaussian(T x1, T x2){
// Kernel operator
}


