#ifndef _ALGOUTFILE_H_
#define _ALGOUTFILE_H_

#include<stdio.h>
#include<math.h>
#include<stdlib.h>
#include<string.h>
#include<iostream>
#include <iomanip>
#include <string>
#include <fstream>

using namespace std;

void Exel_Output(string name, std::ios_base::openmode my_mode, float a[], int j, string discription); //向文件写入数组数据
void file_create(string name, std::ios_base::openmode my_mode);//建立文件

#endif