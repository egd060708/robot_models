#pragma once

#include<iostream>
#include <iomanip>
#include <string>
#include <fstream>

using namespace std;

void exel_output(string name, std::ios_base::openmode my_mode, float a[], int j, string discription); //���ļ�д����������
void file_create(string name, std::ios_base::openmode my_mode);//�����ļ�
