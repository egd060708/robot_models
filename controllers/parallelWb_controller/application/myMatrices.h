/*! @file	myMatrices.h
 *  @brief	基于一维向量的矩阵运算类型
 *	@author	zzr
 *  @date	2023.9.10
 *
 *  
 */
#ifndef _MYMATRICES_H_
#define _MYMATRICES_H_

#include <iostream>
#include <vector>
#include <stdexcept>

#define USING_NAMESPACE_MM using namespace mM;
#define MATRIX myMatrices<real_t>
#define MATRIX_PTR myMatrices<real_t>*

namespace mM {

    template <class T>
    class myMatrices {
    /* 友元函数 */
        template<class T>
        friend myMatrices<T> rowCombine(myMatrices<T>& A, myMatrices<T>& B);
        template<class T, class... Args>
        friend myMatrices<T> rowCombine(myMatrices<T>& A, myMatrices<T>& B, Args... rest);
        template<class T>
        friend myMatrices<T> colCombine(myMatrices<T>& A, myMatrices<T>& B);
        template<class T, class... Args>
        friend myMatrices<T> colCombine(myMatrices<T>& A, myMatrices<T>& B, Args... rest);

    private:
        uint16_t rows;
        uint16_t cols;
        T* data;

    public:
        /* 行列构造 */
        myMatrices(uint16_t rows, uint16_t cols) : rows(rows), cols(cols)
        {
            data = new T[rows * cols];
            clear(0);
        }
        /* 方阵构造 */
        myMatrices(uint16_t n) : rows(n), cols(n)
        {
            data = new T[rows * cols];
            clear(0);
        }

        /* 矩阵清理（赋值统一） */
        void clear(T num)
        {
            if (rows > 0 && cols > 0)
            {
                for (uint16_t i = 0; i < rows; i++)
                {
                    for (uint16_t j = 0; j < cols; j++)
                    {
                        data[i * cols + j] = num;
                    }
                }
            }
        }

        /* 单位矩阵 */
        void eye()
        {
            if ((rows > 0) && (cols > 0) && (rows == cols))
            {
                for (uint16_t i = 0; i < rows; i++)
                {
                    data[i * cols + i] = 1;
                }
            }
        }
        
        /* 获取矩阵的行数 */
        uint16_t getRows() const {
            return rows;
        }

        /* 获取矩阵的列数 */
        uint16_t getCols() const {
            return cols;
        }
        
        /* 获取矩阵中特定位置的元素 */
        T getElement(uint16_t row, uint16_t col) const {
            return data[row * cols + col];
        }
        
        /* 设置矩阵中特定位置的元素 */
        void setElement(uint16_t row, uint16_t col, T value) {
            data[row * cols + col] = value;
        }

        /* 设置整个矩阵向量 */
        void setArray(const T* array, uint16_t length)
        {
            if (rows == length / cols)
            {
                memcpy(data, array, rows * cols * sizeof(T));
            }
            else
            {
            }
        }
        
        /* 设置行向量 */
        void setRowArray(const T* _array, uint16_t _row)
        {
            if (_row < rows)
            {
                for (int i = 0; i < cols; i++)
                {
                    data[cols * _row + i] = _array[i];//取出对应行数上的每一个元素
                }
            }
        }
        
        /* 设置列向量 */
        void setColArray(const T* _array, uint16_t _col)
        {
            if (_col < cols)
            {
                for (int i = 0; i < rows; i++)
                {
                    data[i * cols + _col] = _array[i];//取出对应列数上的每一个元素
                }
            }
        }
        
        /* 返回默认数据类型的数据向量 */
        const T* getArray() {
            return data;
        }
        
        /* 返回行向量 */
        void getRowArray(T* _dst, uint16_t _row)
        {
            if (_row < rows)
            {
                for (int i = 0; i < cols; i++)
                {
                    _dst[i] = data[cols * _row + i];//取出对应行数上的每一个元素
                }
            }
        }
        
        /* 返回列向量 */
        void getColArray(T* _dst, uint16_t _col)
        {
            if (_col < cols)
            {
                for (int i = 0; i < rows; i++)
                {
                    _dst[i] = data[i * cols + _col];//取出对应列数上的每一个元素
                }
            }
        }
        
        /* 打印矩阵 */
        void print() const {
            for (uint16_t i = 0; i < rows; i++) {
                for (uint16_t j = 0; j < cols; j++) {
                    std::cout << getElement(i, j) << " ";
                }
                std::cout << std::endl;
            }
            std::cout << std::endl;
        }

        //矩阵加法
        myMatrices<T> operator+(const myMatrices<T>& other) const;
        // 矩阵减法
        myMatrices<T> operator-(const myMatrices<T>& other) const;
        // 矩阵乘法
        myMatrices<T> operator*(const myMatrices<T>& other) const;
        myMatrices<T> operator*(const T other) const;
        // 矩阵求行列式
        T determinant() const;
        // 矩阵求代数余子式
        myMatrices<T> getCofactorMatrix(uint16_t rowToRemove, uint16_t colToRemove) const;
        // 矩阵转置
        myMatrices<T> transpose() const;
        // 矩阵求逆
        myMatrices<T> inverse() const;
        // 矩阵行增广
        void rowExpansion(myMatrices<T> other);
        // 矩阵列增广
        void colExpansion(myMatrices<T> other);
    };



    /* 矩阵加法 */
    template<class T>
    myMatrices<T> myMatrices<T>::operator+(const myMatrices<T>& other) const {

        myMatrices<T> result(rows, cols);

        if (rows != other.rows || cols != other.cols) {
            throw std::invalid_argument("Matrix dimensions are not compatible for addition");
        }
        else
        {
            for (uint16_t i = 0; i < rows; i++) {
                for (uint16_t j = 0; j < cols; j++) {
                    result.setElement(i, j, getElement(i, j) + other.getElement(i, j));
                }
            }
        }

        return result;
    }

    /* 矩阵减法 */
    template<class T>
    myMatrices<T> myMatrices<T>::operator-(const myMatrices<T>& other) const {

        myMatrices<T> result(rows, cols);

        if (rows != other.rows || cols != other.cols) {
            throw std::invalid_argument("Matrix dimensions are not compatible for subtraction");
            return result;
        }
        else
        {
            for (uint16_t i = 0; i < rows; i++) {
                for (uint16_t j = 0; j < cols; j++) {
                    result.setElement(i, j, getElement(i, j) - other.getElement(i, j));
                }
            }
        }

        return result;
    }

    /* 矩阵乘法 */
    template<class T>
    myMatrices<T> myMatrices<T>::operator*(const myMatrices<T>& other) const {

        myMatrices<T> result(rows, other.cols);

        if (cols != other.rows) {
            throw std::invalid_argument("Matrix dimensions are not compatible for multiplication");
        }
        else
        {
            for (uint16_t i = 0; i < rows; i++) {
                for (uint16_t j = 0; j < other.cols; j++) {
                    T sum = 0;
                    for (uint16_t k = 0; k < cols; k++) {
                        sum += getElement(i, k) * other.getElement(k, j);
                    }
                    result.setElement(i, j, sum);
                }
            }
        }

        return result;
    }

    /* 矩阵与常数相乘 */
    template<class T>
    myMatrices<T> myMatrices<T>::operator*(const T other) const {
        myMatrices<T> result(rows, cols);
        for (uint16_t i = 0; i < rows; i++)
        {
            for (uint16_t j = 0; j < cols; j++)
            {
                result.setElement(i, j, this->getElement(i, j) * other);
            }
        }
        return result;
    }
 
    /* 矩阵转置 */
    template<class T>
    myMatrices<T> myMatrices<T>::transpose() const {
        myMatrices<T> result(cols, rows);

        for (uint16_t i = 0; i < rows; i++) {
            for (uint16_t j = 0; j < cols; j++) {
                result.setElement(j, i, getElement(i, j));
            }
        }

        return result;
    }

    /* 矩阵求逆 */
    template<class T>
    myMatrices<T> myMatrices<T>::inverse() const {

        myMatrices<T> adj(rows, cols);

        if (rows != cols) {
            throw std::invalid_argument("Matrix is not square");
            return adj;
        }

        T det = determinant();
        if (det == 0) {
            throw std::runtime_error("Matrix is singular; cannot compute inverse");
            return adj;
        }

        for (uint16_t i = 0; i < rows; i++) {
            for (uint16_t j = 0; j < cols; j++) {
                // 计算(i, j)位置的代数余子式
                myMatrices<T> cofactorMatrix = getCofactorMatrix(i, j);
                double cofactor = cofactorMatrix.determinant();

                // 使用伴随矩阵的(i, j)位置存储代数余子式
                adj.setElement(i, j, cofactor);
            }
        }

        return adj * (1. / det);
    }

    /* 计算矩阵的行列式 */
    template<class T>
    T myMatrices<T>::determinant() const {
        if (rows != cols) {
            throw std::invalid_argument("Matrix must be square to calculate determinant");
            return 0;
        }

        if (rows == 1) {
            return getElement(0, 0);
        }

        if (rows == 2) {
            // 对于2x2矩阵，行列式计算公式为 ad - bc
            return getElement(0, 0) * getElement(1, 1) - getElement(0, 1) * getElement(1, 0);
        }

        float det = 0;
        for (uint16_t i = 0; i < cols; i++) {
            // 计算代数余子式的值
            float cofactor = getElement(0, i) * getCofactorMatrix(0, i).determinant();
            // 使用递归计算行列式
            det += (i % 2 == 0 ? 1 : -1) * cofactor;
        }

        return det;
    }

    /* 创建代数余子式 */
    template<class T>
    myMatrices<T> myMatrices<T>::getCofactorMatrix(uint16_t rowToRemove, uint16_t colToRemove) const {

        myMatrices<T> cofactor(rows - 1, cols - 1);

        if (rowToRemove < 0 || rowToRemove >= rows || colToRemove < 0 || colToRemove >= cols) {
            throw std::invalid_argument("Invalid row or column index");
            return cofactor;
        }

        uint16_t cofactorRow = 0;
        for (uint16_t i = 0; i < rows; i++) {
            if (i == rowToRemove) {
                continue; // 跳过要移除的行
            }

            uint16_t cofactorCol = 0;
            for (uint16_t j = 0; j < cols; j++) {
                if (j == colToRemove) {
                    continue; // 跳过要移除的列
                }

                cofactor.setElement(cofactorRow, cofactorCol, getElement(i, j));
                cofactorCol++;
            }

            cofactorRow++;
        }

        return cofactor;
    }

    /* 矩阵行增广 */
    template<class T>
    void myMatrices<T>::rowExpansion(myMatrices<T> other)
    {
        if (cols != other.cols){
            throw  std::invalid_argument("Matrix dimensions are not compatible for expansion");
        }
        else
        {
            T* p = data;//读取原有内存地址
            data = new T[(rows + other.rows) * cols];//重新分配内存
            memcpy(data, p, sizeof(T)*rows*cols);//将原数据copy
            memcpy(data+(rows * cols), other.getArray(), sizeof(T) * other.rows * other.cols);//copy增广数据
            rows = rows + other.rows;//更改行数
            delete p;//删除原数据内存
        }
    }

    /* 矩阵列增广 */
    template<class T>
    void myMatrices<T>::colExpansion(myMatrices<T> other)
    {
        if (rows != other.rows) {
            throw  std::invalid_argument("Matrix dimensions are not compatible for expansion");
        }
        else
        {
            T* p = data;//读取原有内存地址
            data = new T[rows * (cols + other.cols)];//重新分配内存
            uint16_t n = cols + other.cols;
            for (uint16_t i = 0; i < rows; i++)
            {
                memcpy(data + i*n, p + i*cols, sizeof(T) * cols);//将原数据copy
                memcpy(data + i * n + cols, other.getArray() + i * other.cols, sizeof(T) * other.cols);//copy增广数据
            }
            cols = cols + other.cols;//更改列数
            delete p;//删除原数据内存  
        }
    }

    /* 计算 Kronecker 乘积（参考matlab的kron函数） */
    template <class T>
    myMatrices<T> kron(myMatrices<T>& A, myMatrices<T>& B) {
        uint16_t m = A.getRows();
        uint16_t n = A.getCols();
        uint16_t p = B.getRows();
        uint16_t q = B.getCols();

        myMatrices<T> result(m * p, n * q);

        for (uint16_t i = 0; i < m; i++) {
            for (uint16_t j = 0; j < n; j++) {
                for (uint16_t k = 0; k < p; k++) {
                    for (uint16_t l = 0; l < q; l++) {
                        result.setElement(i * p + k, j * q + l, A.getElement(i, j) * B.getElement(k, l));
                    }
                }
            }
        }

        return result;
    }

    template <class T, class... Args>
    myMatrices<T> kron(myMatrices<T>& A, myMatrices<T>& B, Args&... rest) {
        uint16_t m = A.getRows();
        uint16_t n = A.getCols();
        uint16_t p = B.getRows();
        uint16_t q = B.getCols();

        myMatrices<T> result(m * p, n * q);

        for (uint16_t i = 0; i < m; i++) {
            for (uint16_t j = 0; j < n; j++) {
                for (uint16_t k = 0; k < p; k++) {
                    for (uint16_t l = 0; l < q; l++) {
                        result.setElement(i * p + k, j * q + l, A.getElement(i, j) * B.getElement(k, l));
                    }
                }
            }
        }

        return kron(result, rest...);
    }

    /* 构建对角块矩阵 */
    template <class T>
    myMatrices<T> blkdiag(const myMatrices<T>& A, const myMatrices<T>& B) {

        myMatrices<T> result(A.getRows() + B.getRows(), A.getCols() + B.getCols());

        uint16_t rowOffset = 0;
        uint16_t colOffset = 0;

        for (uint16_t j = 0; j < A.getRows(); j++) {
            for (uint16_t k = 0; k < A.getCols(); k++) {
                result.setElement(rowOffset + j, colOffset + k, A.getElement(j, k));
            }
        }

        rowOffset += A.getRows();
        colOffset += A.getCols();

        for (uint16_t j = 0; j < B.getRows(); j++) {
            for (uint16_t k = 0; k < B.getCols(); k++) {
                result.setElement(rowOffset + j, colOffset + k, B.getElement(j, k));
            }
        }

        rowOffset += B.getRows();
        colOffset += B.getCols();

        return result;
    }

    template <class T, class... Args>
    myMatrices<T> blkdiag(const myMatrices<T>& A, const myMatrices<T>& B, const Args&... rest) {

        myMatrices<T> result(A.getRows() + B.getRows(), A.getCols() + B.getCols());

        uint16_t rowOffset = 0;
        uint16_t colOffset = 0;

        for (uint16_t j = 0; j < A.getRows(); j++) {
            for (uint16_t k = 0; k < A.getCols(); k++) {
                result.setElement(rowOffset + j, colOffset + k, A.getElement(j, k));
            }
        }

        rowOffset += A.getRows();
        colOffset += A.getCols();

        for (uint16_t j = 0; j < B.getRows(); ++j) {
            for (uint16_t k = 0; k < B.getCols(); ++k) {
                result.setElement(rowOffset + j, colOffset + k, B.getElement(j, k));
            }
        }

        rowOffset += B.getRows();
        colOffset += B.getCols();

        return blkdiag(result, rest...);
    }

    /* 创建零矩阵 */
    template<class T>
    myMatrices<T> zeros(uint16_t n)
    {
        myMatrices<T> result(n, n);
        return result;
    }
    template<class T>
    myMatrices<T> zeros(uint16_t rows, uint16_t cols)
    {
        myMatrices<T> result(rows, cols);
        return result;
    }

    /* 创建单位对角阵 */
    template<class T>
    myMatrices<T> eye(uint16_t n)
    {
        myMatrices<T> result(n, n);
        for (uint16_t i = 0; i < n; i++)
        {
            result.setElement(i, i, 1);
        }
        return result;
    }

    /* 矩阵行向合并（行不变列变） */
    template<class T>
    myMatrices<T> rowCombine(myMatrices<T>& A,myMatrices<T>& B)
    {
        myMatrices<T> result(A.getRows() + B.getRows(), A.getCols());
        if (A.getCols() != B.getCols()) {
            throw  std::invalid_argument("Matrix dimensions are not compatible for combine");
            return result;
        }
        else
        {
            memcpy(result.data, A.getArray(), sizeof(T) * A.getRows() * A.getCols());//将原数据copy
            memcpy(result.data + (A.getRows() * A.getCols()), B.getArray(), sizeof(T) * B.getRows() * B.getCols());//copy增广数据
            return result;
        }
    }

    template <class T, class... Args>
    myMatrices<T> rowCombine(myMatrices<T>& A, myMatrices<T>& B,Args... rest)
    {
        myMatrices<T> result(A.getRows() + B.getRows(), A.getCols());
        if (A.getCols() != B.getCols()) {
            throw  std::invalid_argument("Matrix dimensions are not compatible for combine");
            return result;
        }
        else
        {
            memcpy(result.data, A.getArray(), sizeof(T) * A.getRows() * A.getCols());//将原数据copy
            memcpy(result.data + (A.getRows() * A.getCols()), B.getArray(), sizeof(T) * B.getRows() * B.getCols());//copy增广数据
            return rowCombine(result,rest...);
        }
    }

    /* 矩阵列向合并（行变列不变） */
    template<class T>
    myMatrices<T> colCombine(myMatrices<T>& A, myMatrices<T>& B)
    {
        myMatrices<T> result(A.getRows(), A.getCols() + B.getCols());
        if (A.getRows() != B.getRows()) {
            throw  std::invalid_argument("Matrix dimensions are not compatible for combine");
            return result;
        }
        else
        {
            uint16_t n = A.getCols() + B.getCols();
            for (uint16_t i = 0; i < A.getRows(); i++)
            {
                memcpy(result.data + i * n, A.getArray() + i * A.getCols(), sizeof(T) * A.getCols());//将原数据copy
                memcpy(result.data + i * n + A.getCols(), B.getArray() + i * B.getCols(), sizeof(T) * B.getCols());//copy增广数据
            }
            return result;
        }
    }

    template<class T, class... Args>
    myMatrices<T> colCombine(myMatrices<T>& A, myMatrices<T>& B, Args... rest)
    {
        myMatrices<T> result(A.getRows(), A.getCols() + B.getCols());
        if (A.getRows() != B.getRows()) {
            throw  std::invalid_argument("Matrix dimensions are not compatible for combine");
            return result;
        }
        else
        {
            uint16_t n = A.getCols() + B.getCols();
            for (uint16_t i = 0; i < A.getRows(); i++)
            {
                memcpy(result.data + i * n, A.getArray() + i * A.getCols(), sizeof(T) * A.getCols());//将原数据copy
                memcpy(result.data + i * n + A.getCols(), B.getArray() + i * B.getCols(), sizeof(T) * B.getCols());//copy增广数据
            }
            return colCombine(result,rest...);
        }
    }

}

#endif
