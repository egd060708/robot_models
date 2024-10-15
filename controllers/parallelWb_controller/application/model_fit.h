/*! @file	model_fit.h
 *  @brief	ģ�;�����������
 *	@author	zzr
 *  @date	2023.9.12
 *
 *  1��setFunctions
 *	2��modelGenerate
 */
#ifndef _MODEL_FIT_H_
#define _MODEL_FIT_H_

#include <qpOASES.hpp>
#include "myMatrices.h"

USING_NAMESPACE_MM
USING_NAMESPACE_QPOASES

/* ģ�����������ģ�͵�����������ģ�͵���������Ϻ����Ľ��� */
template<uint_t rows,uint_t cols,uint_t order>
class modelFit {
private:
	MATRIX model = MATRIX(rows * cols, order + 1);//�������ֽ״β������������־���λ�ò���

	/* �ݹ������ⷽ�̽��: �Ա���������ϵ�������̽��� */
	real_t functionSolve(const real_t _x, const real_t _para[order + 1], uint_t _orderNum)
	{
		if (_orderNum)
		{
			return _para[_orderNum] * powf(_x, _orderNum) + functionSolve(_x, _para, _orderNum - 1);
		}
		else
		{
			return _para[_orderNum];
		}
	}
public:
	/* ģ�����ɺ���	�βΣ��Ա��� */
	MATRIX modelGenerate(real_t _x) {
		MATRIX result(rows, cols);
		for(int i=0;i < rows;i++)
			for (int j = 0; j < cols; j++) {
				real_t tmp[order + 1];
				model.getRowArray(tmp, i * cols + j);
				result.setElement(i, j, functionSolve(_x,tmp, order));
			}
		return result;
	}
	/* ������Ϸ��̲������Զ����շ��̸�������ѭ��д�� */
	void setFunctions(const real_t _functions[rows*cols*(order + 1)])
	{
		model.setArray(_functions, rows * cols * (order + 1));
	}

};

#endif
