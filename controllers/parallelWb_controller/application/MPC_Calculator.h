/*! @file	MPC_Calculator.h
 *  @brief	基于qpOASES求解库的简单qp求解类型
 *	@author	zzr
 *  @date	2023.9.11
 *
 *	具体原理参考b站Dr.CAN的mpc控制器相关视频
 */
#ifndef _MPC_CALCULATOR_H_
#define _MPC_CALCULATOR_H_

#include <qpOASES.hpp>
#include "myMatrices.h"
#include <limits>//定义各种变量的储存最大值

USING_NAMESPACE_MM
USING_NAMESPACE_QPOASES


template<uint_t xNum, uint_t uNum, uint_t preStep = 10, uint_t ctrlStep = 5>// 问题规模(状态变量和输入变量个数，预测及控制步长）
class MPC_CalculatorClassdef {
public:
	// 离散状态空间方程
	MATRIX A = MATRIX(xNum);
	MATRIX B = MATRIX(xNum, uNum);
	// 权重矩阵，补偿矩阵
	MATRIX Q = MATRIX(xNum);
	MATRIX R = MATRIX(uNum);
	MATRIX F = MATRIX(xNum);
	// 状态向量
	MATRIX Y = MATRIX(xNum, 1);//目标向量
	MATRIX X = MATRIX(xNum, 1);//当前状态
	MATRIX U = MATRIX(uNum, 1);//输出向量
	// 中间变量
	MATRIX G = MATRIX(xNum);
	MATRIX E = MATRIX(ctrlStep * uNum, xNum);
	MATRIX L = MATRIX(uNum * ctrlStep, (ctrlStep + 1) * xNum);
	MATRIX H = MATRIX(ctrlStep * uNum);
	// 预测结果
	MATRIX Y_K = MATRIX(xNum * (ctrlStep + 1), 1);	// qp求解给定
	MATRIX X_K = MATRIX(xNum, preStep + 1);			// qp求解状态
	MATRIX U_K = MATRIX(uNum, preStep);				// qp求解输出
	MATRIX X_COMPARE = MATRIX(2 * xNum, 1);			// 对齐时间戳后的状态，预测在低位，实际在高位
	// 约束矩阵
	MATRIX lb = MATRIX(1, uNum * ctrlStep);// low
	MATRIX ub = MATRIX(1, uNum * ctrlStep);// up


	// qp求解器
	QProblemB qp_solver;
	// qp求解执行次数
	int_t nWSR_static = 20;		// 最大qp迭代次数
	int_t nWSR = 20;
	real_t CPU_t_static = 0.008;// 最长CPU使用时间
	real_t CPU_t = 0.008;
	uint8_t isModelUpdate = 1;	// 系统模型是否更新

public:
	MPC_CalculatorClassdef() :qp_solver(ctrlStep* uNum, HST_POSDEF)
	{
		Q.eye();
		lb.clear(-std::numeric_limits<real_t>::max());
		ub.clear(std::numeric_limits<real_t>::max());
		Options option;
		option.printLevel = PL_NONE;//禁用打印输出
		qp_solver.setOptions(option);
	}

	// qp求解得预测输出
	MATRIX prediction(MATRIX& y_k, MATRIX& x_k)
	{
		real_t qp_out[ctrlStep * uNum];
		MATRIX g_new(ctrlStep * uNum, 1);
		g_new = E * x_k - L * y_k;

		if (isModelUpdate == 1)
		{
			qp_solver.init(H.getArray(), g_new.getArray(), lb.getArray(), ub.getArray(), nWSR, &CPU_t);
		}
		else
		{
			qp_solver.hotstart(g_new.getArray(), lb.getArray(), ub.getArray(), nWSR, &CPU_t);
		}
		
		nWSR = nWSR_static;
		CPU_t = CPU_t_static;
		qp_solver.getPrimalSolution(qp_out);
		//std::cout << qp_out[0] << " " << qp_out[1] << " " << qp_out[2] << std::endl;

		MATRIX result(uNum, 1);
		for (int i = 0; i < uNum; i++)
		{
			result.setElement(i, 0, qp_out[i]);
		}
		return result;
	}


	

	// mpc控制器参数矩阵生成
	void mpc_matrices()
	{
		MATRIX M(xNum);
		M.eye();

		MATRIX C(xNum, ctrlStep * uNum);

		MATRIX tmp(xNum);
		tmp.eye();
		MATRIX tmps(xNum, uNum);
		// 填充C矩阵和M矩阵
		for (int i = 0; i < ctrlStep; i++)
		{
			if (i) {
				MATRIX tmp2 = tmp * B;
				tmps = colCombine(tmp2, tmps);
			}
			else {
				tmps = tmp * B;
			}
			MATRIX copy = tmps;
			if (i < ctrlStep - 1) {
				MATRIX zero_expand(xNum, uNum * (ctrlStep - i - 1));
				copy = colCombine(copy, zero_expand);
			}
			C = rowCombine(C, copy);
			tmp = A * tmp;
			M = rowCombine(M, tmp);
		}

		MATRIX tmp1(ctrlStep);
		tmp1.eye();
		MATRIX Q_bar = kron(tmp1, Q);
		Q_bar = blkdiag(Q_bar, F);
		MATRIX R_bar = kron(tmp1, R);

		// 计算G, E, F, H
		MATRIX M_T = M.transpose();
		MATRIX C_T = C.transpose();

		G = M_T * Q_bar * M; // G: n x n
		L = C_T * Q_bar; // F: NP x n
		E = L * M; // E: NP x n
		H = C_T * Q_bar * C + R_bar; // NP x NP

	}

	// mpc初始化
	void mpc_init(MATRIX _A, MATRIX _B, MATRIX _Q, MATRIX _R, MATRIX _F)
	{
		A = _A; B = _B; Q = _Q; R = _R; F = _F;
		mpc_matrices();
	}

	// 控制器状态更新
	void mpc_update(MATRIX _Y, MATRIX _X, int_t _nWSR = 10, real_t _cpu_t = 1)
	{
		Y = _Y; X = _X; nWSR_static = _nWSR; CPU_t_static = _cpu_t;

		for (int i = 0; i < xNum; i++)
		{
			X_K.setElement(i, 0, X.getElement(i, 0));//获取当前状态作为初始值
		}
		for (int i = 0; i <= ctrlStep; i++)
			for (int j = 0; j < xNum; j++) {
				Y_K.setElement(i * xNum + j, 0, Y.getElement(j, 0));//设置目标向量
			}
	}

	// 设置输入约束（上下限）
	void setConstrain(MATRIX _lb, MATRIX _ub)
	{
		for (int i = 0; i < ctrlStep; i++)
		{
			for (int j = 0; j < uNum; j++)
			{
				lb.setElement(0, i * uNum + j, _lb.getElement(0, j));
				ub.setElement(0, i * uNum + j, _ub.getElement(0, j));
			}
		}
	}

	// 控制器求解
	void mpc_solve()
	{
		//执行预测
		MATRIX tmp_xk = X;
		MATRIX tmp_uk(uNum, 1);
		for (int i = 0; i < preStep; i++)
		{
			tmp_uk = prediction(Y_K, tmp_xk);//qp求解出当前输出
			tmp_xk = A * tmp_xk + B * tmp_uk;//预测下一周期的状态
			
			for (int j = 0; j < xNum; j++)
			{
				X_K.setElement(j, i + 1, tmp_xk.getElement(j, 0));//把新状态记录下来
			}
			for (int k = 0; k < uNum; k++)
			{
				U_K.setElement(k, i, tmp_uk.getElement(k, 0));//把预测输出记录下来
			}
		}
		//std::cout << X_K.getElement(0, 0) << " " << X.getElement(0, 0) << std::endl;
		//copy输出值
		for (int i = 0; i < uNum; i++)
		{
			U.setElement(i, 0, U_K.getElement(i, 0));
		}
		//std::cout << U.getElement(0, 0) << " " << U.getElement(1, 0) << " " << U.getElement(2, 0) << std::endl;
	}

	//使用外部输入进行模型预测
	void mpc_solve(real_t ext_uk[uNum])
	{
		//执行预测
		MATRIX tmp_xk = X;
		MATRIX tmp_uk(uNum, 1);
		for (int i = 0; i < preStep; i++)
		{
			tmp_uk.setArray(ext_uk,uNum);//qp求解出当前输出
			tmp_xk = A * tmp_xk + B * tmp_uk;//预测下一周期的状态
			for (int j = 0; j < xNum; j++)
			{
				X_K.setElement(j, i + 1, tmp_xk.getElement(j, 0));//把新状态记录下来
			}
			for (int k = 0; k < uNum; k++)
			{
				U_K.setElement(k, i, tmp_uk.getElement(k, 0));//把预测输出记录下来
			}
		}
		//copy输出值
		for (int i = 0; i < uNum; i++)
		{
			U.setElement(i, 0, U_K.getElement(i, 0));
		}
	}

	// 进行预测状态与实际状态的对齐存放: 输入要对齐的预测状态
	void compare_storage()
	{
		static MATRIX preStorage(xNum, preStep);
		static uint_t count = 0;
		real_t array_get[xNum] = { 0 };
		real_t array_put[2 * xNum] = { 0 };
		if (count > (preStep-1))
		{
			count = 0;
		}
		preStorage.getColArray(array_put, count);
		X.getColArray(array_put + xNum, 0);
		X_COMPARE.setArray(array_put, 2 * xNum);
		X_K.getColArray(array_get, preStep);
		preStorage.setColArray(array_get, count);
		count++;
	}

	// 获取输出值
	const MATRIX getOutput()
	{
		return U;
	}

	// 获取预测向量
	const MATRIX getPreState()
	{
		return X_K;
	}

	// 获取输出向量
	const MATRIX getPreCtrl()
	{
		return U_K;
	}

	// 获取对齐的预测与实际状态向量
	const MATRIX getCompareState()
	{
		return X_COMPARE;
	}

};



#endif

	