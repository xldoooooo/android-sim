package com.xldoooooo.androidsim.sim

/**
 * 所有动力学模型实现此接口
 * 状态方程：ẋ = f(t, x, u)
 *
 * @param stateDim  状态维度
 * @param ctrlDim   控制输入维度
 */
interface DynamicsModel {
    val stateDim: Int
    val ctrlDim: Int

    /**
     * 状态导数
     * @param t 当前时间
     * @param x 当前状态向量
     * @param u 控制输入向量
     * @return  ẋ，状态导数向量
     */
    fun deriv(t: Double, x: DoubleArray, u: DoubleArray): DoubleArray
}
