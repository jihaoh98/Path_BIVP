# Path_BIVP
## Experimental result
P.S Maybe you can't see the graph in the readme.pdf, please see the result in the [README.md](https://github.com/jihaoh98/Path_BIVP/blob/main/README.md)!

![result1](https://github.com/jihaoh98/Path_BIVP/blob/main/graph/res1.png)

![result2](https://github.com/jihaoh98/Path_BIVP/blob/main/graph/res2.png)

![result3](https://github.com/jihaoh98/Path_BIVP/blob/main/graph/res3.png)

![result4](https://github.com/jihaoh98/Path_BIVP/blob/main/graph/res4.png)

![result5](https://github.com/jihaoh98/Path_BIVP/blob/main/graph/res5.png)

![result7](https://github.com/jihaoh98/Path_BIVP/blob/main/graph/res7.png)
## Analysis
>这次主要是针对于BIVP问题进行求解,BIVP问题表示起始点和终止点的position, velocity, acceleration是给定的,而中间的路径点只固定相应的position,每一段的时间也是给定的.本次实验对象是minimum jerk trajectory, 因此s=3,从而可以得到采用五阶多项式去拟合轨迹是最优的,另外在整个BIVP问题中,始终是保持continuous snap的, 因此可以作为边界条件使用.

>每一段轨迹之间的时间根据梯形公式来确定, 给定最大速度以及加速度,可以计算一个比较合理的时间.

>设一共有n段轨迹,由于采用五阶多项式去拟合,因此每一段共有6个参数,整个轨迹便具有6*n个轨迹,6n个方程的来源:
* 起始点和终止点的p,v,a可以作为6个方程
* 共有n段轨迹,所以有n-1个中间点, 这些中间点的位置是给定的,因此可以作为n-1个方程
* 另外在这n-1个中间点处满足snap continuous, snap continuous即表示position, velocity, acceleration, jerk, snap都是连续的,因此还可以作为5*(n-1)个方程
* 综上,共有6 + n - 1 + 5 * (n - 1) = 6n - 6 + 6 = 6n个方程,可以解出各段轨迹的系数,从而得到轨迹
>可以根据c = M.inverse() * b来得到最终每一段的系数

## 核心代码
P.S 下面的M,c即为Mc=b中的变量

[计算M](https://github.com/jihaoh98/Path_BIVP/blob/main/src/click_gen.cpp#:~:text=//%20%2D%2D%2D%2D%2D%2D%2D%2D%2D%2D%2D%2D%2D%2D%2D%2D%2D%2D%2D%2D%2D%2D%2D%2D%20Put%20your%20solution%20below%20%2D%2D%2D%2D%2D%2D%2D%2D%2D%2D%2D%2D%2D%2D%2D%2D%2D%2D%2D%2D%2D%2D%2D%2D)

[计算系数矩阵c](https://github.com/jihaoh98/Path_BIVP/blob/main/src/click_gen.cpp#:~:text=coefficientMatrix%20%3D%20Condition_matrix.inverse()%20*%20boundary_matrix%3B)






  
