### 坐标系定义

相机坐标系为坐标系A
以pitch轴的中心点为原点的坐标系为坐标系B
以yaw轴的中心点为原点的坐标系为坐标系C
根据规则文件定义的物体的3D坐标在坐标系V下

x轴的正方向朝右，y轴正方向朝下，以右手定则确定z轴正方向（朝前）

### 变换关系

#### 状态输入

物体在坐标系A（相机坐标系）下的位置$P_A=[x_A,y_A,z_A]^T$姿态四元数为$q_A$

#### A→B变换

A相对于B平移了$(x_1,y_1,z_1)$（从机械得到数据）且绕B的x轴旋转了$\theta$

平移向量$t_{BA}$=$[x_1,y_1,z_1]^T$

旋转矩阵$R_{BA}$为绕x轴旋转$\theta$

$$
R_{BA}=\left[\begin{array}{ccc}{1}&{0}&{0}\\
{0}&{\cos\theta}&{-\sin\theta}\\
{0}&{\sin\theta}&{\cos\theta}\end{array}\right]
$$

#### B→C变换

B相对于C绕y轴旋转了$\gamma$

平移向量$t_{CB} = [0, 0, 0]^T$

旋转矩阵$R_{CB}$为绕y轴旋转$\gamma$

$$
R_{CB}=\left[\begin{array}{ccc}{\cos\gamma}&{0}&{\sin\gamma}\\
{0}&{1}&{0}\\
{-\sin\gamma}&{0}&{\cos\gamma}\end{array}\right]
$$

### 从A系到C系的变换-推导

坐标位置变换$P_A$→$P_C$，从A到B为$P_B=R_{BA}P_A+t_{BA}$，从B到C为$P_C=R_{CB}P_B+t_{CB}$，在$t_{CB}$为0时：

$$
\begin{array}{c}{P_{C}=R_{CB}\big(R_{BA}P_{A}+t_{BA}\big)}\\
{{P_{C}}={R_{CB}}{R_{BA}}{P_{A}}+{R_{CB}}{t_{BA}}}\end{array}
$$

对于四元数姿态变换$q_A$→$q_C$，假设四元数定义为$q=[w,x,y,z]$，对应于$R_{BA}$得到四元数$q_{BA}$（绕x轴旋转$\theta$：

$$
q_{BA} = [\cos(\frac{\theta}{2}), \sin(\frac{\theta}{2}), 0, 0]
$$

对应于$R_{CB}$得到四元数$q_{CB}$（绕y轴旋转$\gamma$）

$$
q_{CB}=[\cos(\frac{\gamma}{2}),0,\sin(\frac{\gamma}{2}),0]
$$

然后进行姿态叠加：物体相对于C的姿态 = (C相对于B的旋转)$\otimes$(B相对于A的旋转)$\otimes$(物体在A中的姿态)

$$
\mathbf{q_{C}}=\mathbf{q_{CB}}\otimes\mathbf{q_{BA}}\otimes\mathbf{q_{A}}
$$

### 从A系到C系的变换-结论

物体在C系下的位置为${P_{C}}={R_{CB}}{R_{BA}}{P_{A}}+{R_{CB}}{t_{BA}}={R_{CA}}{P_{A}}+{R_{CB}}{t_{BA}}$

其中第一项中$R_{CA}=R_{CB}\cdot R_{BA}=R_{y}(\gamma)\cdot R_{x}(\theta)$，展开得：

$$
R_{CA}=\left[\begin{array}{lll}{c_{\gamma}}&{0}&{s_{\gamma}}\\
{0}&{1}&{0}\\
{-s_{\gamma}}&{0}&{c_{\gamma}}\end{array}\right]\cdot\left[\begin{array}{lll}{1}&{0}&{0}\\
{0}&{c_{\theta}}&{-s_{\theta}}\\
{0}&{s_{\theta}}&{c_{\theta}}\end{array}\right]=\left[\begin{array}{lll}{c_{\gamma}}&{s_{\gamma}s_{\theta}}&{s_{\gamma}c_{\theta}}\\
{0}&{c_{\theta}}&{-s_{\theta}}\\
{-s_{\gamma}}&{c_{\gamma}s_{\theta}}&{c_{\gamma}c_{\theta}}\end{array}\right]
$$

第二项：

$$
R_{CB}\cdot t_{BA}=\left[\begin{array}{lll}{c_{\gamma}}&{0}&{s_{\gamma}}\\
{0}&{1}&{0}\\
{-s_{\gamma}}&{0}&{c_{\gamma}}\end{array}\right]\left[\begin{array}{l}{x_{1}}\\
{y_{1}}\\
{z_{1}}\end{array}\right]=\left[\begin{array}{c}{x_{1}c_{\gamma}+z_{1}s_{\gamma}}\\
{y_{1}}\\
{-x_{1}s_{\gamma}+z_{1}c_{\gamma}}\end{array}\right]
$$

合并得：

$$
\left\{\begin{array}{l}{x_{C}=c_{\gamma}x_{A}+s_{\gamma}s_{\theta}y_{A}+s_{\gamma}c_{\theta}z_{A}+\left(x_{1}c_{\gamma}+z_{1}s_{\gamma}\right)}\\
{y_{C}=c_{\theta}y_{A}-s_{\theta}z_{A}+y_{1}}\\
{z_{C}=-s_{\gamma}x_{A}+c_{\gamma}s_{\theta}y_{A}+c_{\gamma}c_{\theta}z_{A}+\left(-x_{1}s_{\gamma}+z_{1}c_{\gamma}\right)}\end{array}\right.
$$

对于姿态部分，$\mathbf{q_{C}}=\mathbf{q_{CB}}\otimes\mathbf{q_{BA}}\otimes\mathbf{q_{A}}$，由四元数乘法规则$q_{CA}=q_{CB}\otimes q_{BA}=[c_{g},0,s_{g},0]\otimes[c_{t},s_{t},0,0]$得到：

$$
q_{CA}=\left[\begin{array}{l}{c_{g}c_{t}}\\
{c_{g}s_{t}}\\
{s_{g}c_{t}}\\
{-s_{g}s_{t}}\end{array}\right]=\left[\begin{array}{l}{\cos\frac{\gamma}{2}\cos\frac{\theta}{2}}\\
{\cos\frac{\gamma}{2}\sin\frac{\theta}{2}}\\
{\sin\frac{\gamma}{2}\cos\frac{\theta}{2}}\\
{-\sin\frac{\gamma}{2}\sin\frac{\theta}{2}}\end{array}\right]
$$

但是仅仅得到相机到底盘坐标系的变换还不够，还需要将基于底盘坐标系C下拟合的信息转换为枪管坐标系A'下的信息

### 从A系到C系的变换-逆变换

在进行火控的弹道解算时，从yaw坐标系转换回枪管坐标系类似于相机转yaw坐标系的逆变换，但是需要注意不同点在于：
1. pitch坐标系下枪管的平移变换部分偏移量不同
2. 若将旋转变换视为内旋，先转XY轴再转Z轴，那么考虑到预测的需要，绕Z轴的旋转量会有所不同，设为$\mu_{new}$，相关公式推导如下：

#### 表示坐标系逆变换


对于平移变换，其表达形式为正向变换的逆变换，由正向公式$P_{C}=R_{CA}P_{A}+R_{CB}t_{BA}$，利用旋转矩阵的正交性($R^{-1}=R^T$)进行反解得到$P_{A}^{\prime}=R_{CA}^{T}(P_{C}-R_{CB}t_{BA})$

1. 平移项补偿

令$$\begin{array}{r}\\
{T_{\mathit{shift}}=R_{CB}t_{BA}=\left[\begin{array}{lll}{c_{\gamma}}&{0}&{s_{\gamma}}\\
{0}&{1}&{0}\\
{-s_{\gamma}}&{0}&{c_{\gamma}}\end{array}\right]\left[\begin{array}{l}{x_{1}}\\
{y_{1}}\\
{z_{1}}\end{array}\right]=\left[\begin{array}{l}{x_{1}c_{\gamma}+z_{1}s_{\gamma}}\\
{y_{1}}\\
{-x_{1}s_{\gamma}+z_{1}c_{\gamma}}\end{array}\right]}\end{array}$$

2. 应用逆旋转

$$R_{CA}^{T}=\left[\begin{array}{lll}{c_{\gamma}}&{0}&{-s_{\gamma}}\\
{s_{\gamma}s_{\theta}}&{c_{\theta}}&{c_{\gamma}s_{\theta}}\\
{s_{\gamma}c_{\theta}}&{-s_{\theta}}&{c_{\gamma}c_{\theta}}\end{array}\right]$$

3. 位置解析解
$$\left[\begin{array}{c}{x_{A}}\\
{y_{A}}\\
{z_{A}}\end{array}\right]=R_{CA}^{T}\left(\left[\begin{array}{c}{x_{C}}\\
{y_{C}}\\
{z_{C}}\end{array}\right]-\left[\begin{array}{c}{x_{1}c_{\gamma}+z_{1}s_{\gamma}}\\
{y_{1}}\\
{-x_{1}s_{\gamma}+z_{1}c_{\gamma}}\end{array}\right]\right)$$
令中间变量$\Delta P=P_{C}-T_{shift}=[\Delta x,\Delta y,\Delta z]^{T}$则：
$$\left\{\begin{array}{l}{x_{A}=c_{\gamma}\Delta x-s_{\gamma}\Delta z}\\
{y_{A}=s_{\gamma}s_{\theta}\Delta x+c_{\theta}\Delta y+c_{\gamma}s_{\theta}\Delta z}\\
{z_{A}=s_{\gamma}c_{\theta}\Delta x-s_{\theta}\Delta y+c_{\gamma}c_{\theta}\Delta z}\end{array}\right.$$

对于姿态变换，由$q_{C}=q_{CA}\otimes q_{A}$进行逆变换得到$q_{A}=q_{AC}\otimes q_{C}$，四元数逆变换是正向四元数的共轭，对于$q_{CA}=[w,x,y,z]$，其逆变换$q_{AC}=[w,-x,-y,-z]$

之所以要采用从pitch坐标系（C坐标系）开始是由于相机坐标系下的观测量是波动的、有噪声的，需要在C坐标系进行滤波和拟合后才能使用

$$
q_{AC}=\left[\begin{array}{l}{\cos\frac{\gamma}{2}\cos\frac{\theta}{2}}\\
{-\cos\frac{\gamma}{2}\sin\frac{\theta}{2}}\\
{-\sin\frac{\gamma}{2}\cos\frac{\theta}{2}}\\
{\sin\frac{\gamma}{2}\sin\frac{\theta}{2}}\end{array}\right]
$$
根据四元数乘法规则，$[w_{1},v_{1}]\otimes[w_{2},v_{2}]=[w_{1}w_{2}-v_{1}\cdot v_{2},w_{1}v_{2}+w_{2}v_{1}+v_{1}\times v_{2}]$得到$q_{A}=[w_{A},x_{A},y_{A},z_{A}]$

#### 旋转角度的剥离

由四元数$q_A$得到物体在相机坐标系A下的旋转矩阵$R_A$:
$$R_A = \begin{bmatrix}
1 - 2(y_A^2 + z_A^2) & 2(x_A y_A - w_A z_A) & 2(x_A z_A + w_A y_A) \\
2(x_A y_A + w_A z_A) & 1 - 2(x_A^2 + z_A^2) & 2(y_A z_A - w_A x_A) \\
2(x_A z_A - w_A y_A) & 2(y_A z_A + w_A x_A) & 1 - 2(x_A^2 + y_A^2)
\end{bmatrix} = \begin{bmatrix}
r_{11} & r_{12} & r_{13} \\
r_{21} & r_{22} & r_{23} \\
r_{31} & r_{32} & r_{33}
\end{bmatrix}$$

需要将虚拟平面上的点D（定点，表示能量机关击打区域中心）在虚拟坐标系V的$xOy$平面上先旋转$\Delta \mu$（按照解析表达式、相应拟合参数和与当前的时间差得到），再应用原始的$R_A$变换。其中虚拟坐标系下的旋转表示较简单，点D绕
Z轴旋转$\Delta \mu$后的坐标$D'=[x',y',0]^T$，表达式如下：

$$\left[\begin{array}{l}{x^{\prime}}\\
{y^{\prime}}\\
{0}\end{array}\right]=\left[\begin{array}{ll}{\cos(\Delta\mu)}&{-\sin(\Delta\mu)}&{0}\\
{\sin(\Delta\mu)}&{\cos(\Delta\mu)}&{0}\\
{0}&{0}&{1}\end{array}\right]\left[\begin{array}{l}{x}\\
{y}\\
{0}\end{array}\right]$$

展开得$$\begin{cases}
x' = x \cos(\Delta \mu) - y \sin(\Delta \mu) \\
y' = x \sin(\Delta \mu) + y \cos(\Delta \mu)
\end{cases}$$

#### 最终的解析解

总体思路是由D旋转$\Delta \mu$得到D'然后再执行$R_A$,$P_A$的原始变换得到枪管坐标系下的坐标，即：
$$\begin{array}{r}{P_{final}=R_{A}\cdot D^{\prime}+P_{A}=\left[\begin{array}{l}{r_{11}x^{\prime}+r_{12}y^{\prime}+x_{A}}\\
{r_{21}x^{\prime}+r_{22}y^{\prime}+y_{A}}\\
{r_{31}x^{\prime}+r_{32}y^{\prime}+z_{A}}\end{array}\right]}\end{array}$$