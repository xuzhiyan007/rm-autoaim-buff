### 坐标系定义

相机坐标系为坐标系A
以pitch轴的中心点为原点的坐标系为坐标系B
以yaw轴的中心点为原点的坐标系为坐标系C

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

### 从A系到C系的逆变换

对于平移变换，其表达形式为正向变换的逆变换，由$P_{C}=R_{CA}P_{A}+T_{total}$得到其逆变换$P_{A}=R_{CA}^{-1}\big(P_{C}-T_{total}\big)$由于旋转矩阵是正交矩阵，故其逆等于转置，所以

$$
P_{A}=\left[\begin{array}{lll}{c_{\gamma}}&{0}&{-s_{\gamma}}\\
{s_{\gamma}s_{\theta}}&{c_{\theta}}&{c_{\gamma}s_{\theta}}\\
{s_{\gamma}c_{\theta}}&{-s_{\theta}}&{c_{\gamma}c_{\theta}}\end{array}\right]\left(\left[\begin{array}{l}{x_{C}}\\
{y_{C}}\\
{z_{C}}\end{array}\right]-\left[\begin{array}{l}{x_{1}c_{\gamma}+z_{1}s_{\gamma}}\\
{y_{1}}\\
{-x_{1}s_{\gamma}+z_{1}c_{\gamma}}\end{array}\right]\right)
$$

设枪管相对pitch坐标系的偏移量为$t_{BA}'=[x_1',y_1',z_1']^T$中间变量$\Delta x=x_{C}-\left(x_{1}'c_{\gamma}+z_{1}s_{\gamma}\right)$，$\Delta y=y_{C}-y_{1}'$，$\Delta z=z_C-(-x_1's_{\gamma}+z_1'c_{\gamma})$得到

$$
\left\{\begin{array}{l}{x_{A}=c_{\gamma}\Delta x-s_{\gamma}\Delta z}\\
{y_{A}=s_{\gamma}s_{\theta}\Delta x+c_{\theta}\Delta y+c_{\gamma}s_{\theta}\Delta z}\\
{z_{A}=s_{\gamma}c_{\theta}\Delta x-s_{\theta}\Delta y+c_{\gamma}c_{\theta}\Delta z}\end{array}\right.
$$
