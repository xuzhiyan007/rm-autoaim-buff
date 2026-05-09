Z轴是朝前的，由于三维旋转的顺序不可交换，为方便后续拟合，定义旋转顺序为X->Y->Z
整体旋转矩阵可以理解为$R_x(\alpha)$为绕X轴旋转$\alpha$，$R_y(\beta)$为绕Y轴旋转$\beta$，$R_z(\gamma)$为绕Z轴旋转$\gamma$同时，注意“绕Z轴旋转”的Z轴为旋转后的Z轴，那么就是一个内旋问题，旋转矩阵应该表述为$$R=R_{x}(\alpha)R_{y}(\beta)R_{z}(\gamma)$$
其中：
$$R_{x}=\left[\begin{array}{lll}{1}&{0}&{0}\\
{0}&{c_{\alpha}}&{-s_{\alpha}}\\
{0}&{s_{\alpha}}&{c_{\alpha}}\end{array}\right],\quad R_{y}=\left[\begin{array}{lll}{c_{\beta}}&{0}&{s_{\beta}}\\
{0}&{1}&{0}\\
{-s_{\beta}}&{0}&{c_{\beta}}\end{array}\right],\quad R_{y}=\left[\begin{array}{lll}{c_{\gamma}}&{-s_{\gamma}}&{0}\\
{s_{\gamma}}&{c_{\gamma}}&{0}\\
{0}&{0}&{1}\end{array}\right]$$
各旋转矩阵定义如上
对R矩阵展开得：
$$R=R_{x}R_{y}R_{z}=\left[\begin{array}{lll}{c_{\beta}}&{0}&{s_{\beta}}\\
{\dots}&{\dots}&{\dots}\\
{\dots}&{\dots}&{\dots}\end{array}\right]\left[\begin{array}{lll}{c_{\gamma}}&{-s_{\gamma}}&{0}\\
{s_{\gamma}}&{c_{\gamma}}&{0}\\
{0}&{0}&{1}\end{array}\right]$$
为便于计算的解耦，计算$R$矩阵的第一行前两列（不包含$\alpha$），第一行的元素中：
$$\begin{array}{c}{r_{00}=c_{\beta}\cdot c_{\gamma}+0\cdot s_{\gamma}+s_{\beta}\cdot 0=\cos\beta\cos\gamma}\\
{r_{01}=c_{\beta}\cdot\left(-s_{\gamma}\right)+0\cdot c_{\gamma}+s_{\beta}\cdot 0=-\cos\beta\sin\gamma}\end{array}$$
消去$\cos(\beta)$得：
$${\frac{-r_{01}}{r_{00}}}={\frac{-{\bigl(}-\cos\beta\sin\gamma{\bigr)}}{\cos\beta\cos\gamma}}={\frac{\sin\gamma}{\cos\gamma}}=\tan\gamma$$
$$\gamma=\mathrm{arctan2}(-r_{01},r_{00})$$
再引入四元数到矩阵的变换，此处可通过Rodrigues函数实现，也可以解析表达：
$$\begin{array}{c}{r_{00}=1-2(y^{2}+z^{2})}\\
{r_{01}=2(xy-wz)}\end{array}$$
代入以上关于$\gamma$的表达式得：
$$\gamma=\arctan 2\left(2(wz-xy),\quad 1-2(y^{2}+z^{2})\right)$$
