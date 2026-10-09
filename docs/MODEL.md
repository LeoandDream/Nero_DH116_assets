# 模型与已知边界

## 机械来源

源URDF决定质量、惯性、关节轴、限位和线性mimic。没有采用旧DH116 MJCF的二次mimic或额外机械damping／armature替代源机制。源中缺失的机械项编译成默认值，不等于真实硬件测量为零。

DH116为左DH116-L000-A1，不能当作右手、DH116S或V2。`nero_plus_manufacturer_flange.urdf`是来源有说明的组合描述，不宣称是厂商验证的DH安装方案。

## 接触近似

MuJoCo接触使用分块凸代理；可视源mesh不等于实际碰撞表面。当前手部76块、机械臂51块、连接件8块，内部过滤只保留base_link/link1这一直接相邻对。没有导入整份Never清单或关闭手／物体碰撞。

该近似改善了先前粗代理表达，但有限静态核对不证明整个动作空间，也不证明真实材料／摩擦／驱动机制已校准。模型没有包含训练实验的厨房、碗或额外支撑。

## 控制含义

robot state：NERO7 qpos＋DH11611 qpos，顺序见robot-contract.json。

robot action：前7维绝对关节位置rad，经过原生JointPositionController；后6维独立归一化绝对手位置[-1,1]，再转换到源主动关节角。

MJCF ctrl：前7维motor torque Nm，后6维position actuator的关节角目标rad；维度相同并不表示语义相同。

手位置servo是MuJoCo内置position模式。固定gain根据源惯性与既有MJCF响应参考选取，没有新增底层控制器；不等于硬件标定增益。真实手力矩／速度上限未知，源URDF effort／velocity=0仍是缺项，不伪造额定上限。

## 静态交付核对

模型可由MuJoCo3.2.3编译。按同名body／joint／geom／actuator比较质量、惯性、轴、范围、机械项、接触项、gain、五组mimic与参考姿态均保持；18个标量关节、13个actuator、5个equality及1个exclude。

查看入口只reset到参考姿态并mj_forward，随后显示或导出图像；没有物理时间步进、轨迹重放、策略推理或训练。静态图片是交接模型视图，不是抓持成功证据。
