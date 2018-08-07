# 导航程序

## 准备工作

* 脚本（.sh）文件中的第一行因release方式不同会有不同。直接二进制文件运行时在bin文件夹下从第二行开始运行即可。
* 完成校准，参见校准文档 (calibration.md)
* 完成环境录制，参见环境录制 (record.md)
* 底盘设置，参见底盘文档（actuator.md)
* 设置环境变量。

```
source config_environment.sh
```

* 设置已录制数据的默认文件夹：


```
record_path=$HOME/Boteye/data/???
```

* 执行导航数据生成程序：

```
python ${MASTER_DIR}/pc_apps/navigation/pre_navigation.py --record_path=$record_path
```

or
```
python ${MASTER_DIR}/bin/pre_navigation.py --record_path=$record_path
```


* 如果binary不在`${MASTER_DIR}/build/` 请加 `--build_folder=binary所在文件夹`

* 确认设备权限，比如

```
sudo chmod 777 /dev/ttyUSB0
```


## 导航模式

* 导航有两种模式，控制参数在

```
${MASTER_DIR}/XP/config/navigation_param.yaml -> Navigation/mode
```

* 鼠标控制模式 (control)

```
mode: control
```

* 指定轨迹模式 (loop)

```
mode: loop
```

## 鼠标控制导航
* 确认XP/config/navigation_param.yaml中的use_trajectory_file值为false。

* 执行导航命令:
```
${MASTER_DIR}/pc_apps/scripts/navi.sh 参数：之前导航数据生成程序生成的导航文件夹路径
```
or
```
${MASTER_DIR}/scripts/navi.sh 参数：同上
```
如：
```
${MASTER_DIR}/scripts/navi.sh $record_path/navigation/
```
* 出现以下窗口：

![导航](https://image.ibb.co/hfNa1y/Screenshot_from_2018_07_19_16_28_31.png)


* 机器人会原地旋转至确定位置，左上角会显示normal，右边两个圆会变成一个。

![recloc_done](https://image.ibb.co/ecQETd/Screenshot_2018_07_19_16_34_42.png)

* 在以下窗口总点击目标点，出现蓝色的规划轨迹，机器人开始导航

![导航路线](https://image.ibb.co/ngvyod/Screenshot_from_2018_07_19_16_35_29.png)

## 指定轨迹导航
* 确认XP/config/navigation_param.yaml中的use_trajectory_file值为true。
### 生成轨迹，执行：

```
${MASTER_DIR}/build/pc_apps/navigation/trajectory_maker --navigation_folder=$record_path/navigation/
```
or
```
${MASTER_DIR}/bin/trajectory_maker --navigation_folder=$record_path/navigation/
```

  * 左边为可操作区域，右边显示规划路径

![trajectory_maker](https://image.ibb.co/hchB8d/Screenshot_from_2018_07_19_16_52_15.png)


  * 点击左边图，按空格，按p，确定起点

![start_point](https://image.ibb.co/eWAngy/Screenshot_from_2018_07_19_16_52_31.png)

  * 再在左边图点击目标点，按空格，确定第一个目标点, 按p规划第一段路径

![first_point](https://preview.ibb.co/dLnNEJ/Screenshot_from_2018_07_19_16_53_14.png)


  * 再点下一个目标点，按空格确认，按p规划路径。

![second_point](https://image.ibb.co/e6wM8d/Screenshot_from_2018_07_19_16_53_35.png)

  * 如此重复直到按ESC结束。


### 开始导航

* 执行 导航命令：
```
${MASTER_DIR}/pc_apps/scripts/navi.sh 参数：之前导航数据生成程序生成的导航文件夹路径
```
or
```
${MASTER_DIR}/scripts/navi.sh 参数：同上
```
如：
```
${MASTER_DIR}/scripts/navi.sh $record_path/navigation/
```
