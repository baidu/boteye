# 环境录制

## 准备工作

* 脚本（.sh）文件中的第一行因release方式不同会有不同。直接二进制文件运行时在bin文件夹下从第二行开始运行即可。

* 完成校准，参见校准文档 (calibration.md)

* 设置环境变量。

```
source config_environment.sh
```

## 录轨迹

* 设置所要存储录制数据的默认文件夹：

```
record_path=$HOME/Boteye/data/???
```


* 执行命令
```
${MASTER_DIR}/pc_apps/scripts/record.sh 参数：存储录制数据的文件夹路径
```
or
```
${MASTER_DIR}/scripts/record.sh 参数：同上
```
如：
```
${MASTER_DIR}/scripts/record.sh $record_path
```
会出现这样的窗口

![录制窗口](https://preview.ibb.co/mTKr8d/Screenshot_from_2018_07_19_16_09_25.png)

* 如果出现了bad calibration字样，说明校准有问题，如下

![校准问题](https://preview.ibb.co/if5L1y/Screenshot_from_2018_07_19_16_09_38.png)
