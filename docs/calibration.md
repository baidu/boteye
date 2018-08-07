# 模组校准



## 准备工作

* 脚本（.sh）文件中的第一行因release方式不同会有不同。直接二进制文件运行时在bin文件夹下从第二行开始运行即可。

* 设置环境变量。在根目录下运行：

```
source config_environment.sh
```

* 执行命令, 新建校准相关的文件夹:
```
${MASTER_DIR}/pc_apps/scripts/folders.sh
```
or
```
${MASTER_DIR}/scripts/folders.sh
```

* A4纸打印ApritTag5x7.png

* 精确测量二维码方格边长（0.03m左右）

* 把A4纸平摊的贴到硬的纸板上

![矫正板](https://ai.bdstatic.com/file/825D9CFB4EFF48289B19FC501A14BA1A)


## 采集矫正图片

执行命令:
```
${MASTER_DIR}/pc_apps/scripts/calib_data.sh
```
or
```
${MASTER_DIR}/scripts/calib_data.sh
```


* 共采60张左右

* 消去所有图中表示未覆盖的红色格子

![红色格子](https://ai.bdstatic.com/file/2704775EC4B2403D915E5253B11CC724)

* 样例图片

![样例](https://ai.bdstatic.com/file/EAB9159088DE48F2B238D5B60F31A6C7)

* 取得理想的覆盖效果：

![覆盖效果](https://image.ibb.co/dq4Z5J/calib_reproj.png)

## 校准

* 修改 `scripts/calib_compute.sh` 中的square size之后执行

