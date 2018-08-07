# 底盘设置

* 设置选项在navi.yaml中

## 底盘品牌

* 步科底盘设

```
--actuator=kinco
```

* UDP速度角速度信息设

```
--actuator=UDP
```

UDP接收端参考下面UDP协议接收导航信息部分

## 底盘安装方向

* 反向设

```
reverse_heading=true
```

## UDP协议接收导航信息

* `guide_receiver` 通过UDP传输的方式从app_tracking处接收导航信息，并打印在命令行窗口中。

* 开启另一个命令行窗口，运行 `./guide_receiver`

* `guide_receiver` 收到的信息格式为：
guide message from [IP]: [PORT] [A, B, C]
 * 用于接收UDP协议的默认端口是8889，IP地址没有限制
 * A为当前线速度控制量（单位：m/s）
 * B为当前角速度控制量（单位：rad/s，逆时针为正）
 * C为导航状态（0：失败，1：正确，2：停止，3：到达目的地结束，4：SLAM定位丢失，5：避障）。

