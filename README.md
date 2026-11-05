# STM32 双轮底盘

`car_slam` 的 STM32 下位机工程：STM32F103、TB6612 双电机驱动、双轮编码器和串口通信。

- USART1 接收左右轮目标转速，字段单位为 **0.1 RPM**。
- TIM2 以 **100 Hz** 执行双轮 PD 闭环；当前 P=10、D=0.02，无积分和前馈。
- 最大目标转速 **±120 RPM**，PWM 输出上限 **1200**，编码器每圈计数 **1468.2**。
- 每 **20 ms** 回传编码器累计计数及 STM32 采样时间。
- 收到零目标停止；无新指令时保持最后目标，未设置下位机通信超时停车。

## 文件

| 路径 | 内容 |
| --- | --- |
| `Core/` | 主程序、外设初始化、中断及控制逻辑 |
| `Drivers/` | STM32 HAL 与 CMSIS 源码及原始许可证 |
| `car_swarm.ioc` | CubeMX 外设配置 |
| `STM32F103xx_FLASH.ld` | 当前板使用的 32 KB Flash、10 KB RAM 链接配置 |
| `PWM_TUNING.md` | 引脚接线、HEX 协议及参数说明 |

## 编译与烧录

安装 `arm-none-eabi-gcc`、`make` 和 `st-flash` 后，在工程目录执行：

```bash
make -j2
st-flash --reset write build/car_swarm.bin 0x08000000
```

CubeMX 重新生成工程后，需检查链接脚本仍与当前板的容量一致。PD 参数尚需带载实测整定，极低转速不保证起动。
