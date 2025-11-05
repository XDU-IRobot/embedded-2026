# OpenOCD 调试配置

`openocd/` 目录下存放了 C 板对应的 stlink 和 daplink 的 OpenOCD 配置文件，以及 C 板对应的 svd 外设描述文件，调试时在 Clion、Ozone 中自行配置使用。

## CLion

1. Edit Configurations...

![Image](https://github.com/user-attachments/assets/a201bc21-2a59-4f15-afd7-fb0f9071003b)

2. 添加 OpenOCD 配置

![Image](https://github.com/user-attachments/assets/41319dec-275b-4f37-bb3c-ca026fe59027)

3. 选择可执行文件、根据调试器类型填 cfg 配置文件位置

![Image](https://github.com/user-attachments/assets/fae62b07-1250-432f-8eeb-49d8b88e9f55)

4. 切换运行配置到创建的 OpenOCD 配置，点击 Debug 按钮即可开始调试

![Image](https://github.com/user-attachments/assets/f784f7d7-0490-401d-aaf2-c7d1b5c192ac)
