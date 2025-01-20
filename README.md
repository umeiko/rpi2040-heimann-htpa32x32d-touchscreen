
# 【Umeko-DEC-H】海曼htpa性价比口袋热成像仪
`rpi2040-heimann-htpa32x32d-touchscreen`
- 三维模型
- 硬件PCB工程
- 源代码

# 编译事项
- 使用`Arduino-pico`开发
- MCU超频到`250Mhz`
- 使用`-O3`优化

# 简介
相比于`mlx90640`增加了三分之一的像素，画面更加细腻了一些。同时也使用了很受欢迎的树莓派`rp2040`作为mcu主控。三个按钮，分别用作重启，测温点控制，以及画面暂停的功能。开关打开直接开机使用，非常的灵敏。触摸屏可以触摸选点，非常的符合直觉。硬件工程开源在[立创开源平台](https://oshwhub.com/umekoko/hai-man-htpa-chao-mi-ni-re-cheng-xiang-yi)

如何搭建开发环境参考[mlx90640项目](https://github.com/umeiko/RP2040-MLX90640-touchscreen-arduino)，这里不过多赘述。