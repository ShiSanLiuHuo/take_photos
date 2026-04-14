# camera
相机启动节点
继承自老代码

## 发布话题
- `/image_for_radar` - 相机图像

## Test camera fps
```shell
colcon test --packages-select camera
colcon test --event-handlers console_direct+ --packages-select camera
```

## 运行节点
```
ros2 run camera camera_node
```
按q键退出调整界面

## 参数设置
- 曝光时间：在 20000 us时，fps为50