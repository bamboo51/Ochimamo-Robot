# おちまものTurtlebot3 | LiDAR・カメラ編
このbranchは、LiDARから読み取ったセンサーの値とカメラから読み取った画像を組み合わせ、人がいる位置を推定する。

## ソースコードの確認
> おすすめな方法は、VSCodeでリモート修正を行う。
> ref: https://qiita.com/ist-sh-ha/items/359df9097cf14d2f7868

- RaspberryPi上では、`~/yolo_lidar/src/people_mapper_pkg/people_mapper_pkg`にソースコードが置かれている。
- **すべてのコードはRaspberryPi上で実行してください**

- このノードに動かしたい場合は、
``` bash
# ~/yolo_lidar/src/で実行
# パッケージをビルド
# RaspberryPi上でパイがすでにビルドしたので、改めて実行する必要はない
colcon build

# このパッケージを動かすために必要なパッケージ（別々のターミナルで実行）
# ロボットを動かすため
ros2 launch turtlebot3_bringup robot.launch.py
# カメラを配信
ros2 run camera_ros camera_node --ros-args -p format:='RGB888' -p width:=320 -p height:=240

# ホームダイレクトリーに実行
cd ~
ros2 run people_mapper_pkg people_mapper_node
```