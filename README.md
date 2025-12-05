# おちまものTurtlebot3 | BLE編
このbranchは、LiDARとカメラから人の位置を推定した後、危険であるかどうかを判断するノード。危険である場合は、「危険」つまり”1”というメッセージを受信するM5Stackに送る。

2つのフォルダーがあり、
- turtlebot: ROS2ノードで書かれたコードのため、ROSをインストールしない場合は動けません。しかし、コードはPythonで書かれているため、ROSを使わずに工夫すれば自分のパソコンでも使える。
- m5stack: M5Stackに使うコードで、turtlebotからもらった信号を基に警報を出す。

## ソースコードの確認
> おすすめな方法は、VSCodeでリモート修正を行う。
> ref: https://qiita.com/ist-sh-ha/items/359df9097cf14d2f7868

- RaspberryPi上では、`~/ble_warn/src/ble_warn/ble_warn`にソースコードが置かれている。
- **すべてのコードはRaspberryPi上で実行してください**
- なお、このノードはLiDARノードの出力を聞くため、LiDARノードも同時実行する必要がある。
- 5秒間以内にM5Stackが見つからない場合は終了する。

- このノードに動かしたい場合は、
``` bash
# ~/yolo_lidar/src/で実行
# パッケージをビルド
# RaspberryPi上でパイがすでにビルドしたので、改めて実行する必要はない
colcon build

# ホームダイレクトリーに実行
cd ~
ros2 run ble_warn ble_warn
```