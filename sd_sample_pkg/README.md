## sd_sample_pkg
2022年度SD演習で製作したライントレーサーパッケージ。サンプルパッケージの名前の変更を怠ってそのまま使っている。

## ライントレーサーのメインプログラム
- ```robot_simulation.launch```
### 起動コマンド
roslaunch sd_sample_pkg robot_simulation.launch

## 遠隔操作プログラム
- ```scripts/teleop.py```

### 起動コマンド
```
roscd sd_sample_pkg/scripts
chmod +x　teleop.py
rosrun sd_sample_pkg teleop.py
```
↑上から２行目までは権限付与用のため一度やればよい。


## ライントレースプログラム
- ```follower_line_finder.py```

### 起動コマンド
```
roscd sd_sample_pkg/scripts
chmod +x　follower_line_finder.py
rosrun sd_sample_pkg follower_line_finder.py
```
↑上から２行目までは権限付与用のため一度やればよい。
