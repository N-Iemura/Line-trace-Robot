## sd_sample_pkg

ROSを用いたロボットシミュレーション演習講義における、応用課題についてのプログラム


### ライントレーサーのgazebo起動

# terminal 1
roslaunch sd_sample_pkg robot_simulation.launch

### 遠隔操作のサンプルプログラム

- ```scripts/teleop.py```

# terminal 2
rosrun sd_sample_pkg teleop.py
```


## シミュレーション

- ロボットモデル: ```urdf/robot.urdf.xacro```
- サンプルのworld: ```worlds/sample.world```
