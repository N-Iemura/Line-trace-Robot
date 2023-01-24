## sd_sample_pkg
2022年度SD演習で製作したライントレーサーパッケージ。サンプルパッケージの名前の変更を怠ってそのまま使っている。ファイル整理も怠ったためたくさんのファイルが混在しているが、再現するには以下の３つのプログラムだけで良いのでここですべて説明する。

## ライントレーサーのメインプログラム
- ```robot_simulation.launch```
### 起動コマンド
```
roslaunch sd_sample_pkg robot_simulation.launch
```

![1](https://user-images.githubusercontent.com/79625740/214285768-c87d09ec-1b52-4621-a0d3-45e657eef93f.png)


## 遠隔操作プログラム
- ```scripts/teleop.py```

### 起動コマンド
```
roscd sd_sample_pkg/scripts
chmod +x　teleop.py
rosrun sd_sample_pkg teleop.py
```
(↑上から２行目までは権限付与用のため一度やればよい。)

1,2で左回りか右回りかを選択できる。

![2](https://user-images.githubusercontent.com/79625740/214286000-4681eb54-e3fe-4a2b-88b9-1c391f9cc17b.png)



## ライントレースプログラム
- ```follower_line_finder.py```

### 起動コマンド
```
roscd sd_sample_pkg/scripts
chmod +x　follower_line_finder.py
rosrun sd_sample_pkg follower_line_finder.py
```
(↑上から２行目までは権限付与用のため一度やればよい。)

下図のように3つの画面が現れ、MASKED, MASKED_yellow には青色、黄色ラインのみを移す画面、BGR Imageには青線の重心を赤点で現在のカメラに出力している。さらにBGR Imageの左上に```scripts/teleop.py```の1または2で左、右を指定したものが表示される。また、黄色線を検出したときには停止するようになっている。

![3](https://user-images.githubusercontent.com/79625740/214287231-addee6d0-a631-4b30-9d88-50ead22aff56.png)

