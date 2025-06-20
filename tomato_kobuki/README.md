# Kobuki Operation
Kobukiを動かすためのパッケージ

## How to
以下の2つのノードが用意されています．それぞれ実行して動作を確認してください．

1. kobuki_key_operation

    Kobukiをキーボードで操作するノードです．
    操作方法はノードを実行するとターミナル上に表示されます．
    このノードを実行しているターミナルがアクティブになっていないと，
    キー入力が受け付けられないので注意してください．

    ターミナルを開き，次のコマンドでGazeboのworldを立ち上げます．
    ~~~
    $ roslaunch turtlebot_gazebo turtlebot_world.launch
    ~~~
    次に別のターミナルで以下のようにノードを起動します．
    ~~~
    $ rosrun kobuki_operation kobuki_key_operation
    ~~~
    キーボード操作でGazebo内のKobukiが動かせるはずです．

2. kobuki_custom_operation

    あらかじめ指定した通りにKobukiを動作させるノードです．
    デフォルトではKobukiが繰り返し前後に動くようプログラムされています．

    ターミナルを開き，次のコマンドでGazeboのworldを立ち上げます．
    ~~~
    $ roslaunch turtlebot_gazebo turtlebot_world.launch
    ~~~
    次に別のターミナルで以下のようにノードを起動します．
    ~~~
    $ rosrun kobuki_operation kobuki_custom_operation
    ~~~
    しばらくするとKobukiが勝手に動き出します．

## TODO
プログラムを改造して独自の動作に作り変えてください．

1. 回避行動の改造

    上記の2つのノードは，障害物にぶつかると回避行動を取るように指令を出します．
    デフォルトでは単に後退するだけになっているので，自由に改造してください．
    
    回避行動のプログラムは`src`の中の`kobuki_avoidance_reaction.cpp`に書かれています．
    以下の3つの関数を使って回避行動を作ってみましょう．

    - `kobukiMove(speed,turn)`

        Kobukiを指定の速度で動かします．speedには前進速度，turnには回転速度を，それぞれ小数で指定します．
    
    - `kobukiKeep(duration)`

        durationで指定した秒数だけ状態を維持します．
    
    - `kobukiStop()`

        Kobukiを急停止させます．

2. 一連の動作をプログラム

    1.と同じ要領で`kobuki_custom_reaction.cpp`を改造して，Kobukiの一連の動作をプログラムしてください．


