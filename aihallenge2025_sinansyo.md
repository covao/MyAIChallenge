自動運転 AI Challenge 2025 指南書
===========
# 概要
- この指南書は、Autowareを使った自動運転競技である自動運転AI チャレンジ 2025の開発環境構築やノウハウをまとめたものです。
- この指南書はAIチャレンジへの取り組みを通じ、ユーザー自身の体験やAIとの対話を通じてアップデートしていきます
-  AIエージェントはこの指南書のコンテキストを理解し、ユーザーの目標達成を支援します。
# 目標
- 自動運転AIチャレンジのシミュレータ構築
- ユーザーが使いやすいシミュレータ環境の構築
- Autowareのパラメータを調整し、自動運転AIチャレンジのコースを完走する
# PC環境
-  OS: Windows 11
- CPU: Intel(R) Core(TM) Ultra 7 155H (3.80 GHz)
- NVIDIA GPUなし
- RAM: 32 GB
- AI Challengeの仮想環境 WSL
　　- OS: Ubuntu 22.04 LTS
　　- プロセッサ数: 22
　　- メモリサイズ: 16GB
　　-  スワップサイズ: 32GB

# シミュレータのインストール
- [自動運転AIチャレンジ 2025 ソースコード一式 GitHub リポジトリ ](https://github.com/AutomotiveAIChallenge/aichallenge-2025)
~~~ bash
cd ~ 
git clone https://github.com/AutomotiveAIChallenge/aichallenge-2025.git
~~~
- 以降は、[セットアップ手順](https://automotiveaichallenge.github.io/aichallenge-documentation-2025/setup/workspace-setup.html)に従う


# チートシート
## Dockerコンテナ外の操作コマンド

### リポジトリのアップデート
~~~ bash
cd ~ # Move to home directory
git clone https://github.com/AutomotiveAIChallenge/aichallenge-2025.git # Clone repository
cd aichallenge-2025 # path to aichallenge2025
git pull origin main # Update to latest files
~~~

### Dockerコンテナのビルド
~~~
# AI Challenge 2025 Docker Build
cd ~/aichallenge-2025
./docker_build.sh dev
docker images
~~~

### Dockerコンテナ起動(CPU版のAWSIM)
~~~
cd ~/aichallenge-2025
./docker_run.sh dev cpu # Launch the Docker container with CPU support
~~~

## Dockerコンテナ内の操作コマンド

### Simulatorの起動
~~~ bash
./run_evaluation.bash  # launch Autoware from the container
~~~
### Simulatorの車両走行開始
~~~ bash
./publish.bash all # Start simulator and publish all topics
~~~
### Simulatorのプロセス終了
~~~ bash
./pkill_all.bash # kill all processes
~~~


### rqt　ROSトピックやノードをモニターするための GUIツール

~~~ bash
# if rqt is not installed, install it
if ! command -v rqt &> /dev/null; then
    echo "rqt is not installed. Installing now..."
    sudo apt update && sudo apt install -y ros-humble-rqt
    # Install all additional rqt plugins]
    sudo apt install -y ros-humble-rqt-common-plugins ros-humble-rqt-image-view ros-humble-rqt-plot ros-humble-rqt-bag
else
    echo "rqt is already installed."
fi
~~~


### rosboardの起動
~~~
cd rosboard
source /opt/ros/humble/setup.bash
./run
~~~
### ROS rqtの起動

~~~ bash
source /opt/ros/humble/setup.bash
rqt --force-discover # Start rqt with force discover to load all plugins
~~~

### ROS Turtle Sim起動
~~~ bash
# run turtlesim with random motion
source /opt/ros/humble/setup.bash
ros2 run turtlesim turtlesim_node
~~~
### ROS Turtle Sim Telopの起動
~~~ bash
# Turtle Sim Teleop
source /opt/ros/humble/setup.bash
ros2 run turtlesim turtle_teleop_key
~~~


# 便利なツール
## Dockerコンテナ内の追加ツール
### ファイルブラウザ Nautilus
~~~ bash
if ! command -v nautilus &> /dev/null; then
    echo "Nautilus is not installed. Installing now..."
    sudo apt update && sudo apt install -y nautilus
else
    echo "Nautilus is already installed."
fi
sudo nautilus /aichallenge # Open file browser at /aichallenge in the container
~~~
### テキストエディター gedit
~~~ bash
if ! command -v gedit &> /dev/null; then
    echo "gedit is not installed. Installing now..."
    sudo apt update && sudo apt install -y gedit
else
    echo "gedit is already installed."
fi
gedit
~~~
### ROSのTurtle Sim
- セットアップ
~~~ bash
# Turtle Sim Demo
# Check if turtlesim is installed, if not, install it
if ! command -v turtlesim_node &> /dev/null; then
    echo "turtlesim is not installed. Installing now..."
    sudo apt update && sudo apt install -y ros-humble-turtlesim
fi
~~~ bash

~~~

# ROS2基本コマンド

### Source ROS2 Humbleの起動
~~~
source /opt/ros/humble/setup.bash
~~~
### トピック一覧表示
~~~
ros2 topic list # List all topics
~~~
### ノード一覧表示
~~~
ros2 node list # List all nodes
~~~


# Docker基本コマンド
### Dockerコンテナ一覧表示
~~~ bash
docker images # List all images
~~~

### 実行中のDockerを保存する
~~~ bash
docker commit -m "Update Autoware" -a "Your Name" aichallenge2025_dev aichallenge2025_dev_updated
~~~

### 不要なDockerイメージを削除する
~~~ bash
docker rmi $(docker images | grep "^<none>" | awk '{print $3}') # Remove dangling images
~~~
### Dockerイメージとコンテナを全て強制削除
~~~ bash
docker rm -f $(docker ps -aq)      # ALl Container
docker rmi -f $(docker images -aq) # All Image
~~~



# Simulator パラメータファイル
/aichallenge/workspace/src/aichallenge_submit/aichallenge_submit_launch/launch/reference.launch.xml


# 関連情報

- [自動運転AIチャレンジ  環境構築](https://automotiveaichallenge.github.io/aichallenge-documentation-2025/setup/requirements.html)
-  [自動運転AIチャレンジ に関する技術ブログ Qiita Advent Calendar 2024](https://qiita.com/advent-calendar/2024/jidounten-ai)
- [Autoware入門講座](https://automotiveaichallenge.github.io/aichallenge-documentation-2025/course/index.html)
- [オンライン採点環境](https://aichallenge-board.jsae.or.jp/)
- [ROS 2 Documentation](https://docs.ros2.org/)
- [Autoware Documentation](https://autowarefoundation.github.io/autoware.auto/)
