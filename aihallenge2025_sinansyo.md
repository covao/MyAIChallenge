自動運転 AI Challenge 2025 指南書
===========
# 概要
- この指南書は、Autowareを使った自動運転競技である自動運転AI チャレンジ 2025の開発環境構築やノウハウをまとめたものです。
- この指南書は自動運転AIチャレンジへの取り組みを通じて、AIとの対話しながらユーザー自身の体験をもとにアップデートしていきます
-  AIエージェントがこの指南書のコンテキストを深く理解し、ユーザーの目標達成を支援することを想定しています。

# 目標
- 自動運転AIチャレンジのシミュレータ構築
- ユーザーが使いやすいシミュレータ環境の支援
- Autowareのパラメータを調整し、自動運転AIチャレンジのコースを完走する
# 使用したPC環境 
- OS: Windows 11
- CPU: Intel(R) Core(TM) Ultra 7 155H (3.80 GHz)
- NVIDIA GPUなし
- RAM: 32 GB
- AI Challengeの仮想環境 WSL
  - OS: Ubuntu 22.04 LTS
  - プロセッサ数: 22
  - メモリサイズ: 16GB
  - スワップサイズ: 32GB

# シミュレータのインストール
- [自動運転AIチャレンジ 2025 ソースコード一式 GitHub リポジトリ ](https://github.com/AutomotiveAIChallenge/aichallenge-2025)参照
~~~ bash
cd ~ 
git clone https://github.com/AutomotiveAIChallenge/aichallenge-2025.git
~~~
- 以降は、[セットアップ手順](https://automotiveaichallenge.github.io/aichallenge-documentation-2025/setup/workspace-setup.html)に従う


# チートシート
## Dockerコンテナ外の操作コマンド
- AIチャレンジのシミュレータは、Dockerコンテナ内で動作します。
- Dockerコンテナ外での操作は、主にリポジトリのアップデートやDockerコンテナのビルド、起動に関するものです

### リポジトリのアップデート
~~~ bash
cd ~ # Move to home directory
git clone https://github.com/AutomotiveAIChallenge/aichallenge-2025.git # Clone repository
cd aichallenge-2025 # path to aichallenge2025
git pull origin main # Update to latest files
~~~

### Dockerコンテナのビルド
~~~ bash
cd ~/aichallenge-2025
./docker_build.sh dev
docker images
~~~

### Dockerコンテナ起動(CPU版のAWSIM)
~~~ bash
cd ~/aichallenge-2025
./docker_run.sh dev cpu # Launch the Docker container with CPU support
~~~

### Dockerコンテナ起動(Simulator起動なし/セットアップ用)
~~~ bash
cd ~/aichallenge-2025
docker run -it aichallenge-2025-dev-ubuntu:latest
~~~
### 実行中のDockerを保存する
~~~ bash
NAME="aichallenge_new" 
CID=$(docker ps -a -q | head -n1)   # get first running container ID
docker commit "$CID" "${NAME}:latest"   # save as temporary name
docker stop "$CID" # stop the container
~~~

### 最近保存したイメージをAIChallengeに置換
~~~ bash
NAME="aichallenge-2025-dev-ubuntu" 
LatestName=$(docker images --format '{{.Repository}}:{{.Tag}}' | head -n1)
docker tag "$NAME:latest" "$NAME:backup"  # replace tag

docker rmi $NAME:latest # Remove the tag
docker tag "$LatestName" "$NAME:latest"  # replace tag
docker rmi "$LatestName"
~~~

### 実行中のDockerコンテナを強制停止する
~~~bash
CID=$(docker ps -q | head -n1); [ -n "$CID" ] && docker kill "$CID" # Force kill the first running container (if any)
~~~


## Dockerコンテナ内の操作コマンド
- AIチャレンジのシミュレータは、Dockerコンテナ内で動作します。
### Simulatorの起動
~~~ bash
cd /aichallenge
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

### rosboardの起動
~~~
source /opt/ros/humble/setup.bash && \
cd ~/rosboard && ./run
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

# 便利なツールのインストール
## Dockerコンテナ内に追加するツール
- AIチャレンジのシミュレータを使いやすくするために、Dockerコンテナ内にいくつかのツールを追加します。
### ファイルブラウザ Nautilus
~~~ bash
sudo apt update && sudo apt install -y nautilus
~~~
### テキストエディター gedit
~~~ bash
sudo apt update && sudo apt install -y gedit
~~~

## ROSのrosboard
~~~bash
sudo apt update
sudo apt install -y git python3-pip ros-humble-rmw-fastrtps-cpp
sudo pip3 install --user tornado simplejpeg
sudo git clone https://github.com/dheera/rosboard.git ~/rosboard
cd /aichallenge
~~~

### ROSのrqt
~~~ bash
sudo apt update
sudo apt install -y ros-humble-rqt ros-humble-rqt-common-plugins
~~~

### ROSのTurtle Sim
~~~ bash
sudo apt update && sudo apt install -y ros-humble-turtlesim
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


### 不要なDockerイメージを削除する
~~~ bash
docker image prune -f # Remove unused images
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
