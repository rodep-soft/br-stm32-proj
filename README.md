# br-stm32-proj

STM32F767ZI (Nucleo-144) + Zenoh-Pico による ROS 2 通信プロジェクト。

## クイックスタート

### 1. 環境構築 (初回のみ)
```bash
make setup
```
- Git サブモジュール初期化 (`zenoh-pico`, `micro-cdr`)
- ST-LINK 用 udev ルール設定
- Python 依存関係インストール (`eclipse-zenoh`, `zenoh-cli`)
- .msg コード生成

### 2. ビルド & 書き込み
```bash
make build    # ファームウェアビルド
make flash    # ST-LINK 経由で STM32 へ書き込み (st-flash / openocd)
```

### 3. Zenoh 受信テスト (PC 側)
ルータ PC (`192.168.50.30`) 経由で STM32 のメッセージを受信する場合：
```bash
make sub ARGS="-m client -e udp/192.168.50.30:7447"
```

ルータ役の PC 単体で待ち受ける場合：
```bash
make sub
```

### 4. ROS 2 との連携

#### 方法 A: ROS 2 中継ノードの利用 (推奨・確実)
Zenoh で受信したメッセージをそのまま標準 ROS 2 トピック `/chatter` へパブリッシュします：
```bash
# ROS 2 ターミナルで実行
python3 tools/zenoh_to_ros2.py -e udp/192.168.50.30:7447

# 別のターミナルで受信確認
ros2 topic list
ros2 topic echo /chatter
```

#### 方法 B: rmw_zenoh_cpp による直接受信
```bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ZENOH_CONFIG_OVERRIDE='mode="client";connect/endpoints=["udp/192.168.50.30:7447"]'
export ROS_DOMAIN_ID=0

ros2 topic echo /chatter std_msgs/msg/String
```

### 5. その他の便利コマンド
```bash
make firewall-off  # PC 側のファイアウォール・パケットフィルタを全開放 (要 sudo)
make router        # スタンドアロン zenohd ルータの起動
make test          # ホスト側の CDR シリアライズ単体テスト実行
make help          # 利用可能コマンド一覧
```