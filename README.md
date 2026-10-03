# br-stm32-proj

STM32H5 + Zenoh-Picoを用いたbr制御用ファームウェア. zenohを使ってUDP通信でros2と統合することができる.
中継スクリプトやブリッジを一切介さず、ros2側からノードおよびトピックとして認識されます。

---

## 前提条件: Nix のインストール (未導入の場合)

> [!WARNING]
> **Ubuntu / Debian の `apt install nix-bin` は絶対に使わないでください！**  
> `apt` 経由の Nix はバージョンが古く、Flakes やマルチユーザーデーモンが正しく構成されないため動作しません。

必ず公式推奨の **Determinate Nix Installer** を使用してください（Flakes が最初から有効化され、トラブルなく一発で入ります）：

```bash
# 1. Nix のインストール (要 sudo 権限)
curl --proto '=https' --tlsv1.2 -sSf -L https://install.determinate.systems/nix | sh -s -- install

# 2. シェルを再起動 (または現在のターミナルでパスを反映)
source /nix/var/nix/profiles/default/etc/profile.d/nix-daemon.sh

# 3. 確認
nix --version
```

---

## クイックスタート

### 1. 環境構築 (初回のみ)
```bash
make setup
```
- Nix 設定 (`nixconf`): Flakes & Cachix バイナリキャッシュの自動登録
- Git サブモジュール初期化 (`zenoh-pico`, `micro-cdr`)
- ST-LINK 用 udev ルール設定
- Python 依存関係インストール (`eclipse-zenoh`, `zenoh-cli`)
- .msg コード生成

### 2. ビルド & 書き込み
```bash
make build    # ファームウェアビルド
make flash    # ST-LINK 経由で STM32 へ書き込み (st-flash / openocd)
```
書き込み後、STM32 本体の黒いリセットボタン（B1）を押すこと.

---

## 3. ROS2統合

```bash
# 1. RMW とルータ接続先を設定 (UDP)
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ZENOH_CONFIG_OVERRIDE='mode="client";connect/endpoints=["udp/192.168.50.30:7447"]'
export ROS_DOMAIN_ID=0

# 2. トピック & ノード確認
ros2 topic list
ros2 node list

# 3. メッセージ受信
ros2 topic echo /chatter
```

フレーム受信tickも必要な場合は、`can_transport_msgs/msg/TimedFrame`（Classic）
または`TimedFDFrame`（FD）の`can/timed_frames`・`canfd/timed_frames`を入力として使用できます。
ingressからの出力topicは通常入力と同じ`can/frames`・`canfd/frames`です。
`rx_monotonic_ns`はSTM32のCAN受信時点の単調時計で、`timestamp_valid`がfalseの間は
ROS時刻ではありません。ROS時刻との同期がない状態でPC到着時刻と混同しないため、
時刻が必要な処理は`timestamp_valid`を必ず確認してください。

IMUを別STM32からCAN-FDで送る場合は、IMU payloadに計測時刻を含めて透過転送します。
PC側の`imu_can_decoder`がCAN ID `0x500`の18 byte frameをdecodeし、`0x510`の同期要求に
対するIMU側の`0x511`応答からPCとのclock offsetを推定します。同期成立前は計測時刻を
ROS時刻としてpublishしません。bridge STM32はこのプロトコルを解釈せず、通常の
`canfd/tx`・`canfd/frames`として転送します。

---

## 4. Zenoh 単体テスト (CLI)

ROS 2 を介さず、Zenoh レベルでパケットを直接モニタする場合：

```bash
# ルータ PC (192.168.50.30) 経由で受信
make sub ARGS="-m client -e udp/192.168.50.30:7447"

# 自 PC をルータとして受信
make sub
```

---

## 5. その他の便利コマンド
```bash
make firewall-off  # PC 側のファイアウォール・パケットフィルタを全開放 (要 sudo)
make router        # スタンドアロン zenohd ルータの起動
make test          # ホスト側の CDR シリアライズ単体テスト実行
make help          # 利用可能コマンド一覧
```


## 注意

### zenohd

zenohルータは一つだけ立てること！！

### list

ros2 topic listで出ない場合は環境変数を疑うか、daemonを再起動すること

```bash
ros2 topic list --no-daemon
```