# br-stm32-proj

STM32F767ZI (Nucleo-144) + Zenoh-Picoを用いたbr制御用ファームウェア. zenohを使ってUDP通信でros2と統合することができる.
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

## 3. ROS2統合 (ロボット内静的 IP 構成)

本機（STM32）はロボット実機での高信頼性・即時起動のため、**静的 IP（192.168.50.10）** で動作します。
ルータ不要で、PC との LAN ケーブル直結やスイッチングハブ接続で即通信可能です。

### PC 側のネットワーク設定例:
- **IP アドレス**: `192.168.50.2` (または `192.168.50.30`)
- **ネットマスク**: `255.255.255.0`
- **ゲートウェイ**: `192.168.50.1`

```bash
# 1. RMW とルータ接続先を設定 (UDP)
# ※ PC の IP が 192.168.50.30 の場合
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ZENOH_CONFIG_OVERRIDE='mode="client";connect/endpoints=["udp/192.168.50.30:7447"]'
export ROS_DOMAIN_ID=0

# 2. トピック & ノード確認 (STM32 起動後、即座に検出されます)
ros2 topic list
ros2 node list

# 3. メッセージ受信
ros2 topic echo /chatter

# 4. CAN フレーム送信テスト (STM32 の CAN バスへ送出)
ros2 topic pub /can_msgs/frame can_msgs/msg/Frame "{id: 291, dlc: 8, data: [1,2,3,4,5,6,7,8]}"
```

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