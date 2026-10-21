# can protocolの生成
.dbcファイルを作成し，cantoolsコマンドを使用

cd protocols
uv venv
source .venv/bin/activate
uv pip install cantools

uv run cantools generate_c_source ../*.dbc

# usb to canを使用したcanデータの確認
python3 can_dbc_logger.py

デフォルト
USB: /dev/ttyUSB0
boudrate 1M bps
