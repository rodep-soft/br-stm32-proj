## can protoclの生成
# .dbcファイルを作成し，cantoolsコマンドを使用

mkdir protocols && cd protocols
uv venv
source .venv/bin/activate
uv pip install cantools

uv run cantools generate_c_source ../*.dbc
