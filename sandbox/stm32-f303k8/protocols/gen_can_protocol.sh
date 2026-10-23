uv venv
source .venv/bin/activate
uv pip install cantools

uv run cantools generate_c_source ../*.dbc

rm -rf .venv


