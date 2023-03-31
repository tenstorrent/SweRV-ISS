scriptpath=$( cd -P -- "$(dirname -- "$(command -v -- "$1")")" && pwd -P )

if [[ "$1" == "-f" ||  "$1" == "force" ]]; then
  if [ -d $scriptpath/ralph_venv ]; then
    rm -rf ralph_venv
  fi
fi

if [ ! -d $scriptpath/ralph_venv ]; then
  python3 -m venv ralph_venv
  source ralph_venv/bin/activate
  pip install --upgrade pip
  pip install -r requirements.txt
else
  source ralph_venv/bin/activate
fi
