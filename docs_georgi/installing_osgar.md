# Important repositories
https://github.com/robotika

https://github.com/robotika/osgar/tree/master

https://github.com/robotika/osgar-apps

# Installing venv on Windows

1. `python -m venv venv`
2. `.\venv\Scripts\activate.bat`
3. `python -m pip install --upgrade pip`
4. `pip install -r requirements.txt`
5. `pip install -e .[tools]`

# Installing osgar and osgar-apps simultaneously

1. `pip install uv` # once, globally
2. `uv venv --python 3.11 .\venv-osgar` # or 3.10 if you hit more cp311 wheel gaps
3. `.\venv-osgar\Scripts\activate`
4. Fix pins across ALL THREE files (osgar/requirements.txt,
   osgar-apps/requirements.txt, osgar-apps/pyproject.toml):
   - numpy -> `numpy>=1.23.5,<2`
   - comment out `#pyrealsense2==2.53.1.4623` (needed only for a specific
     camera, not the Luxonis OAKs)
   - requests -> `requests>=2.32.0`
5. In osgar-apps/requirements.txt, comment out the
   `git+https://github.com/robotika/osgar.git@master#egg=osgar` line
   (and the `[tool.uv.sources] osgar = {...}` entry in pyproject.toml,
   if present) — osgar will be installed separately as an editable dev
   install in step 7, and these would silently overwrite it.
6. `uv pip install -r .\osgar\requirements.txt -r .\osgar-apps\requirements.txt`
   - Single resolver pass across everything except osgar itself
7. `pip install -e .\osgar`
   - Editable install LAST, so it isn't clobbered by anything above
8. (Only if a *different* package still errors on "no matching wheel")
   install MS C++ Build Tools, or drop the venv to Python 3.10.