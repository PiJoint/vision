# vision

## Install


```bash
git clone git@github.com:PiJoint/vision.git $catkin_ws/pijoint_vision
```

```bash
git clone git@github.com:PiJoint/pitorch.git --depth 1 --branch v1.13.2 && cd pitorch && pip3 install .
```

```bash
cd $catkin_ws/pijoint_vision && pip3 install -r requirements.txt
```

```bash
catkin_make install
```

## Run
```bash
cd $catkin_ws/pijoint_vision && python3 -i src/main.py
```