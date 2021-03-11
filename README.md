# Forward Kinematics on a Joint Chain

## Install Instructions

```
git submodule update
pip install -r requirements.txt
python setup.py install
```

## Running the tests

```
git submodule update
pip install -r requirements.txt
python setup.py build_ext --inplace
make -C tests/ tests
```


## Example Usage

```
import math
import chain

joint_chain = chain.JointChain()

j0 = joint_chain.base()
j1 = jc.add_joint([1, 0, 0], [0, -.707, 0, .707])
j2 = jc.add_joint([1, 1, 0], [0, 0, .707, .707])
grip = jc.add_joint([1, 1, 1], [0, 0, .707, .707])

j0.set_angle(math.pi / 2)

print(grip.position)
print(grip.orientation)
```
