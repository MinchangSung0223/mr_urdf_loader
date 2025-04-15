# "mr_urdf_loader" Python Package Instructions #

## Dependency Requirement

`numpy` , `modern_robotics` , `urdfpy` should be preinstalled.

## Installing the Package ##
```bash
  pip install mr_urdf_loader
```

or

```bash
  git clone https://github.com/tjdalsckd/mr_urdf_loader.git
  cd mr_urdf_loader
  pip install .
```



## Importing the Package ##

To import the package, we recommend using

```
from mr_urdf_loader import loadURDF
urdf_name = "./test.urdf"
MR=loadURDF(urdf_name)
M  = MR["M"]
Slist  = MR["Slist"]
Mlist  = MR["Mlist"]
Glist  = MR["Glist"]
Blist  = MR["Blist"]

```


## Examples ##
### 1. Simple MR test
```bash
cd tests/example/3DoF
python3 urdf_loader.py
```
![image](https://user-images.githubusercontent.com/53217819/202921164-f450da46-58bd-4335-a0b7-018957b851b0.png)


### 2. Pybullet Simulation
```bash
cd tests/example/3DoF
pip install pybullet
python3 sim.py
```
![image](https://user-images.githubusercontent.com/53217819/202921126-a5c297fb-fd0f-4ef4-91fe-4e0b7821c516.png)


### 3. UR5 Simulation
```bash
cd tests/example/ur5
python3 ur5_sim.py
```
![image](https://user-images.githubusercontent.com/53217819/202973442-54be472e-c43e-4569-981f-bc87bf00b678.png)
## Using the Package Locally ##

It is possible to use the package locally without installation. Download and
place the package in the working directory. Note that since the package is 
not installed, you need to move the package if the working directory is
changed. Importing is still required before using.


### 4. Pybullet Issue
Pybullet의 경우 마지막 링크 urdf에 inertial이 기입되지 않은 경우 동역학 계산이 달라진다.
```
<link name="eef_link" />

```
따라서 다음처럼 mass가 없는 링크에도 inertial 정보를 기입해야 Modern robotics library와 동일하게 계산된다.

```
<link name="eef_link">
  <collision>
      <geometry>
        <box size="0.00 0.00 0.00"/>
      </geometry>
      <origin rpy="0 0 0" xyz="0.00 0 0"/>
    </collision>
    <inertial>
      <mass value="0.0"/>
      <origin rpy="0 0 0" xyz="0.0 0.0 0.0"/>
      <inertia ixx="0.0" ixy="0.0" ixz="0.0" iyy="0.0" iyz="0.0" izz="0.0"/>
    </inertial>
</link>

```
