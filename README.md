# "mr_urdf_loader" Python Package Instructions #

## Dependency Requirement

`numpy` , `modern_robotics` , `urdfpy` should be preinstalled.

## Installing the Package ##

### Recommended Method ###

Use [pip](https://en.wikipedia.org/wiki/Pip_(package_manager)) to install by
running

```
pip install numpy
pip install modern_robotics
pip install urdfpy

pip install .
``` 

If pip is not preinstalled, check 
[here](https://pip.pypa.io/en/stable/installing/) for help installing pip. 

## Importing the Package ##

To import the package, we recommend using

```
from mr_urdf_loader import loadURDF
MRSetup = loadURDF(urdf_name)
```
## Examples ##
### 1. Simple MR test
```bash
cd example/3DoF
python3 urdf_loader.py
```
![image](https://user-images.githubusercontent.com/53217819/202921164-f450da46-58bd-4335-a0b7-018957b851b0.png)


### 2. Pybullet Simulation
```bash
cd example/3DoF
pip install pybullet
python3 sim.py
```
![image](https://user-images.githubusercontent.com/53217819/202921126-a5c297fb-fd0f-4ef4-91fe-4e0b7821c516.png)


### 3. UR5 Simulation
```bash
cd example/ur5
python3 ur5_sim.py
```
![image](https://user-images.githubusercontent.com/53217819/202973442-54be472e-c43e-4569-981f-bc87bf00b678.png)
## Using the Package Locally ##

It is possible to use the package locally without installation. Download and
place the package in the working directory. Note that since the package is 
not installed, you need to move the package if the working directory is
changed. Importing is still required before using.


### 4. Pybullet Error
Pybullet의 경우 마지막 링크 urdf를 다음과 같이 inertial 정보를 기입하지 않는 경우 임의의 값으로 수정하여 Dynamics를 해석하는 듯 싶다.
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
