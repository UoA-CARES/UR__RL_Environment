# 
<h1 align="center">UR-5 Ball in Cup Environment</h1>


<div align="center">
  This repository contains the code to use RL on a UR5 robot with the traditional ball-in-cup game. The necessary package and basic instructions are presented.
  Before running this environment, ensure that the following requirements are met:

</div>
## 1. Clone and Install Python URX Package

- Clone the Python URX repository from GitHub:
  ```
  git clone https://github.com/UoA-CARES/python-urx.git
  ```
- Navigate into the cloned repository directory:
  ```
  cd python-urx
  ```
- Install the Python URX package in editable mode using pip:
  ```
  pip install -e .
  ```

## 2. Install Math3d Package

Install the `math3d` package version 3.4.1:

```
pip install math3d==3.4.1
```

## 3. Test the Environment

Run the provided `general_test_example.py` example to ensure everything is set up correctly:

```
python general_test.py
```

---


It is recommended to create a Conda environment with Python 3.11. 

