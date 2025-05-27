# AMANEQ Streaming Low-Resolution Tdc (Str-HRTDC) 10G
Streaming low-resolution TDC with 10GbE

[User guide](https://spadi-alliance.github.io/ug-amaneq/)

## Vivado version
2023.1.1

# INSTALLATION guide
The procedure to generate the Vivado project. You need to complete the following things once after cloning the project.

## Update submodule
You need to update the submodule after cloning.
```
git submodule update -i
```

## Generate .xpr file
Generate the Vivado project from the TCL script. The Vivado project related directories and files are genereted under build directory.
The files in the build directory are not under Git management. You can remove them and re-generate the project anytime.

**For Windows User:**
Launch the for-windows.bat in CMD or PowerShell with the argument for the path for vivado.

Example

![image](https://github.com/spadi-alliance/AMANEQ-Skeleton/assets/41090607/ba7c1ef0-8c9b-4c0e-9947-22093b7ae244)

**For Linux User:**
Please execute project.tcl by vivado with the batch mode with same arguments written in for-windows.bat.


