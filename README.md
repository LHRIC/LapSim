# LapSim

## Overview
This project implements a kinematics model that allows simulation, analysis, and visualization using parameters of unsprung hardpoints. 

## Requirements 
- Python 3.9+

### Dependencies
Install required packages with:
```pip install -r requirements.txt```

## Usage
If using the Jupyter notebook: 
1. Clone the repository with: ```git clone <repository name>```
2. Add the desired hardpoints by either changing:
   - The  ```.yaml``` file that is in the parameters folder.
   - If you would like to add your own ```.yaml``` file, ensure that it is in the parameters folder and in the first cell, the ```kin_model_1.from_hardpoints('<filename.yaml>')``` reflects it.
   - If you would like to use an Excel sheet with the hardpoints, ensure that the labels of each hard point follow the convention detailed below. Ensure that the Excel file is in the parameters folder and that you are using the correct function with the corresponding file name ```test_kin_model.from_xlsx('parameters/<filename>.xlsx', steering_delta, front_left_delta, rear_left_delta)```
3. Open the ```kinematics_model.ipynb``` file, or if you want to compare two sets of hardpoints, use ```kinematic_report_comparisons.ipynb```
4. Run cells sequentially to build the kinematic arrays, analyze results, and visualize.

If using the Python script:
1. Do the previous steps 1 and 2. To run the script, open the terminal and type in ```python kinematics_model.py```
