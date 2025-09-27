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
1. To download VS Code:
   - Download it following the instructions here: https://code.visualstudio.com/download
   - Once downloaded, open VS Code and on the slide bar on the left, click the logo with 4 squares (that is extensions). Download the Jupyter extension (ensure it is only ```Jupyter``` and not Jupyter Slide Show or anything else). 
2. To clone the repository:
   - If you have Git installed on your computer, copy into the terminal or VS code using the command: ```git clone https://github.com/LHRIC/LapSim.git```
   - If you do not have Git, click the green ```Code``` button in the upper right-hand corner of the LAPSIM HOMEPAGE ON THE GITHUB WEBSITE. There should be a drop-down, click ```download ZIP```. Once downloaded, unzip the folder and place it somewhere accessible, such as ```Desktop```.
3. To install the Python environment:
    - Open up your terminal (Open a terminal by clicking the terminal button on the top bar, there should be a drop-down, then click on new terminal) and type in ```which python``` to check if you have Python already installed.
      - If a directory such as ```/opt/anaconda3/bin/python``` pops up, then Python is already installed. Then, check the version by typing in the terminal ```python --version``` or ```python3 --version.```
      - If no directory pops up, the easiest way to set up Python is using MiniConda (it is free and pretty lightweight). (MiniConda download and setup instructions are at the bottom.)
4. Add the desired hardpoints by either changing:
   - The  ```.yaml``` file that is in the parameters folder.
   - If you would like to add your own ```.yaml``` file, ensure that it is in the parameters folder and in the first cell, the ```kin_model_1.from_hardpoints('<filename.yaml>')``` reflects it.
   - If you would like to use an Excel sheet with the hardpoints, ensure that the labels of each hard point follow the convention detailed below. Ensure that the Excel file is in the parameters folder and that you are using the correct function with the corresponding file name ```test_kin_model.from_xlsx('parameters/<filename>.xlsx', steering_delta, front_left_delta, rear_left_delta)```
5. Open the ```kinematics_model.ipynb``` file, or if you want to compare two sets of hardpoints, use ```kinematic_report_comparisons.ipynb```
6. Run cells sequentially to build the kinematic arrays, analyze results, and visualize. NOTE: If you change something like the hard points file or Excel sheet, you will need to rerun all of the cells (especially the FIRST cell). THIS IS VERY IMPORTANT, OR THE CODE WILL NOT PROCESS THE NEW DATA.  

If using the Python script:
1. Do the previous steps 1-4. To run the script, open the terminal and type in ```python kinematics_model.py```

Install MiniConda (Follow the instructions on the website): https://www.anaconda.com/docs/getting-started/miniconda/install

Common Mistakes and How to Debug: 
- Python version is too old, update by 
- Copying and pasting the error into ChatGPT is very helpful.

- update requirements.txt
