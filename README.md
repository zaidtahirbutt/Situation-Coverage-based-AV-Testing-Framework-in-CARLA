[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

# Situation-Coverage-based-AV-Testing-Framework-in-CALRA

Situation Coverage-based (SitCov) Autonomous Vehicle (AV)-Testing Framework in CALRA. This repository contains the files and the steps needed to run the SitCov AV-Testing Framework in Carla. Our SitCov AV-Testing Framework uses CARLA software and Scenario Runner API. 

The full steps of getting this framework to run are mentioned below. These instructions are for a windows 10 os, please run equivalent Linux commands or anyother os commands.


## 1) Install Anaconda and create a virtual environment in Anaconda software
--------------------------------------------------------------------------

- First creating virtual python env in anaconda and install python using the following command:
  
  **conda create -n carla_sitcov_avtesting pip python=3.7**
  
-	Activate the virtual env to install the subsequent packages inside the virtual env using following command:

     **activate carla_sitcov_avtesting**
     
- Set up two folders in your drive. One for Carla and one for Scenario Runner. 

## 2) Install CARLA 9.10 (This exact version)

- Then go to **Carla 9.10** website here https://carla.readthedocs.io/en/0.9.10/start_quickstart/ and download **Carla 9.10** and follow the guidance on how to run it.

- One of the requirements mentioned on the page is to run this command:

     **pip install --user pygame numpy**
     
- **Test Carla installation** by going to the examples folder in Python API folder inside Carla (using the cd command in the terminal). My examples folder is at the following address in my PC, "D:\Carla9.10\WindowsNoEditor\PythonAPI\examples". Run the spawn_npc.py python script while CarlaUE4.exe is running using this command:

     **python spawn_npc.py**

## 3) Install Scenario Runner 9.10 (This exact version)

- Set up **scenario runner 9.10** for **Carla 9.10** using these links https://github.com/carla-simulator/scenario_runner/tree/0.9.10  and  https://github.com/carla-simulator/scenario_runner/blob/0.9.10/Docs/getting_scenariorunner.md and follow all the instructions provided there to run scenario runner 9.10 with Carla 9.10.

- Go into scenario runner folder and run the following commands (helping tip in Scenario Runner 9.10 installation):

     **pip3 install --user -r requirements.txt**
- To check the **Scenario Runner** installation, run the **CarlaUE4.exe** (**in Carla folder**) and run the following two python scripts in order in the **Scenario Runner folder**:
		
     **python scenario_runner.py --scenario NoSignalJunctionCrossing --reloadWorld**
		
     **python manual_control.py**


License
-------

Situation-Coverage-based-AV-Testing-Framework-in-CARLA specific code is distributed under MIT License.




