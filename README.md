***🧮 README: Running the Code***

***📘 Project Overview***
This project combines:
•	PuLP – a Python library for Linear and Integer Optimization
•	Tkinter – Python’s built-in GUI (Graphical User Interface) framework
It allows users to interact with an optimization model through a simple GUI interface.
________________________________________

***⚙️ Prerequisites***
Before running the code, make sure you have:
•	Python 3.8 or higher
•	pip (Python package manager)
To check your Python version, open a terminal or command prompt and type:
python --version
________________________________________

***📦 Installation Steps***
1.	Clone or download the project folder
2.	Install dependencies:
3.	pip install pulp
🔹 Note: Tkinter comes pre-installed with most Python distributions.
If not, install it manually using:
o	Windows: Included with standard Python installer
o	Linux (Ubuntu/Debian):
o	sudo apt-get install python3-tk
o	Mac: Included with system Python
4.	Verify installation:
5.	python -m pulp
If no errors appear, PuLP is installed correctly.
________________________________________

***▶️ Running the Code***
Run the Python file using:
python ILP_for_floorplanning.py
This will:
•	Launch a Tkinter window (GUI)
•	Allow you to input data or parameters
•	Run the PuLP optimization model
•	Display results on the GUI or console

***🪟 GUI Instructions***

***Startup***
•	The interface will start with 4 default modules and a preset chip aspect ratio.
 
***Adding Modules***
1.	Click “Add Module” to add extra modules
2.	Enter:
o	Module Name (e.g., ALU, MEM1)
o	Width and Height of module
o	Orientation Mode:
	Free → Solver decides rotation
	Fixed → Module fixed in orientation
	Rotatable → Module may rotate 90°

***Removing Modules***
•	Select the desired module from the module list.
•	Click “Remove Selected”.

***Setting Chip Aspect Ratio***
•	Use the aspect ratio field to set your desired chip proportion (e.g., 1.5 for 3:2).

***Solving***
•	Click “Solve & Draw” to:
o	Run the ILP-based floorplanning solver
o	Automatically compute the optimal layout
o	Display a floorplan diagram with module placements

***Output***
•	A visualization window will open, showing:
o	Module positions
o	Dimensions
o	Orientation
o	Total chip area and aspect ratio results
