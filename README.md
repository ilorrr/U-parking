# U-parking
Make sure you have the following installed before starting:

Python 3.10 or higher
https://www.python.org/downloads/

During install, check “Add Python to PATH”

Visual Studio Code
https://code.visualstudio.com/

USE POWERSHELL AS TERMINAL FOR COMMANDS TO WORK CHAMPIONS

From the project root:

python -m venv venv

Activate the Virtual Environment
.\venv\Scripts\Activate.ps1

If you see an execution policy error, run:

Set-ExecutionPolicy -Scope Process -ExecutionPolicy Bypass
.\venv\Scripts\Activate.ps1


Install Django
python -m pip install --upgrade pip
python -m pip install django


Verify installation:

python -m django --version

Create the Django Project

From the project root:

django-admin startproject backend

Run the Django Development Server

cd backend
python manage.py runserver

Open in Browser

Navigate to:

http://127.0.0.1:8000/
