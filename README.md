Prerequisites

Make sure you have the following installed before starting:

Required

Python 3.10 or higher
https://www.python.org/downloads/

During install, check “Add Python to PATH”

Node.js (LTS)
https://nodejs.org/

(Required for React + Expo)

Visual Studio Code
https://code.visualstudio.com/

USE POWERSHELL AS TERMINAL FOR COMMANDS TO WORK (WINDOWS)
 
Clone the Repository
git clone https://github.com/yourusername/U-parking.git
cd U-parking

Backend Setup (Django)
Create & Activate Virtual Environment

From the project root:

python -m venv venv
.\venv\Scripts\Activate.ps1


If you see an execution policy error, run:

Set-ExecutionPolicy -Scope Process -ExecutionPolicy Bypass
.\venv\Scripts\Activate.ps1

Install Python Dependencies
python -m pip install --upgrade pip
pip install -r requirements.txt


Verify Django installation:

python -m django --version

Run the Django Development Server
cd backend
python manage.py migrate
python manage.py runserver


Open in browser:

http://127.0.0.1:8000/

Web Frontend Setup (React)

From the project root:

cd frontend
npm install
npm start


Web app runs at:

http://localhost:3000/


Fetches data from the Django backend.

Mobile App Setup (React Native + Expo)

From the project root:

cd mobile
npm install
npx expo start


Install Expo Go on your phone

Scan the QR code from the terminal or browser

Phone and computer must be on the same network

 If calling the backend from mobile, use your computer’s local IP, not 127.0.0.1:

http://192.168.x.x:8000/
