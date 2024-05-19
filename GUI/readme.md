# Necessary Libraries
serial
paho.mqtt.client
cv2
tensorflow
google.protobuf.internal
cvzoneHandtrackingModule
time
enum
threading
tkinter
json


# .bat file creation guide
You can follow the instructions here or this youtube video create your own .bat file to run the python program in this directory. https://www.youtube.com/watch?v=-Wl6ZwlICSE

Below is an example of .bat file:

1 @echo off
2 "C:\Users\prism\AppData\Local\Programs\Python\Python310\python.exe" "C:\Users\prism\OneDrive\Documents\2024\Uni\S1\CSSE4011\Project\project_csse4011\project_csse4011\GUI\main.py"
3 pause

Open a blank notepad file and do the following.

Lines 1 and 3 will remain the same in your .bat file. The first element of line 2 is the path to the python executable and your machine. You can get the path by hitting the windows button and searching for IDLE python. Right click on it and open file location. Once you are in file explorer and you see IDLE python, right click on it again and select Open file location. This takes you the directory where python.exe is located. You can now copy this path and paste it in line 2 surrounded by by double quotes. Now on the same line enter a space and then enter the path of the main file (main.py) for this python program. You should end up with something similar to the example above. 

Finally save the file with an appropriate name and change the file extension to ".bat" it should look like "file_name.bat".

Now you can find the file and execute it. The python script will run accordingly.
