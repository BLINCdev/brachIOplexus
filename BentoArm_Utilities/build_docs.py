from os import system

doc_files = ('helper_functions.py', 'inverse_kinematics.py', 'robot.py', 'socket_handler.py')

for file in doc_files:
    system(f"pdoc --html {file} --force")