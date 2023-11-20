# master_thesis

Master Thesis Project

- git clone --recursive https://github.com/fegaeze/master_thesis.git
- cd master_thesis
- rosdep update
- source setup.sh
- roslaunch go1_mud_test mud_test.launch simulation:=false (default = true)

- Go to client, npm install

Note: Might get error when running ./setup.sh if you have a different version of behavior tree 
- sudo apt-get remove ros-noetic-behaviortree-cpp-v3

If you want to work on the web interface, follow the instructions below:
- if you dont have nodejs installed:
    - install latest version (from nodesource, apt-get only installs up to v10. minimum of 12 needed)

Shout out to react-ros (flynneva)

if npm run dev does not run, the node version is outdated, 
https://davidwalsh.name/upgrade-nodejs
sudo apt-get install --reinstall nodejs libnode64
node -v
restart terminnal





