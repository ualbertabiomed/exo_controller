# Exoskeleton - Controller (Raspberry Pi 4) Code

## Summary

The controller directory is a catkin package, meant to be uploaded on the controller for the Exoskeleton.

## Development Instructions

### Github setup

Now in Github you must either use a personal access token (PAT) or a SSH key. The SSH key takes a bit more to set up, but it's really convient once it's working. Follow these three steps to set it up:

1. [Generate](https://docs.github.com/en/authentication/connecting-to-github-with-ssh/generating-a-new-ssh-key-and-adding-it-to-the-ssh-agent) an SSH key
2. Add it to your github account
3. Test the connection

To setup an SSH key, follow [these instructions](

### Setup

1. Create a [catkin workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace) if you don't have one already
2. Navigate to the catkin_ws/src/ directory: `cd catkin_ws/src/`
3. Clone this repository
   - HTTPS: `git clone https://github.com/ualbertabiomed/exo-controller.git`
   - SSH: `git clone git@github.com:ualbertabiomed/exo-controller.git`
4. Navigate back to your catkin_ws root: `cd ..`
5. Build the package: `catkin_make`

### Creating your branch

1. Navigate to the repo directory: `cd catkin_ws/src/exo-controller`
2. Create a new branch: `git branch`*`your_name`*
3. Checkout (switch to) that branch: `git checkout`*`your_name`*

### Making changes
#### Option 1 - GUI (Easier?)
   Use either a GUI such as the VSCode source control tab or Github desktop
#### Option 2 - Command Line
Use `git add <files>`, `git commit`, and `git push`
- For more info on these commands type `man <command name>`, e.g. `man git add` or search online

### Pull requests

After pushing some changes, open a pull request to merge into main and ask someone else to review it. See all pull requests [here](https://github.com/ualbertabiomed/exo-controller/pulls).

## Upload Instructions

### Instructions for setting up a new Pi

#### Method 1: Docker (WIP)
1. Install Ubuntu 20.X
2. Create uab user with password “uabiomed”
3. Name device “uab-piX” where X is whatever number Pi this is
4. Do the following in order:
   1. “sudo apt update”
   2. “sudo apt upgrade”
   3. “sudo apt install curl git make” # Get packages to instal docker and our repo
   4. “curl -sSL https://get.docker.com | sh” # Install docker from website, will take a bit
   5. “sudo usermod -aG docker uab” # Adds uab to docker group
   6. Logout and log back in
   7. “docker run hello-world” # Verify everything worked
5. Setup a github SSH key by following the steps listed here, specifically:
   1. Generating a new SSH key and adding it to the agent
   2. Adding a new SSH key to your github account
   3. Testing your SSH connection

#### Method 2: Native
1. Install Ubuntu 20.X
2. Create uab user with password "uabiomed"
3. Name device "uab-piX" where X is whatever number Pi this is
4. Setup github SSH key (See step 5 above)
5. Pull exoskeleton repo
6. Install ROS as detailed [here](http://wiki.ros.org/noetic/Installation/Ubuntu)
7. Install motor dependencies, what these are is tbd...
8. Run launch file

## Style

Try to follow [this](http://wiki.ros.org/PyStyleGuide) style guide 

Valid Names
Package Resource Names have strict naming rules as they are often used in auto-generated code. For this reason, a ROS package cannot have special characters other than an underscore, and they must start with an alphabetical character. A valid name has the following characteristics:
1. First character is an alpha character ([a-z|A-Z])
2. Subsequent characters can be alphanumeric ([0-9|a-z|A-Z]), underscores (_) or a forward slash (/)
3. There is at most one forward slash ('/'). 
