# mp_host_setup

Host computer Docker setup for interfacing with a Mini Pupper over a network or just running simulations locally.

## Setup

First, install Docker from either the command line or from the Docker website.

```bash
~$ sudo apt install docker
```

Next, install `docker-compose`. Instead of using the version located in the Ubuntu repositories (it's old), we will install it straight from their GitHub. The current version is `2.20.3`, but you can change it to newer versions depending on when they release one.

```bash
~$ mkdir -p ~/.docker/cli-plugins/
~$ curl -SL https://github.com/docker/compose/releases/download/v2.20.3/docker-compose-linux-x86_64 -o ~/.docker/cli-plugins/docker-compose
```

We will then have to make the plugin executable.

```bash
~$ chmod +x ~/.docker/cli-plugins/docker-compose
```

Nvidia card owners may want to install `nvidia-docker` to help with performance of visualization and simulation tools such as RViz and Gazebo.

```bash
~$ sudo apt install nvidia-docker
```

Finally, clone the repository into a desired folder and then change directory (`cd`) to it.

```bash
~$ git clone https://github.com/zmk5/mp_host_setup.git
~$ cd mp_host_setup
```


## Building and Running the Docker Images

To build the images, run the following command in the `mini_pupper_host` directory:

```bash
~$ docker compose build
```

### Running a Mini Pupper Simulation

Once the images are built, we can run a simulation with RViz using the following command:

```bash
~$ docker compose up simulation
```

This will bring up a Gazebo simulation along with RViz.

### Controlling a Mini Pupper with Teleop Node

With the simulation up, we can control the pupper using the teleop node. Instead of using `docker compose up` we will run `docker compose run` instead. 

**NOTE**: The `up` command simply creates and starts the container service in a non-interactive mode. This isn't a big deal for GUI services since you can interact with the GUI that pops up and it will work. However, this is an issue when trying to run a ROS node that takes in user input from a keyboard in the CLI. The `run` command runs a one-off command of our teleop service that we can interact with and move the pupper.

```bash
~$ docker compose up keyboard
```
