# balancing_robot

## Hardware Files
All the non GRiSP related files as well as the prototyping files are in this directory : https://github.com/7dric/Opensource_self_balancing_robot

## Disclaimer

This is what worked for us, no assurance it will work for anybody with other packages and dependencies version. Don't hesitate to follow the tutorials on the [grisp wiki page](https://github.com/grisp/grisp/wiki) and to join the GRiSP slack if you have any question.

## Requirements (not necessary if you did the tutorials)

The deployment of the software on the GRiSP2 board is a bit of a headache. This is a step by step instructions that worked for us, using Ubuntu 24.04.

First, be sure you have the latest version of all the basic packages:

```bash
sudo apt-get update
sudo apt-get upgrade
sudo apt install -y libncurses5-dev libncursesw5-dev curl build-essential libssl-dev
```

### Installing Kerl

Kerl is used to build and install Erlang/OTP instances.

First we need to download kerl in the /usr/bin folder and give it write permission. :

```bash
cd /usr/local/bin
sudo curl -O https://raw.githubusercontent.com/kerl/kerl/master/kerl
sudo chmod a+x ./kerl
```

Then we need to add it to our path: 
```bash
nano ~/.bashrc
```

Add the following line
```bash
#KERL
export PATH="$PATH:/usr/local/bin/kerl"
```

Save and exit the file and then refresh the path :

```bash
source ~/.bashrc
```

Ensure that kerl is well installed and up to date with 

```bash
kerl version
kerl upgrade
```


### Install Erlang/OTP

Erlang is the main tool to use in this project, we will here use kerl to build it. First find the latest release with :

```bash
kerl update releases
kerl list releases all
```

Then use the following command to build a release (changing ${RELEASE_NUM} by the latest supported release number (27.2.2) for me):

```bash
kerl build ${RELEASE_NUM} ${RELEASE_NUM}
```

Then install the build :

```bash
kerl install ${RELEASE_NUM} /usr/local/lib/erlang/${RELEASE_NUM}
```

If you don't have yet the permission to write in `/usr/local/lib` use :

```bash
sudo chown -R $USER:$USER /usr/local/lib
```

You must then activate the erlang install :

Either for the current session :
```bash
. /usr/local/lib/erlang/${RELEASE_NUM}/activate
```

Or for all time :

```bash
. /usr/local/lib/erlang/${RELEASE_NUM}/activate
nano ~/.bashrc
```

And add 
```bash
#ERL
export PATH="$PATH:/usr/local/lib/erlang/27.2.2/bin"
```

Then update path with:

```bash
source ~/.bashrc
```

### Install rebar3

Download the rebar3 e-script from [here](https://www.rebar3.org/)

Then, install using :

```bash
cd ~/Downloads
chmod a+x ./rebar3
./rebar3 local install
```

Then add it to your path:

```bash
nano ~/.bashrc
```

And add at the end :

```bash
#REBAR
export PATH="export PATH=/home/${username}/.cache/rebar3/bin:$PATH"
```

Then update rebar3 using :

```bash
rebar3 local upgrade
```

Then, we need to install globally the plugins required for grisp devellopment with rebar3.

create a file in `~/.config/rebar3/rebar.config`. Fill it with :

```erlang
{plugins, [
    rebar3_hex,
    rebar3_grisp
]}.
```

Then update your plugins:

```bash
rebar3 update
rebar3 plugins upgrade rebar3_grisp
```

At this point you should be able to run the demo app in the grisp tutorials.

## Requirements for the balancing robot

For the Lilygo code deployment, see [this](https://github.com/Nicodaube/Robot/blob/main/lilygo_robot_code/lilyGO_tuto.md) other repository.

Now, we will install the GRISP toolchain. Due to an issue when trying to build the toolchain with the docker image, we will use a prebuilt image of the toolchain, available [here](https://github.com/grisp/grisp/wiki/Building-the-VM-from-source#use).

In the rebar.config, you can then modify the toolchain path like so : ![alt text](./image_md/toolchain.png)

Now, modify all the rebar.config file to suit your path and your OTP version. 

## Deploying the software to an SD card

go to the balancing_robot directory and run the following command :

```bash
rebar3 grisp build
rebar3 grisp deploy
```