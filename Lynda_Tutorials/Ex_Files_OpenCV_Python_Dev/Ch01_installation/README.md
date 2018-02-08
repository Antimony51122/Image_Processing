## Installing OpenCV

Firstly, make sure your system and programs are up to date:

```bash
sudo apt-get update
```
then:

```bash
sudo apt-get upgrade
```

Next, we are going to install some of the prerequistes for the packages:

```bash
sudo apt-get install build-essential cmake git pkg-config
```
> - `cmake` for compiling
> - `git` for downloading packages and some additional packaging configurations. 

After that, we are going to install some additional libraries that are specifically for reading image formats:

```bash
sudo apt-get install libjpeg8-dev libtiff5 libjasper-dev libpng12-dev
```
> - `libjpeg8-dev` for jpeg reading
> - `libtiff5-dev` for tiff format reading
> - `libjasper`for additional reading
> - `libpng12-dev` for png reading

> mind that if you have typo, the terminal will tell you that's an invalid package

Next, we are going install a couple of code relating to video format:

```bash
sudo apt-get install libavcodec-dev libavformat-dev libswscale-dev libv4l-dev
```

Next, we are going to install a library that allows us to use OpenCV's user interface features: 

```bash
sudo apt-get install libgtk2.0-dev
```

And next, we are going to add some modules that will allow use to optimise OpenCV commands:

```bash
sudo apt-get install libatlas-base-dev gfortran
```
At this point, we have all the prerequisites we need.

We can check our python version by:

```bash
python3 --version
```

After this, we are going to use pip for installing packages, if you don't currently have pip, simply type:

```bash
wget http://bootstrap.pypa.io/get-pip.py
```
once downloaded, retrieve:

```bash
sudo python3 get-pip.py
```

Now we are going to use install one of the key libraries we are going to using in our image processing with OpenCV which is called the NumPy library:

```bash
sudo pip install numpy
```
If we we want to validate that this installed correctly, we can type:

```bash
python3
```
to enter the Python environment, then use:

```bash
import numpy
```
If it worked, there'll be no additional printouts, and we can use:

```bash
exit()
```
to exit out of the Python3 environment. 

At this point, we have installed all the prerequisites we need before we start compiling OpenCV itself. 

## Compiling OpenCV


