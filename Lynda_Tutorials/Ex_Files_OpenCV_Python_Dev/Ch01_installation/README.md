## Installing OpenCV

Firstly, make sure your system and programs are up to date:

```bash
$ sudo apt-get update
```
then:

```bash
$ sudo apt-get upgrade
```

Next, we are going to install some of the prerequistes for the packages:

```bash
$ sudo apt-get install build-essential cmake git pkg-config
```
> - `cmake` for compiling
> - `git` for downloading packages and some additional packaging configurations. 

After that, we are going to install some additional libraries that are specifically for reading image formats:

```bash
$ sudo apt-get install libjpeg8-dev libtiff5 libjasper-dev libpng12-dev
```
> - `libjpeg8-dev` for jpeg reading
> - `libtiff5-dev` for tiff format reading
> - `libjasper`for additional reading
> - `libpng12-dev` for png reading

> mind that if you have typo, the terminal will tell you that's an invalid package

Next, we are going install a couple of code relating to video format:

```bash
$ sudo apt-get install libavcodec-dev libavformat-dev libswscale-dev libv4l-dev
```

Next, we are going to install a library that allows us to use OpenCV's user interface features: 

```bash
$ sudo apt-get install libgtk2.0-dev
```

And next, we are going to add some modules that will allow use to optimise OpenCV commands:

```bash
$ sudo apt-get install libatlas-base-dev gfortran
```
At this point, we have all the prerequisites we need.

We can check our python version by:

```bash
$ python3 --version
```

After this, we are going to use pip for installing packages, if you don't currently have pip, simply type:

```bash
$ wget http://bootstrap.pypa.io/get-pip.py
```
once downloaded, retrieve:

```bash
$ sudo python3 get-pip.py
```

Now we are going to use install one of the key libraries we are going to using in our image processing with OpenCV which is called the NumPy library:

```bash
$ sudo pip install numpy
```
If we we want to validate that this installed correctly, we can type:

```bash
$ python3
```
to enter the Python environment, then use:

```bash
$ import numpy
```
If it worked, there'll be no additional printouts, and we can use:

```bash
$ exit()
```
to exit out of the Python3 environment. 

At this point, we have installed all the prerequisites we need before we start compiling OpenCV itself. 

## Compiling OpenCV

Having installed all the required dependencies for OpenCV3, we can now start to compile from source. We are gonig to first download and clone two git repositories. The first one will be an additional contribution repository for OpenCV and then we will download the OpenCV repository itself. For this we will use:

```bash
$ git clone https://github.com/Itseez/opencv_contrib.git
```
Once the git clone has completed, we can now navigate into the directory that was created:

```bash
$ cd opensv_contrib
```
and now we are going to specifically check out the version of OpenCV we want to install, in this case 3.2.0 will be used:

```bash
$ git checkout 3.2.0
```
> **Note** that it says we aer in a detached HEAD state, meaing we aer looking at the 3.2 release. If we pull from matser, we may be compiling against not fully tested or complete features.

Next we are going to navigate back one level `cd ../` and download our next repository, which is the main repository for OpenCV:

```bash
$ git clone https://github.com/Itseez/opencv.git
```
and this will commence the download of the OpenCV libraries itself. 

Once the download has completed, we now navigate into the OpenCV folder `cd opencv`. Once again, we check out verison 3.2.0

```bash
$ git checkout 3.2.0
```
At this point, we are going to create the directory `build`:

```bash
$ mkdir build
```
which allow us to actually compile the build files into a single location.

> ***tip:** you can clean the screen using `ctrl` + `L`*

The below is referring to [https://github.com/opencv/opencv_contrib](https://github.com/opencv/opencv_contrib) 

Navigate in to `build`: `cd build` and follow the cmake command, which will be the precursor to the actual make command to install the CV library.

```bash
$ cmake -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=/usr/local -D INSTALL_C_EXAMPLES=ON -D INSTALL_PYTHON_EXAMPLES=ON -D OPENCV_EXTRA_MODULES_PATH=~/opencv_contrib/modules -D BUILD_EXAMPLES=ON ..
```
> - `OPENCV_EXTRA_MODULES_PATH` points to the directory of the first git we had, the contribution repository.
> - When it has finished verifying and downloading additional dependencies, we should note some of the last infomation it prints out, such as the Python 3 interpreter that it found. This is how you can be assured it has found the correct Python install

Then, run:

```bash
$ make -j8
```
> The number here should equal the number of cores on your machine

After all above, we start officially install OpenCV using:

```bash
$ sudo make install
$ sudo ldconfig
```
Eventually, check to make sure file was created in the appropreiate location, take a look:

```bash
$ ls /usr/local/lib/python3.5/dist-packages/
```

## Testing the Install

Firstly, we enter the python environment by:

```bash
$ Python3
```
followed with:

```bash
>>> import numpy
>>> numpy.__version__
```
to check which version of numpy we have installed

Then we check the version of opencv by:

```bash
>>> import cv2
>>> cv2.__version__
```
which returns the current version of opencv we have installed.

**Note**: If you encounter the error of :

```
ImportError: No module named cv2
```
you can fix this by reinstalling opencv using anaconda:

```bash
$ conda install opencv 
```


