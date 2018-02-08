# Installing OpenCV

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
