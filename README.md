# Extra Packages needed:
1- geographic info
```
sudo apt-get install ros-melodic-geographic-info
```
OR
```
sudo apt-get install ros-melodic-geodesy
```

# Python packages needed:
```
pip3 install numpy
pip3 install pyproj (requires pip version >= 19.0)
pip3 install selenium
```
if you have any problems installing pyproj, try updating pip to the latest version.

you can check the pip version by running 
```
pip3 -V
```
# Additional stuff:
running the map node requires something called [geckodriver](https://github.com/mozilla/geckodriver/releases) and requires the use of firefox

to install geckodriver:
### 1- getting the executable
from the above link, get the url of the version you need (in this case linux x64)

then:
```
wget <link>
tar -xvf <downloaded file>
```

### 2- create link in bin directory

```
sudo ln -Pf <path to geckodriver file> /usr/bin/geckodriver
```
