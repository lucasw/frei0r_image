# frei0r_image
frei0r ROS image integration


```
sudo apt install frei0r-plugins
```

## Custom install

```
git clone git@github.com:lucasw/frei0r.git
mkdir build_frei0r
cd build_frei0r
cmake ../frei0r/ -DCMAKE_INSTALL_PREFIX=$HOME/other/install
make
make install
```
