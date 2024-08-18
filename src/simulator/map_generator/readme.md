# Map Generator

* install pcl
```bash
sudo apt-get install libpcl-dev
```

* build command
```bash 
colcon build --packages-up-to map_generator --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
```