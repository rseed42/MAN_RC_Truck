Configuration of `/etc/environment`

```
CMAKE_PREFIX_PATH=/opt/ros/kinetic
LD_LIBRARY_PATH=/opt/ros/kinetic/lib
PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:/usr/local/games:/usr/games:/opt/ros/kinetic/bin
ROS_DISTRO=kinetic
ROS_ETC_DIR=/opt/ros/kinetic/etc/ros
ROS_MASTER_URI=http://localhost:11311
ROS_PACKAGE_PATH=/opt/ros/kinetic/share
ROS_ROOT=/opt/ros/kinetic/share/ros
PYTHONPATH=/opt/ros/kinetic/lib/python2.7/dist-packages
```

Move all start scripts to `/usr/sbin`

Put the systemd files in: `/lib/systemd/system/name.service`. The source is to be found in `./systemd`.

Install and enable the services

```
systemctl enable roscore.service
```
