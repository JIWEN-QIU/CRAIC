# SUPER

# Clone and build

使用

```bash
git clone git@github.com:RENyunfan/SUPER.git --recursive
```

将内部依赖的子模块`ROG-Map`和`mars_planning_utils`一起clone下来。注意你是否拥有这两个仓库的权限。

然后直接catkin_make就好了。

# Launch

Release里有放一个MARSIM的压缩包，可以下载再来简单测试一下

```bash
roslaunch test_interface test.launch
```

随后用3d nav goal点击即可，注意点击的yaw方向就是飞机的目标yaw

测试launch文件在 [exploration.launch](super_planner/launch/exploration.launch) 

```xml
<launch>
    <rosparam command="load" file="$(find super_planner)/config/exp_advanced_params.yaml"></rosparam>
    <rosparam command="load" file="$(find super_planner)/config/exp.yaml"></rosparam>
    <node pkg="super_planner" name="fsm_node" type="fsm_node" output="screen">
    </node>
</launch>
```

通过两次加载参数，实现了`exp.yaml`覆盖掉`exp_advanced_params.yaml`中重复的内容。用户只需要修改`exp.yaml`中的参数就好。

