# ROS的rosrun&roslaunch、rosparam联合使用
<br>
参考：https://blog.csdn.net/qq_35635374/article/details/121243673
<br>

## 一、rosrun&roslaunch工具

### 1.ROS功能包的介绍


ros的功能包是最小的目录单位，功能包和功能包之间不想相互嵌套【防盗标记–盒子君hzj】

### 2.功能包自动检索


【命令行启动】 在catkin_make和source之后，使用roslaunch命令+功能包+launch名字快速启动，不需要路径信息，补全可能反应比较慢

【launch文件启动】 在catkin_make和source之后，使用launch文件的include标签可以启动其他模块的launch ,$(find 功能包名)/文件下功能包的相对路径/XXX.launch

### 3.rosrun&roslaunch的使用场景


Rosrun和roslaunch的命令行相似；rosrun/roslaunch 功能包+节点名字

在ROS中可以用rosrun运行一个节点，又可以用roslaunch运行一堆节点

rosrun一般用来启动单个.py文件节点，py文件不用编译直接可以理解成节点名字

roslaunch一般用来启动多个C++ 的一堆节点

rosrun是启动一个节点，roslaunch是用launch文件启动一堆节点，只要catkin make编译出节点就可以用rosrun去启动单一节点，roslaunch相当于是rosrun的集合 【防盗标记–盒子君hzj】 . . .

## 二、launch文件的功能


launch 文件是一个 XML 格式的文件，可以启动本地和远程的多个节点，还可以在参数服务器中设置参数，启动其他launch文件等（自启动ROS Master）【防盗标记–盒子君hzj】

## 三、2、launch文件的<>标签类型


根元素采用标签定义，所有的东西都要在该标签下

### 1.嵌套标签【用于启动其他模块的launch文件】


```bash
1、功能：当系统中有很多个launch文件的时候，可以相互包含，类似于C语言中的头文件
2、格式：<include file="$(功能包名)/.../launch文件在功能包相对应的路径/XXX.launch"/>
```


### 2.变量标签


```bash
launch文件内部的局部变量，仅限于launch文件使用，定义局部变量，类似于宏【防盗标记–盒子君hzj】

（1）定义格式
<arg name="arg-name" default="arg-value"/>

（2）调用格式
$(arg xxx）
<param name="foo" value="$(arg arg-value"/>
<node name="node" pkg="package"type="type" args="$(arg arg-name)"/>

注意
变量可以是参数、也可以是功能包代号
```


### 3.标签


```bash
（1）功能
	设置ROS系统运行中的参数，储存在参数服务器中

（2）<rosparam>设置多个参数（.yaml文件）
	介绍：使用在node文件的外部变量
	格式：<rosparam file="param.yaml"command="load"ns="params"/>
	rosparam file:参数名文件及路径
	command：操作指令
	ns：命名空间属性
【防盗标记–盒子君hzj】

（3）<param>设置单个参数
	【尽量写在yaml文件，好修改】
	格式：<param name="aaa"value="bbb">
	name:参数名
	value：参数值
```


### 4.是启动一个节点标签


```bash
格式：<node pkg="package_name" type="executable_name" name="node_name"/>
pkg:节点所在功能包的名称
type:节点的可执行文件名称
name:节点运行时的名称
其他参数
output:运行时吧日志输出到屏幕上或者存起来
respawn:节点失效的时候重新启动
required：应答数据
ns：命名空间属性
args：节点的输入参数（若是启动rviz，这就是.rviz的路径和文件名）
```


【防盗标记–盒子君hzj】

### 5.重映射标签


```bash
（1）功能：重映射ROS中的计算图资源的命名（类似于重命名）
（2）格式：<remap from="/a/b/c(原话题名)">to="/A/B/C(现在重新命名的话题)"
（3）from:原话题名
（4）to:重映射现在的话题名
（5）弊端：使用重映射后调用的环境变量可能找不到了
```


## 四、launch文件的启动方式


### 1.命令行的形式


在launch文件夹开一个终端【roslaunch 文件名.launch】 【launch文件是存放相对自由的，可在功能包内，也可以在功能包外】

Launch文件最好放在一个功能包里面，用命令行容易寻找得到，同时编程的时候工程也不用这么乱。总的launch文件搞一个robot_launch功能包，子功能包内放一个launch文件夹来存放 【防盗标记–盒子君hzj】

### 2.launch文件包含的形式


. .

## 五、设备开机自启动launch


```bash
1.将多个要启动的ros node写入同一个launch文件
2.将要启动的roslaunch命令加入~/.bashrc文件中
3.实现系统开机自动登录功能
4.sudo reboot就ok了【防盗标记–盒子君hzj】
```


注意 roslaunch 命令执行launch文件时，首先会判断是否启动了 roscore,如果启动了，则不再启动，否则，会自动调用 roscore

## 六、rosparam参数服务器的原理


（1）主要思想

使用集中参数服务器( parameter server )维护一个变量集的值，【防盗标记–盒子君hzj】节点可以主动查询其感兴趣的参数的值。留出方便的参数调试接口

（2）主要原理 在ROS Master中就存在一个参数服务器Parameter Server，它是一个全局字典，用来保存各个节点的配置参数，每个节点都是可以访问的，并且会返回需要得到的值。无论是在哪里设置的，只要是参数就会被放到这个全局字典里面，参数服务器是可通过网络访问的共享的多变量字典。它是节点存储参数的地方、用于配置参数、全局共享参数。 【防盗标记–盒子君hzj】 . .

## 七、rosparam参数服务器的功能【列举、设置、获取、 删除、加载文件、写入文件】


通过rosparam -h命令

```bash
Commands:
    rosparam list       列出参数服务器中的参数
    rosparam set       设置参数
    rosparam get	     获取参数值
    rosparam delete  删除参数【防盗标记–盒子君hzj】
    rosparam load     从文件中加载参数到参数服务器【文件类型仅限是.yaml文件】
    rosparam dump  将参数服务器中的参数写入到文件【文件类型仅限是.yaml文件】
```


. .

## 八、rosparam参数服务器的使用方法【命令行、launch、源码编程】


### （1）方法一：命令行接口【rosrun&&rosparam】


（1）通过rosrun命令行设置参数

```bash
（1）语法
	rosrun 包名 节点名称 _参数名:=参数值

（2）示例
	rosrun turtlesim turtlesim_node _A:=100
```


（2）通过rosparam命令行设置参数

```bash
（1）终端加载文件
		rosparam load   从文件加载参数【防盗标记–盒子君hzj】
		$ rosparam load param.yaml

（2）终端查看参数服务器内的参数名
		$ rosparam list
		rosparam list   列举参数名称


（3）终端查看参数名对应的内容
		$ rosparam get 参数名
		rosparam get +参数类型/参数名【查参数内容和格式】

（4）终端设置参数值
		$rosparam set [参数名] [参数值]
		rosparam set    设置参数

（5）rosparam delete 删除参数
```


.

### （2）方法二：.launch接口【&&】


```bash
（1）优点
	１、可以不查看源程序就可以知道节点用到的参数以及给定的初始值；
	２、要修改参数的初始值，可以将它保存到launch文件而不必修改和重新编译源程序

（2）使用步骤
	1）使用<param>标签直接定义参数【针对单个参数】【防盗标记–盒子君hzj】
	<param name="publish_frequency" type="double" value="10.0" />
	<rosparam command="delete" param="my/param" />

	2）使用标签<rosparam>标签配置参数【针对.yaml文件】
	<rosparam file="$(find turtlebot_navigation)/param/global_costmap_params.yaml" command="load" />

（3）示例
	<launch>
	    <param name="p1" value="100" />
	    <node pkg="turtlesim" type="turtlesim_node" name="t1">
	        <param name="p2" value="100" />
	    </node>
	</launch>


（4）<rosparam>标签与<param>标签对比
		<rosparam>标签用于加载文件
		<param>标签用于加载单个参数
		<param>是用来设置单个参数的，而<rosparam>是用来加载参数的，加载的是yaml文件里面配置的参数
		<rosparam file="$(find dashgo_nav)/config/params.yaml" command="load" ns="global_costmap" />
		第一个属性是yaml文件的路径，第二个是选择为加载，第三个是他的命名空间

注意
1）rosparam参数服务器既可以定义全局变量参数，又可以定义局部变量参数
2）标签的type形参不全会自动适配类型，若给定type类型就会强制转换【防盗标记–盒子君hzj】
```


### （3）方法三：源码编程接口【有句柄的方式（较好）、无句柄的方式】


roscpp的参数API有两套：一套在ros::param名字空间下，另一套封装在句柄handle中

（0）创建句柄的方式

```bash
	ros::NodeHandle n;
```


（1）为参数设置默认值

```bash
	用参数名为<param_name>、或值为<default_value>的参数初始化类型的变量variable，其中<param_name>一般在.yaml和launch文件中定义【优先级较高】，<default_value>是一个默认值会被<param_name>覆盖的【优先级较低】
	
	1）有句柄的方式（较好）
	nh.param<<variable_type>>("<param_name>", variable, <default_value>);
	
	2）无句柄的方式
	ros::param::param<<variable_type>>("<param_name>", variable, <default_value>);
	【这个更好一点】
```


（2）为参数设置动态值

```bash
将参数<param_name>赋值为<param_value>

1）有句柄的方式（较好）
ros::NodeHandle::setParam("<param_name>"，<param_value>)
	（1）原理
	设置参数时，首先需要创建 NodeHandle 对象，然后调用该对象的 setParam 函数，【防盗标记–盒子君hzj】
	该函数参数1为参数名，参数2为要设置的参数值，如果参数名以 / 开头，那么就是全局参数，
	如果参数名不以 / 开头，那么，该参数是相对参数还是私有参数与NodeHandle 对象有关，
	如果NodeHandle 对象创建时如果是调用的默认的无参构造，那么该参数是相对参数
	
	（2）示例
	ros::NodeHandle nh;
	nh.setParam("/nh_A",100); //全局,和命名空间以及节点名称无关
	
	nh.setParam("nh_B",100); //相对,参考命名空间
	
	ros::NodeHandle nh_private("~");
	nh_private.setParam("nh_C",100);//私有,参考命名空间与节点名称



（2）无句柄的方式
	（1）原理
		设置参数调用API是ros::param::set，该函数中，参数1传入参数名称，参数2是传入参数值，参数1中参数名称设置时，如果以 / 开头，那么就是全局参数，如果以 ~ 开头，那么就是私有参数，既不以 / 也不以 ~ 开头，那么就是相对参数。

	（2）示例
		（1）ros::param::set("/set_A",100); //全局,和命名空间以及节点名称无关【防盗标记–盒子君hzj】
		（2）ros::param::set("set_B",100); //相对,参考命名空间
		（3）ros::param::set("~set_C",100); //私有,参考命名空间与节点名称
	
	ros::param::set("<param_name>"，<param_value>)【这个更好一点】
```


（3）判断参数是否存在

```bash
判断参数<param_name>是否存在，一般用于条件判断和获取参数之前
	（1）有句柄的方式（较好）
		ros::NodeHandle::hasParam("<param_name>")

（2）无句柄的方式
		ros::param::has("<param_name>")
		【这个更好一点】
```


（4）从参数服务器获取一个参数值

```bash
从参数服务器获取名为<param_name>的参数的值，赋值给变量variable【防盗标记–盒子君hzj】
（1）有句柄的方式（较好）
	ros::NodeHandle::getParam("<param_name>", variable)
	
（2）无句柄的方式
	ros::param::get("<param_name>", variable)
	【这个更好一点】
```


（5）删除参数

```bash
删除参数<param_name>
（1）有句柄的方式（较好）
	ros::NodeHandle::deleteParam("<param_name>")

（2）无句柄的方式
	ros::param::del("<param_name>")
【这个更好一点】
```


. .

## 九、.yaml文件的编写【.yaml文件一般用来存放变量集】


### （1）yaml编写规则


（1）大小写敏感

（2）使用缩进表示层级关系，缩进时不允许使用Tal键，只允许使用空格，缩进的空格数目不重要，只要相同层级的元素左侧对齐即可

（3）”#” 表示注释，从这个字符一直到行尾，都会被解析器忽略【防盗标记–盒子君hzj】

（4）YAML只有两种结构类型需要知道: lists%2c maps . .

### （2）通用格式


```bash
一级变量：
    变量1：XXX
    变量2：XXX
    ....
二级变量：
    变量1：XXX
    变量2：XXX
    ....
```


### （3）例子


```cpp
#There are some params which will be loaded
yaml_param_string1: abcd123
yaml_param_string2: 567efg
 
yaml_param_num1: 123.123
yaml_param_num2: 1234567
 
yaml_param_set:
  param_set_string1: zzzzz
  param_set_num1: 999
  param_set_string2: a6666
  param_set_num2: 2333
 
param_subset:
  param_set_string1: qwer
  param_set_num1: 5432
  param_set_string2: a12s3
```


. .

## 十、.csv文件的编写【.csv文件一般用来存放数据集–电子表格和数据库】


### （1）csv编写规则


文件的每一行都是一组数据记录。每个记录由一个或多个字段组成，用逗号分隔 每行的数据长度可以不一致【防盗标记–盒子君hzj】 每行的数据类型也可以不一样

### （2）例子


```bash
zzz,yyy,xxx
aaa,bbb,ccc
qqq,www,eee,rrr,ttt,yyy
"aaa","bbb","ccc"
```


### 参考资料


[https://blog.csdn.net/woaixiaoyu520/article/details/78455650](https://blog.csdn.net/woaixiaoyu520/article/details/78455650)

## 十一、注意事项(经验)


（1）**ROS中所有的全局参数都用rosparam进行管理** 【若是一个参数用list\set\get\delete进行调用】 【若是某个功能包的一堆参数，用.yaml文件进行存储，用load/dump进行调用】【防盗标记–盒子君hzj】

（2）**若读写的文件不是.yaml文件**，先用一个参数对.csv文件名进行赋值设置（也可以把文件名作为一个变量写在.yaml文件中），再用C++的读写文件的库进行.csv文件的读取 .txt、.csv

（3）**若是设置的name名称较长**，可以再外面设置<arg name"XXX" default=XXX>，通过引用的方式放到param的指令中

（4）**roscpp的参数API有两套**，两套不能混用，一套在ros::param名字空间下，另一套封装在handle中

（5）**在启动工程终端的时候**，rosparam的参数加载情况会最先打印出来的【防盗标记–盒子君hzj】

