# 利用Anaconda安装TensorFlow2.0的CPU和GPU版本

参考：https://blog.csdn.net/Elenstone/article/details/105123255

## 1 安装TensorFlow-cpu版本
新建一个虚拟环境
```bash
conda create -n tensorflow2.0-cpu python=3.7
```
激活虚拟环境：
```bash
conda activate tensorflow2.0-cpu
```
安装Tensorflow2.0 CPU版本
```bash
pip install tensorflow==2.0.0 -i https://pypi.tuna.tsinghua.edu.cn/simple
```
验证Tensorflow2.0 CPU版本
```python
import tensorflow as tf
version = tf.__version__
gpu_ok = tf.test.is_gpu_available()
print("tf version:",version,"\nuse GPU",gpu_ok)
```
如果没有问题就会出现下列情况：
```bash
tf version: 2.0.0
use GPU False
```

## 2 安装TensorFlow-gpu版本
新建一个虚拟环境
```bash
conda create -n tensorflow2.0-gpu python=3.7
```
激活虚拟环境：
```bash
conda activate tensorflow2.0-gpu
```
安装tensorflow2.0 GPU版本的NVINDA驱动
安装GPU版本支持，拥有Nvidia的GPU的windows一般都有默认驱动的，只需要安装cudatoolkit 与 cudnn包就可以了，要注意一点需要安装cudatoolkit 10.0 版本，注意一点，如果系统的cudatoolkit小于10.0需要更新一下至10.0。
```bash
conda install cudatoolkit=10.0 cudnn
```
安装tensorflow2.0 GPU版本
```bash
pip install tensorflow-gpu==2.0.0 -i https://pypi.tuna.tsinghua.edu.cn/simple
```
验证安装:
```python
import tensorflow as tf
version = tf.__version__
gpu_ok = tf.test.is_gpu_available()
print("tf version:",version,"\nuse GPU",gpu_ok)
```
结果：
```python
tf version: 2.0.0
use GPU True
```






