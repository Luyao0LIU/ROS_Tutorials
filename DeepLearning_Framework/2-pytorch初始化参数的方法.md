# pytorch参数初始化两种方法

参考：https://blog.csdn.net/qq_42995479/article/details/120598487
<br>
tensorflow和pytorch中的参数初始化调用方法
https://blog.csdn.net/qq_34218078/article/details/109611105
<br>
深度学习中的参数初始化方法：Xavier详细推导+pytorch实现
https://zhuanlan.zhihu.com/p/648576849


## 初始化方式
### 1.单层网络初始化方式
在创建model后直接调用torch.nn.init里的初始化函数
```python
conv = torch.nn.Conv2d(3, 64, kernel_size = 3, stride = 1, padding = 1)
torch.nn.init.xavier_uniform_(conv.weight)
torch.nn.init.xavier_uniform_(conv.bias, 0)
```

### 2.多层网络初始化方式
(1). 使用apply函数; (2) 在_init_函数中用self.modules()进行初始化

**方法一**
1，先定义初始化模型方法;

2，运用apply()调用weight_init函数.
```python
class Net(nn.Module):
 
    def __init__(self, in_dim, n_hidden_1, n_hidden_2, out_dim):
        super().__init__()
 
        self.layer = nn.Sequential(
            nn.Linear(in_dim, n_hidden_1), 
            nn.ReLU(True),
            nn.Linear(n_hidden_1, n_hidden_2),
            nn.ReLU(True),
            nn.Linear(n_hidden_2, out_dim)
             )    
    def forward(self, x):
        x = self.layer1(x)
        x = self.layer2(x)
        x = self.layer3(x)
        return x
 
 # 1. 根据网络层的不同定义不同的初始化方式     
def weight_init(m):
    if isinstance(m, nn.Linear):
        nn.init.xavier_normal_(m.weight)
        nn.init.constant_(m.bias, 0)
    # 也可以判断是否为conv2d，使用相应的初始化方式 
    elif isinstance(m, nn.Conv2d):
        nn.init.kaiming_normal_(m.weight, mode='fan_out', nonlinearity='relu')
     # 是否为批归一化层
    elif isinstance(m, nn.BatchNorm2d):
        nn.init.constant_(m.weight, 1)
        nn.init.constant_(m.bias, 0)
 
# 2. 初始化网络结构        
model = Net(in_dim, n_hidden_1, n_hidden_2, out_dim)
 
# 3. 将weight_init应用在子模块上
model.apply(weight_init)
#torch中的apply函数通过可以不断遍历model的各个模块，并将weight_init函数应用在这些Module上
```
**警告**： 这种初始化方式采用的是递归的形式， 但是在Python中对递归的层数是有限制的。 
因此， 当网络的层数很深的时候， 可能会报错

**方法二**
定义在模型中，利用self.modules()来进行循环
```python
class Net(nn.Module):
 
    def __init__(self, in_dim, n_hidden_1, n_hidden_2, out_dim):
        super().__init__()
        self.layer = nn.Sequential(
            nn.Linear(in_dim, n_hidden_1), 
            nn.ReLU(True),
            nn.Linear(n_hidden_1, n_hidden_2),
            nn.ReLU(True),
            nn.Linear(n_hidden_2, out_dim)
             )    
 
       for m in self.modules():
             if isinstance(m, nn.Conv2d):
                 nn.init.kaiming_normal_(m.weight, mode='fan_out', nonlinearity='relu')
             elif isinstance(m, (nn.BatchNorm2d, nn.GroupNorm)):
                 nn.init.constant_(m.weight, 1)
                 nn.init.constant_(m.bias, 0)
 
    def forward(self, x):
        x = self.layer1(x)
        x = self.layer2(x)
        x = self.layer3(x)
        return x
```
方法二中，需要了解self.modules()和self.children()的区别，可以见
https://discuss.pytorch.org/t/module-children-vs-module-modules/4551/6​discuss.pytorch.org/t/module-children-vs-module-mod

https://link.zhihu.com/?target=https%3A//discuss.pytorch.org/t/module-children-vs-module-modules/4551/6

## PyTorch提供了几种常见的参数初始化方式的实现
### Xavier Initialization
基本思想是维持输入和输出的方差一致，避免了所有的输出值都为0， 使用于任何激活函数

但是Xavier初始化在tanh中表现的很好， 在relu中表现很差
```python
# Xavier 均匀分布:torch.nn.init.xavier_uniform_(tensor, gain = 1), 服从均匀分布U(-a, a)， 
# 分布参数a=gain * sqrt(6 / (fan_in + fan_out)), gain的大小由激活函数的类型来决定。
# 其中fan_in是指第i层神经元的个数，fan_out是指第i + 1层神经元的个数
for m in net.modules():
     if isinstance(m, (torch.nn.Linear, torch.nn.Conv1d, torch.nn.Conv2d)):
         torch.nn.init.xavier_uniform_(m.weight)avier
 
for m in net.modules():
     if isinstance(m, torch.nn.Conv2d):
         torch.nn.init.xavier_uniform_(m.weight, gain = torch.nn.init.calculate_gain('relu'))
 
# Xavier 正态分布: torch.nn.init.xavier_normal_(tensor, gain = 1) 服从正态分布N(mean = 0, std)，
# 其中 std = gain * sqrt(2 / (fan_in + fan_out))
```

### Kaiming Initialization
针对Xavier在relu表现不佳被提出。基本思想仍然从“输入输出方差一致性”角度出发，在Relu网络中， 假设每一层有一半的神经元被激活，另一半为0。一般在使用Relu的网络中推荐使用这种初始化方式。
```python
# kaiming均匀分布
# torch.nn.init.kaiming_uniform_(tensor, a=0, mode='fan_in', nonlinearity='leaky_relu')
# 服从 U(-a, a), a = sqrt(6 / (1 + b ^2) * fan_in), 其中b为激活函数的负半轴的斜率， relu是0
# model 可以是fan_in或者fan_out。fan_in 表示使正向传播时，方差一致； fan_out使反向传播时， 方差一致
# nonlinearity 可选为relu和leaky_relu， 默认是leaky_relu
 
# kaiming正态分布,  N～ (0,std)，其中std = sqrt(2/(1+b^2)*fan_in)
# torch.nn.init.kaiming_normal_(tensor, a=0, mode='fan_in', nonlinearity='leaky_relu')
 
for m in net.modules():
     if isinstance(m, torch.nn.Conv2d):
          torch.nn.kaiming_normal_(m.weight, mode = 'fan_in')
```

### Orthogonal Initialization 正交初始化
主要是解决神经网络中出现的梯度消失和梯度爆炸等问题，是RNN中常用的初始化方法
```python
for m in modules():
     if isinstance(m, torch.nn.Conv2d):
           torch.nn.init.orthogonal(m.weight)
```

### 常数初始化
```python
for m in modules():
    if isinstance(m, torch.nn.Conv2d):
          torch.nn.init.constant(m.weight, 0.5)
          torch.nn.init.constant(m.bias, 0)
```

## 来源：
https://zhuanlan.zhihu.com/p/188701989

https://zhuanlan.zhihu.com/p/405752148











