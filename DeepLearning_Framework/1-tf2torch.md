
# 主要介绍Pytorch与Tensorflow的相互转换

### 1 模型输入的图片格式问题
Caffe的通道顺序是: NCHW;
Tensorflow的通道顺序默认是: NHWC（但可以设置成NCHW）;
Pytorch的通道顺序是: NCHW, 通常使用`img=img.permute(0,3,1,2)`来进行转换，第一个维度0表示Batch，即(B,H,W,C)--->(B,C,H,W)。

### 2 

