# SAM_SDK

## **1. 项目介绍**

该模型可以根据一个辅助点来分割出图像的物品。它的输入是一张rgb图像和一个辅助点，输出是辅助点对应目标的mask。
它目前支持6种权重，包括对资源需求比较小的mobile_sam以及对通用场景效果更好的sam_b等等。
睿眼中使用它来为指定点位抓取计算被抓取物体的mask，来为后续机械臂的下爪位置提供精确的数据。

- **API链接**：[API链接地址](http://192.168.0.188:8090/ai_lab_rd02/ai_sdks/sam)

## **2. 代码结构**

```
sam/
│
├── README.md        <- 项目的核心文档
├── requirements.txt    <- 项目的依赖列表
├── setup.py        <- 项目的安装脚本
│
├── src/          <- 项目的源代码
│  ├── interface.py       <- 程序的主入口
│  └── general.py        <- 核心功能或业务逻辑代码
└── tests/      <- 功能测试目录
```

## **3.环境与依赖**

* python3.8+
* torch==1.13.1+cu117
* torchvision==0.14.1+cu117
* torchaudio==0.13.1
* numpy
* opencv-python

## **4. 安装说明**

1. 安装Python 3.8或者更高版本
2. 克隆项目到本地：`git clone http://192.168.0.188:8090/ai_lab_rd02/ai_sdks/sam.git`
3. 进入项目目录：`cd sam`
4. 安装依赖：`pip install -r requirements.txt`
5. 编译打包：在与 `setup.py `文件相同的目录下执行以下命令：`python setup.py bdist_wheel`。 在 `dist` 文件夹中找到 `.wheel` 文件，例如：`dist/sam-0.1.0-py3-none-any.whl`。
6. 安装：`pip install sam-0.1.0-py3-none-any.whl`

## **5. 使用指南**

### 推荐硬件&软件&环境配制

- 3090Ti对应安装nvidia-driver-535
- 安装对应cuda版本11.7
- 安装对应torch和torchvision版本
- 命令：`pip install torch==1.13.1+cu117 torchvision==0.14.1+cu117 torchaudio==0.13.1 --extra-index-url https://download.pytorch.org/whl/cu117`

基于开源yolo的sam模型，使用图片的纹理特征信息，在使用点提示，框提示，或者无提示的情况下，得到分割图片的结果，对结果进行处理分析。

### 如何配置权重

yolo_weights：共有六类权重分别为

1.FastSAM-s.pt https://github.com/ultralytics/assets/releases/download/v0.0.0/FastSAM-s.pt

2.FastSAM-x.pt https://github.com/ultralytics/assets/releases/download/v0.0.0/FastSAM-x.pt

3.sam_b.pt  https://github.com/ultralytics/assets/releases/download/v0.0.0/sam_b.pt

4.sam_l.pt https://github.com/ultralytics/assets/releases/download/v0.0.0/sam_l.pt

5.sam_h.pt https://github.com/ultralytics/assets/releases/download/v0.0/sam_h.pt

6.mobile_sam.pt https://github.com/ultralytics/assets/releases/download/v0.0.0/mobile_sam.pt

## 6. 接口示例

```python
import cv2
from sam.interface import DetectBase


def test_detect_base():
    # 初始化SAM类对象
    sam_seg = DetectBase()

    # 选择SAM模型
    predictor = sam_seg.gen_model(5)

    # 图片路径
    color_img = cv2.imread('data.jpg')

    # 数据预处理预留接口
    color_frame = sam_seg.forward_handle_input(color_img)

    # 模型推理
    results = sam_seg.detect(5, color_frame, predictor=predictor, model=None, point=(366, 251), bboxes=None)

    # 清除模型缓存
    sam_seg.delete_model()

    # 模型结果后处理
    center, mask = sam_seg.backward_handle_output(results)

    # 显示结果
    cv2.imshow('mask', mask)
    cv2.waitKey(0)

if __name__ == '__main__':
    test_detect_base()

```

## 7. **许可证信息**

说明项目的开源许可证类型（如MIT、Apache 2.0等）。

* 本项目遵循MIT许可证。

## 8. 常见问题解答（FAQ）**

列出一些常见问题和解决方案。

- **Q1：机械臂连接失败**

  答案：修改过机械臂IP

- **Q2：UDP数据推送接口收不到数据**

  答案：检查线程模式、是否使能推送数据、IP以及防火墙