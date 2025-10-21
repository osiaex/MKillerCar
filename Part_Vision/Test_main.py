import torch
print("CUDA 可用：", torch.cuda.is_available())
print("当前设备：", torch.cuda.get_device_name(0) if torch.cuda.is_available() else "CPU")
