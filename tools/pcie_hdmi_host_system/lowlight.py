"""轻量暗部抬升：三通道共用同一个增益，保持通道比例、不引入偏色。

实现上只是查表式的亮度映射，**不是**降噪，也不是学习式的暗光恢复。
输入接受任意尺寸的 HxWx3 uint8 RGB 图像（预览尺寸或模型输入尺寸均可）。
"""
from functools import lru_cache
import numpy as np


@lru_cache(maxsize=21)
def _table(level):
    """按强度档位（0~20）生成 256×256 查找表，并用 lru_cache 复用。

    返回的二维表用 [原始像素值索引行, 该像素的通道最大值索引列] 取值，
    得到提亮后的像素值。增益在暗部最大、向高光平滑收敛到 1.0，
    纯黑始终映射为纯黑（不会把黑场抬成灰）。
    """
    strength = level / 10.0
    v = np.arange(256, dtype=np.float32)
    # Bounded gain, tapering to unity at highlights; black remains black.
    gain = 1.0 + 1.5 * strength * (1.0 - v / 255.0) ** 2
    return np.minimum(np.rint(gain[:, None] * v[None, :]), 255).astype(np.uint8)


def enhance(rgb, strength=1.0):
    """按 strength（0~2 的浮点强度）抬升暗部，返回同尺寸 uint8 RGB。

    每个像素的增益取其三个通道的最大值去查表，三通道共用同一增益，
    因此通道之间的比例保持不变，不会造成色相偏移。
    strength 为 0（或四舍五入后档位为 0）时直接返回原图，不做多余拷贝。
    """
    if rgb.dtype != np.uint8 or rgb.ndim != 3 or rgb.shape[2] != 3:
        raise ValueError("Expected HxWx3 uint8 RGB")
    if not np.isfinite(strength) or not 0 <= strength <= 2:
        raise ValueError("strength must be in [0, 2]")

    level = int(round(strength * 10))
    if level == 0:
        return rgb

    peak = rgb.max(axis=2)                  # 每像素的通道最大值 → 决定该像素的增益
    return _table(level)[peak[:, :, None], rgb]
