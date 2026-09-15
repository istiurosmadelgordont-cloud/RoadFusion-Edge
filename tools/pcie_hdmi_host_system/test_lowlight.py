"""暗光增强 lowlight.enhance 的测试。

关注点：关闭档行为、黑白两端不被破坏、暗部抬升且不偏色、
灰阶单调性、以及非法输入必须报错。
"""
import unittest
import time
import numpy as np
from lowlight import enhance

class Tests(unittest.TestCase):
    def test_disabled(self):
        """strength=0 时原样返回同一个对象，不做多余拷贝。"""
        a=np.array([[[20,10,5]]],dtype=np.uint8)
        self.assertIs(enhance(a,0),a)
    def test_black_white(self):
        """纯黑不能被抬成灰，纯白也不该被压暗。"""
        a=np.array([[[0,0,0],[255,255,255]]],dtype=np.uint8)
        np.testing.assert_array_equal(enhance(a,2),a)
    def test_shadow_and_color(self):
        """暗部要变亮、通道比例要保持（不偏色）、亮部像素保持不变。"""
        a=np.array([[[40,20,10],[255,0,0],[0,255,0],[255,255,0]]],dtype=np.uint8)
        b=enhance(a,1)
        self.assertGreater(int(b[0,0,0]),40)                        # 暗部确实被抬升
        self.assertLessEqual(abs(int(b[0,0,0])-2*int(b[0,0,1])),1)  # R:G 仍约为 2:1（含取整误差）
        np.testing.assert_array_equal(b[:,1:],a[:,1:])              # 亮部不受影响
    def test_monotonic_gray(self):
        """灰阶必须单调不减，且每个像素只增不减（不能出现明暗反转）。"""
        a=np.repeat(np.arange(256,dtype=np.uint8)[None,:,None],3,axis=2)
        for strength in (0.1,1,2):
            b=enhance(a,strength)
            self.assertTrue(np.all(np.diff(b[0,:,0].astype(int))>=0))
            self.assertTrue(np.all(b>=a))
    def test_invalid(self):
        """非 uint8 输入、NaN 强度都必须拒绝。"""
        with self.assertRaises(ValueError): enhance(np.zeros((2,2,3)),1)
        with self.assertRaises(ValueError): enhance(np.zeros((2,2,3),dtype=np.uint8),float('nan'))

if __name__=='__main__': unittest.main()
