
## 状态转换逻辑

根据原始`wall_following.py`的状态机逻辑：

```
FORWARD (1)
  → TURN_TO_FIND_WALL (3) [当前方距离 < 参考距离]

TURN_TO_FIND_WALL (3)
  → TURN_TO_ALIGN_TO_WALL (4) [当侧面和前方都检测到墙面]
  → FIND_CORNER (8) [当侧面近但前方远]

TURN_TO_ALIGN_TO_WALL (4)
  → FORWARD_ALONG_WALL (5) [对齐完成]

FORWARD_ALONG_WALL (5)
  → FIND_CORNER (8) [侧面距离超出阈值]
  → ROTATE_IN_CORNER (7) [前方距离小于阈值]

ROTATE_IN_CORNER (7)
  → TURN_TO_FIND_WALL (3) [旋转完成]

FIND_CORNER (8)
  → ROTATE_AROUND_WALL (6) [侧面距离 <= 参考距离]

ROTATE_AROUND_WALL (6)
  → TURN_TO_FIND_WALL (3) [前方距离小于阈值]
  → 继续绕墙旋转 [侧面距离 > 参考距离]

HOVER (2)
  → 任何时候都可以进入
```