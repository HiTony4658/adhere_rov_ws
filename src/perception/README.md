# `perception/` — 感知层

## 设计理念

新增顶层分类（CLAUDE.md Phase 5.0 Stage 5），与 `hardwares/` 并列：承载所有从
传感器原始数据中提取**语义信息**（而非物理状态估计，那是
`adhere_controller/estimator/RovStateEstimator` 的职责）的节点。本层对标真实
机器人上独立于控制栈运行的 `apriltag_ros` 边界——仿真侧与真机侧发布**完全
相同**的消息契约，`AdhereController` 的消费逻辑在两者之间无需任何代码改动。

## 当前软件包

| 软件包 | 职责 |
|---|---|
| [`adhere_apriltag_detector`](adhere_apriltag_detector/README.md) | 发布墙面贴壁目标站点的 `/rov/tag_detection`（仿真：邻近触发的点发布器；真机：`apriltag_ros` 等价话题契约） |
