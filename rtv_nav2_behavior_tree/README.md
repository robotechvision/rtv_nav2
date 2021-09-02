# rtv_nav2_behavior_tree

The rtv_nav2_behavior_tree module provides additional useful plugins for nav2_bt_navigator:

| BT Node   |      Type      |  Description |
|----------|:-------------|------|
| FallbackPipeline | Control | Type of sequence node that re-ticks previous children when a child returns RUNNING. If a ticked node returns failure, all nodes since the previous node are halted and the execution is restarted from the previous node. WARNING: loops can occur with this pipeline. It is usually more appropriate to use ProgressCheckerPipeline which uses an anti-loop progress check condition |
| HadFeedbackCondition | Condition | A BT::ConditionNode that listens to a feedback of a FollowPath action and returns SUCCESS when the feedback has been received since last halt() and FAILURE otherwise |
| OptimizePathAction | Action | A nav2_behavior_tree::BtActionNode class that wraps rtv_nav2_msgs::action::OptimizePath |
| ProgressCheckerPipeline | Control | Type of sequence node that re-ticks previous children when a child returns RUNNING. If a ticked node returns failure, all nodes since the previous node are halted and the execution is restarted from the previous node. The last node is a progress condition which in the case of a failure of a node checks whether progress has been made since last failure. If no progress has been detected, the execution is restarted from the last node that hasn't been halted since the no-progress situation began. If this restart fails without progress too, next restart starts from the previous node and so on, until the first node is reached. If the restart from the first node fails without progress, ProgressCheckerPipeline returns FAILURE. |
| ResetterNode | Control | The ResetterNode returns result of its last child. If any of the other children returns SUCCESS, all children are halted, however, unlike in ReactiveFallback, the node does not return SUCCESS but keeps ticking the nodes until the last node returns SUCCESS or FAILURE |