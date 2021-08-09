# rtv_nav2

Stack containing additional packages to work with [navigation2](https://github.com/robotechvision/navigation2). Most important packages are:
 * **rtv_nav2_plan_optimizer** - An intermediate node between planner and controller which can be used to periodically optimize global plan path around robot. A faster optimization with more consistent results of higher quality can be obtained this way, compared to an approach when the whole plan is optimized at once. Contains CeresCostawareSmoother plugin which is a revitalized, fixed and improved version of the original nav2_smac_planner smoother
 * **rtv_nav2_csc_pursuit_controller** - A controller which analyzes a subset (Circle - Straight - Circle) of Reeds-Shepp motions, using a set of measurements (positional, angular and longitudinal similarity to global plan, cost, length, number of direction changes) to pick the most appropriate motion from the subset to move from the current robot pose to a carrot pose. Supports reverse motion.
 * **rtv_nav2_behavior_tree** - Additional useful plugins for nav2_bt_navigator

An example of a working configuration can be found in **indoor_navigation** package.
> Written with [StackEdit](https://stackedit.io/).
