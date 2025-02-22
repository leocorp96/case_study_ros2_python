from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch


def generate_launch_description():
    moveit_config = (MoveItConfigsBuilder("irb6640_205", package_name="case_moveit_config")
                     .planning_pipelines(
                        pipelines=["ompl", "pilz_industrial_motion_planner", "stomp"],
                        default_planning_pipeline="stomp"
                        )
                     .planning_scene_monitor(
                        publish_robot_description=True,
                        publish_robot_description_semantic=True,
                        publish_planning_scene=True,
                        publish_geometry_updates=True,
                        publish_state_updates=True,
                        publish_transforms_updates=True
                     )
                     .to_moveit_configs())
    return generate_demo_launch(moveit_config)
