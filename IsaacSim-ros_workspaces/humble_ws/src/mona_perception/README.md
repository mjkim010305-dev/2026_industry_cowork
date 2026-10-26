# mona_perception

VLM-based movable/fixed obstacle perception.

- `vlm_movability_node.py` - RGB + depth -> segments obstacle candidates, asks a
  VLM (ollama) whether each is movable, majority-votes the answers.
  In: `rgb_topic`, `depth_topic`, `camera_info_topic`, `/vlm/reset`, TF.
  Out: `/obstacle/is_movable`, `/vlm/roi`, `/vlm/annotated`, `/vlm/judgments`, `/vlm/candidates`.
- `movable_obstacle_map_node.py` - transforms `/vlm/candidates` into the map frame, merges repeated sightings.
  Out: `/movable_obstacles/{markers,poses,list}` and a YAML semantic map (`semantic_map_path`).

VLM inference is offloaded to the lab server over HTTP (`ollama_url`); no ROS traffic crosses machines.

## Launch

    ros2 launch mona_perception perception.launch.py [mode:=sim|real] [ollama_url:=http://<server>:11434/api/generate] [model:=gemma4:e4b]

Args: `mode`, `ollama_url`, `model`, `rgb_topic`, `depth_topic`, `camera_info_topic`,
`use_sim_time`, `semantic_map_path`, `query_period`, `latch`, `detector_url`
(empty topic/`use_sim_time` args take the mode default).
`mode:=real` uses frame `festa_color_optical` (TF from festa_demo); its depth topic is a PLACEHOLDER.

## Tests

    python3 -m pytest test
