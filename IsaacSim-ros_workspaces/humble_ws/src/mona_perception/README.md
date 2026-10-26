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

## SAM server

`tools/sam_server.py` is a standalone HTTP server (not a ROS executable, not installed by CMake) that runs on the
lab server in conda env `mona-detect` (ultralytics SAM 2.1). The model loads once at start. Single-threaded: one
request at a time.

    cd <dir that holds/receives the weights>   # sam2.1_s.pt auto-downloads into the cwd on first start
    /home/ciderlab-server1/anaconda3/envs/mona-detect/bin/python tools/sam_server.py [--model sam2.1_s.pt] [--host 0.0.0.0] [--port 8091]

Default port 8091. `GET /health` returns `{"status":"ok","model":...}`.
`POST /segment` takes JSON `{image_jpeg_b64, points:[[u,v],..], point_labels:[1,..], box:[x0,y0,x1,y1]|null}`
(at least one point or a box; points and box may be combined; pixel coords of the sent image) and returns
`{masks_png_b64:[..], scores:[..], latency_s, model}`: all 3 SAM candidate masks at the original resolution,
single-channel PNG 0/255, aligned with `scores`. Malformed request gives HTTP 400.

Test (needs GPU/weights): `/home/ciderlab-server1/anaconda3/envs/mona-detect/bin/python tools/test_sam_server.py`
(also pytest-collectable; pytest is not installed in `mona-detect`).
