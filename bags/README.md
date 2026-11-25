# ROS 2 bags

Bag recordings are not stored in git because they are large (`bags/` is in `.gitignore`).
Suggested layout:

- `bags/raw/`        direct recordings
- `bags/processed/`  cropped or filtered bags
- `bags/docs/`       notes, screenshots, metadata

Record the four sensor clouds and the static transforms:

```bash
ros2 bag record /front/cloud /right/cloud /back/cloud /left/cloud /tf_static -o bags/raw/<recording_name>
```

Replay a recording through the pipeline:

```bash
pipenv run ros2 launch lidar_bringup bag_data.launch.xml bag_path:=bags/raw/<recording_name>
```
