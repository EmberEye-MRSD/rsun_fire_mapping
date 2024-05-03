
# rsun_fire_mapping

The repository contains the codebase for running fire localization, fire mapping and for publishing ground-truth poses.

**Launching Nodes:**

```
  roslaunch rsun_fire_mapping mapping.launch
```

**Running Fire Localization**

```
cd rsun_fire_mapping
python3 src/fire_localization.py 
```

This node is reponsible for publishing a pose array on ros topic */hotspots/array* containing the location of the hotspot with respect to the odom frame

**Running Ground Truth Publisher**

```
roslaunch rsun_fire_mapping gt_publisher.launch
```

Propogates the location of the hotspots provided in the launch file under the param name "gt_path". The locations of the hotspots are published as markers.

Running Fire Mapping

```
cd rsun_fire_mapping
python3 src/temporal_mapping.py
```

This node subscribes to the fire hotspot array published by the fire localization node, conducts post-processing to identify distinct hotspots, and then broadcasts them as markers.
