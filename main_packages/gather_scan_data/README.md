## Example use:

1. Simple use:
`rosrun gather_scan_data gather_scan_data`

2. Passing parameters:

`rosrun gather_scan_data gather_scan_data _file_path:="$(rospack find gather_scan_data)/data"`

`rosrun gather_scan_data gather_scan_data /catvehicle/front_laser_points:=/catvehicle2/front_laser_points /catvehicle/cmd_vel:=/catvehicle2/cmd_vel _file_path:="$(rospack find gather_scan_data)/data"`
