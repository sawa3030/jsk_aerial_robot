# 40.367 start
rosrun hydrus plot_cog_error.py ~/rosbag/20260804_soft_airframe_sawada_change_configuration_2_2026-08-04-14-52-46.bag --output cog_error_20260804_soft_airframe_sawada_change_configuration_2 --window-start-seconds 37.367 --window-end-seconds 60.367
# 31.259 start
rosrun hydrus plot_cog_error.py ~/rosbag/20260804_soft_airframe_sawada_change_configuration_6_2026-08-04-15-08-35.bag --output cog_error_20260804_soft_airframe_sawada_change_configuration_6 --window-start-seconds 28.259 --window-end-seconds 51.259
# 40.307 start
rosrun hydrus plot_cog_error.py ~/rosbag/20260804_soft_airframe_sawada_change_configuration_9_2026-08-04-16-23-33.bag --output cog_error_20260804_soft_airframe_sawada_change_configuration_9 --window-start-seconds 37.307 --window-end-seconds 60.307
# 34.126 start
rosrun hydrus plot_cog_error.py ~/rosbag/20260804_soft_airframe_sawada_change_configuration_10_2026-08-04-16-29-34.bag --output cog_error_20260804_soft_airframe_sawada_change_configuration_10 --window-start-seconds 31.126 --window-end-seconds 54.126

rosrun hydrus plot_rotor_mocap_estimation_error.py ~/rosbag/20260804_soft_airframe_sawada_change_configuration_2_2026-08-04-14-52-46.bag --output 20260804_soft_airframe_sawada_change_configuration_2 --window-start-seconds 37.367 --window-end-seconds 60.367
rosrun hydrus plot_rotor_mocap_estimation_error.py ~/rosbag/20260804_soft_airframe_sawada_change_configuration_6_2026-08-04-15-08-35.bag --output 20260804_soft_airframe_sawada_change_configuration_6 --window-start-seconds 28.259 --window-end-seconds 51.259
rosrun hydrus plot_rotor_mocap_estimation_error.py ~/rosbag/20260804_soft_airframe_sawada_change_configuration_9_2026-08-04-16-23-33.bag --output 20260804_soft_airframe_sawada_change_configuration_9 --window-start-seconds 37.307 --window-end-seconds 60.307
rosrun hydrus plot_rotor_mocap_estimation_error.py ~/rosbag/20260804_soft_airframe_sawada_change_configuration_10_2026-08-04-16-29-34.bag --output 20260804_soft_airframe_sawada_change_configuration_10 --window-start-seconds 31.126 --window-end-seconds 54.126

# 2回目曲がり始めてから、7秒変形、その後5秒静止
rosrun hydrus plot_rotor_mocap_estimation_error.py ~/rosbag/20260807_soft_airframe_sawada_4_joints_9_2026-08-07-16-03-48.bag --output 20260807_soft_airframe_sawada_4_joints_9   --window-start-seconds 35.59 --window-end-seconds 47.86
rosrun hydrus plot_cog_error.py ~/rosbag/20260807_soft_airframe_sawada_4_joints_9_2026-08-07-16-03-48.bag --output cog_error_20260807_soft_airframe_sawada_4_joints_9   --window-start-seconds 35.59 --window-end-seconds 47.86

rosrun hydrus plot_rotor_mocap_estimation_error.py ~/rosbag/20260804_soft_airframe_sawada_box_hold_1_2026-08-04-14-25-51.bag --output 20260804_soft_airframe_sawada_box_hold_1   --window-start-seconds 35 --window-end-seconds 45
rosrun hydrus plot_cog_error.py ~/rosbag/20260804_soft_airframe_sawada_box_hold_1_2026-08-04-14-25-51.bag --output cog_error_20260804_soft_airframe_sawada_box_hold_1   --window-start-seconds 35 --window-end-seconds 45

rosrun hydrus plot_rotor_mocap_estimation_error.py ~/rosbag/20260804_soft_airframe_sawada_go_narrow_space_5_2026-08-04-18-15-55.bag --output 20260804_soft_airframe_sawada_go_narrow_space_5   --window-start-seconds 60 --window-end-seconds 75
rosrun hydrus plot_cog_error.py ~/rosbag/20260804_soft_airframe_sawada_go_narrow_space_5_2026-08-04-18-15-55.bag --output cog_error_20260804_soft_airframe_sawada_go_narrow_space_5   --window-start-seconds 60 --window-end-seconds 75
