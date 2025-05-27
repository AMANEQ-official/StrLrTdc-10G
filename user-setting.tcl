set_property -name "customized_default_ip_location" -value $origin_dir/ip -objects [current_project]
set_property ip_repo_paths $origin_dir/ip [current_project]
update_ip_catalog
set_param project.defaultIPCacheSetting none

file mkdir $origin_dir/build/temp_constraints
close [ open $origin_dir/build/temp_constraints/debug.xdc w ]
read_xdc $origin_dir/build/temp_constraints/debug.xdc
set_property TARGET_CONSTRS_FILE [get_files debug.xdc] [get_filesets constrs_1]

