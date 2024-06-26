# Aliases are short commands to replace long ones.

alias crossbuild='~/Zebramites/zebmite_ws/cross_build.sh'
alias deploy='~/scripts/deploy.sh'
alias hwlaunch='roslaunch minimotor minibot_hardware.launch'
alias killros='~/Zebramites/zebmite_ws/kill_ros_.sh'
alias natbuild='~/Zebramites/zebmite_ws/native_build.sh'
alias rosjet='source ~/Zebramites/zebmite_ws/ROSJetsonMaster.sh'
alias rosstd='source ~/Zebramites/zebmite_ws/ROSStandard.sh'
alias simlaunch='roslaunch controller_node 2023_compbot_combined.launch hw_or_sim:=sim joy_or_key:=key button_box:=false'
alias ls='ls --color'
alias driversim='rqt --standalone rqt_driver_station_sim'
alias driverdash='rqt --standalone rqt_dashboard --qt-binding pyqt'
alias axclient='rosrun actionlib_tools axclient.py'
