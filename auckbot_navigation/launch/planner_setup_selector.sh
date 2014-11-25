#!/bin/bash
# planner setup selector

PS3='Please enter your choice: '
options=("Carrot Planner" "Global Planner Grid" "Global Planner No Grid" "Quit")
select opt in "${options[@]}"
do
    case $opt in
        "Carrot Planner")
			export MB_BASE_GLOBAL_PLANNER='carrot_planner/CarrotPlanner'
			export MB_USE_GRID_PATH='N/A'
			echo 'planner: carrot_planner/CarrotPlanner, grid: N/A'
            ;;
        "Global Planner Grid")
			export MB_BASE_GLOBAL_PLANNER='global_planner/GlobalPlanner'
			export MB_USE_GRID_PATH='true'
			echo 'planner: global_planner/GlobalPlanner, grid: true'
            ;;
        "Global Planner No Grid")
			export MB_BASE_GLOBAL_PLANNER='global_planner/GlobalPlanner'
			export MB_USE_GRID_PATH='false'
			echo 'planner: global_planner/GlobalPlanner, grid: false'
            ;;
        "Quit")
            break
            ;;
        *) echo invalid option;;
    esac
done
