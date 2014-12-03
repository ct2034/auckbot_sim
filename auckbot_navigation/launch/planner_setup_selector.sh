#!/bin/bash
# planner setup selector

PS3='Please enter your choice: '
options=("NavFN" "Carrot Planner" "Global Planner Grid" "Global Planner No Grid")
select opt in "${options[@]}"
do
    case $opt in
        "NavFN")
            export MB_BASE_GLOBAL_PLANNER='navfn/NavfnROS'
            export MB_USE_GRID_PATH='N/A'
            echo "planner: $MB_BASE_GLOBAL_PLANNER, grid: $MB_USE_GRID_PATH"
            break
            ;;
        "Carrot Planner")
            export MB_BASE_GLOBAL_PLANNER='carrot_planner/CarrotPlanner'
            export MB_USE_GRID_PATH='N/A'
            echo "planner: $MB_BASE_GLOBAL_PLANNER, grid: $MB_USE_GRID_PATH"
            break
            ;;
        "Global Planner Grid")
			export MB_BASE_GLOBAL_PLANNER='global_planner/GlobalPlanner'
			export MB_USE_GRID_PATH='true'
            echo "planner: $MB_BASE_GLOBAL_PLANNER, grid: $MB_USE_GRID_PATH"
            break
            ;;
        "Global Planner No Grid")
			export MB_BASE_GLOBAL_PLANNER='global_planner/GlobalPlanner'
			export MB_USE_GRID_PATH='false'
            echo "planner: $MB_BASE_GLOBAL_PLANNER, grid: $MB_USE_GRID_PATH"
            break
            ;;
        *) echo invalid option;;
    esac
done
