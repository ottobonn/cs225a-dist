if [ "$#" -lt 4 ]; then
	cat <<EOM
This script calls ./visualizer, ./simulator, and the specified controller simultaneously.
All the arguments after the controller will be passed directly to it.

Usage: sh run.sh <controller-executable> <path-to-world.urdf> <path-to-robot.urdf> <robot-name> <extra_controller_args>
EOM
else
	trap 'kill %1; kill %2' SIGINT
	trap 'kill %1; kill %2' EXIT
	./simulator $2 $3 $4 &> simulator.log & ./visualizer $2 $3 $4 &> visualizer.log & ./"$@"
	# ./visualizer $2 $3 $4 & ./simulator $2 $3 $4 & ./"$@"
fi
