#!/bin/bash
iris_num=${IRIS_NUM:-4}
typhoon_h480_num=${TYPHOON_NUM:-0}
solo_num=${SOLO_NUM:-0}
plane_num=${PLANE_NUM:-0}
rover_num=${ROVER_NUM:-0}
standard_vtol_num=${STANDARD_VTOL_NUM:-0}
tiltrotor_num=${TILTROTOR_NUM:-0}
tailsitter_num=${TAILSITTER_NUM:-0}

echo "[comm] iris_num=$iris_num typhoon_h480_num=$typhoon_h480_num solo_num=$solo_num"

vehicle_num=0
while(( $vehicle_num< iris_num)) 
do
    python3 multirotor_communication.py iris $vehicle_num&
    let "vehicle_num++"
done

vehicle_num=0
while(( $vehicle_num< typhoon_h480_num)) 
do
    python3 multirotor_communication.py typhoon_h480 $vehicle_num&
    let "vehicle_num++"
done

vehicle_num=0
while(( $vehicle_num< solo_num)) 
do
    python3 multirotor_communication.py solo $vehicle_num&
    let "vehicle_num++"
done

vehicle_num=0
while(( $vehicle_num< plane_num)) 
do
    python3 plane_communication.py $vehicle_num&
    let "vehicle_num++"
done

vehicle_num=0
while(( $vehicle_num< rover_num)) 
do
    python3 rover_communication.py $vehicle_num&
    let "vehicle_num++"
done

vehicle_num=0
while(( $vehicle_num< standard_vtol_num)) 
do
    python3 vtol_communication.py standard_vtol $vehicle_num&
    let "vehicle_num++"
done

vehicle_num=0
while(( $vehicle_num< tiltrotor_num)) 
do
    python3 vtol_communication.py tiltrotor $vehicle_num&
    let "vehicle_num++"
done

vehicle_num=0
while(( $vehicle_num< tailsitter_num)) 
do
    python3 vtol_communication.py tailsitter $vehicle_num&
    let "vehicle_num++"
done
