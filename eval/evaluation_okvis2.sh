#!/bin/bash

# datasets
#declare datasetPath="/home/sleutene/Documents/datasets/okvis2/euroc/"
#declare datasetPath="/srv/user/leuteneg/datasets/euroc/"
#declare datasetPath="/home/sleutene/Documents/datasets/okvis2/tumvi/"
declare datasetPath="/srv/user/leuteneg/datasets/tumvi/"
#declare -a datasets=("MH_01_easy" "MH_02_easy" "MH_03_medium" "MH_04_difficult" "MH_05_difficult" "V1_01_easy" "V1_02_medium" "V1_03_difficult" "V2_01_easy" "V2_02_medium" "V2_03_difficult")
declare -a datasets=("dataset-outdoors1_512_16" "dataset-outdoors2_512_16" "dataset-outdoors3_512_16" "dataset-outdoors4_512_16" "dataset-outdoors5_512_16" "dataset-outdoors6_512_16" "dataset-outdoors7_512_16" "dataset-outdoors8_512_16")
declare -a datasets=("dataset-outdoors7_512_16" "dataset-outdoors8_512_16")
#declare -a datasets=("dataset-outdoors4_512_16")
#declare -a datasets=("dataset-outdoors2_512_16" "dataset-outdoors3_512_16" "dataset-outdoors4_512_16" "dataset-outdoors5_512_16")

#declare -a datasets=("dataset-corridor1_512_16" "dataset-corridor2_512_16" "dataset-corridor3_512_16" "dataset-corridor4_512_16" "dataset-corridor5_512_16" )
#declare -a datasets=("dataset-corridor1_512_16" "dataset-corridor2_512_16" "dataset-corridor3_512_16" "dataset-corridor4_512_16" "dataset-corridor5_512_16" "dataset-magistrale1_512_16" "dataset-magistrale2_512_16" "dataset-magistrale3_512_16" "dataset-magistrale4_512_16" "dataset-magistrale5_512_16" "dataset-magistrale6_512_16" "dataset-room1_512_16" "dataset-room2_512_16" "dataset-room3_512_16" "dataset-room4_512_16" "dataset-room5_512_16" "dataset-room6_512_16" "dataset-slides1_512_16" "dataset-slides2_512_16" "dataset-slides3_512_16" )

#declare -a datasets=("MH_02_easy")

# okvis2
#declare okvis2="/home/sleutene/okvis2/build/okvis_app_synchronous"
declare okvis2="/home/leuteneg/workspace/okvis2/build/okvis_app_synchronous"
declare -a okvis2Configs=("config_vio.yaml" "config_slam.yaml")
#declare -a okvis2Configs=("config_vio.yaml")
 
# Iterate the string array using for loop
for dataset in ${datasets[@]}; do
    echo "processing $dataset..."
    for i in {1..3}; do
        rm $datasetPath/$dataset/mav0/okvis2-* #remove old okvis2
        for config in ${okvis2Configs[@]}; do # evaluate okvis2 versions
            eval $okvis2 $datasetPath/$config $datasetPath/$dataset/mav0 &> $datasetPath/$dataset/mav0/okvis2-$config.log
        done
        istring=$( printf '%02d' $i )
        mkdir -p $datasetPath/$dataset/mav0/$istring/ # dataset run number
        rm $datasetPath/$dataset/mav0/$istring/okvis2-* # clear folder
        cp $datasetPath/$dataset/mav0/okvis2-* $datasetPath/$dataset/mav0/$istring/ # store results there
    done
done
