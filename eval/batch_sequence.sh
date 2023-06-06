#!/bin/bash

function print_process {
  echo -e "\e[37m[Process]: $1 \e[0m"
}
function print_info {
  echo -e "\e[35m[Info]: $1 \e[0m"
}
function print_error {
  echo -e "\e[31m[Error]: $1 \e[0m"
}
function print_warning {
  echo -e "\e[33m[Warning]: $1 \e[0m"
}
function print_success {
  echo -e "\e[32m[Success]: $1 \e[0m"
}

#==========================================================================================
# args input
#==========================================================================================

# set names arguments as bash script input
__export_video__=$""
__only_tags__=1
__sequence_dir__=$""
__export_animation__=$""

print_usage() {
  print_info "Usage: ./batch_sequence.sh \
            -s <sequence_dir>: the folder of the dataset containing the video \
            -e <export_video>: to export the video \
            -t <only_tags>: do not compute evaluation with keypoints"
}

while getopts "s:et" flag; do
  case $flag in
    s) __sequence_dir__="$OPTARG"
    ;;
    e) __export_video__=$"--exportVideo" && 
       __export_animation__=$'--makeAnimation ""'
    ;;
    t) __only_tags__=0
    ;;
    *) print_usage 
       exit 1 ;;
  esac
done

#==========================================================================================
# args parsing
#==========================================================================================

__workin_dir__="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
print_process "Changing directory to $__workin_dir__"
cd $__workin_dir__
__output_dir__="${__workin_dir__}/results"
mkdir -p $__output_dir__
__tslammono_exe__=$__workin_dir__/../build/utils/tslam_monocular

__sequence_name__=$(basename $__sequence_dir__)
__lbl_path__=$__sequence_dir__/${__sequence_name__}_label_sequence/${__sequence_name__}_label_sequence.txt
__calib_path__=$__workin_dir__/calibration/calibration_orange_A_1280_720_r.yml
__map_path__=$__sequence_dir__/${__sequence_name__}_tslam_map_files/${__sequence_name__}_opti_map.map
__video_path__=$__sequence_dir__/${__sequence_name__}_opti_recordings/video.mp4
__csv_gt_path__=$__sequence_dir__/${__sequence_name__}_opti_recordings/refined_stream.csv
__output_subdir__=$__output_dir__/${__sequence_name__}_$(date +%Y-%m-%d_%H-%M-%S)
mkdir -p $__output_subdir__
__output_poses_subdir__=$__output_subdir__/poses
__output_metrics_subdir__=$__output_subdir__/metrics

log_path=${__output_subdir__}/batch_sequence.log
exec > >(tee -a ${log_path} )

mkdir -p $__output_poses_subdir__
print_process "Sorting arguments.."
print_info "Sequence directory: $__sequence_dir__"
print_info "Sequence name: $__sequence_name__"
print_info "Label path: $__lbl_path__"
print_info "Calibration path: $__calib_path__"
print_info "Map path: $__map_path__"
print_info "Video path: $__video_path__"
print_info "Gt csv path: $__csv_gt_path__"
print_info "Output sub-directory: $__output_poses_subdir__"
print_info "Output metrics sub-directory: $__output_metrics_subdir__"
print_info "Export video: $__export_video__"
print_info "Export animation: $__export_animation__"
if [ -z "$__sequence_dir__" ] || 
    [ -z "$__sequence_name__" ] || 
    [ -z "$__lbl_path__" ] || 
    [ -z "$__calib_path__" ] || 
    [ -z "$__map_path__" ] || 
    [ -z "$__video_path__" ] || 
    [ -z "$__csv_gt_path__" ] ||
    [ -z "$__output_subdir__" ] ||
    [ -z "$__output_metrics_subdir__" ] ||
    [ -z "$__output_poses_subdir__" ]; then
    print_error "Some of the variables are empty, please check and try again."
    exit 1
fi

#==========================================================================================
# run pose exporter
#==========================================================================================

__name_keytags__="keytags"
if [ $__only_tags__ -eq 1 ]; then
    python script/compute_poses.py \
          --frames $__lbl_path__ \
          --monoexe $__tslammono_exe__ \
          --calib $__calib_path__ \
          --map $__map_path__ \
          --video $__video_path__ \
          --output $__output_poses_subdir__ \
          $__export_video__ \
          --name $__name_keytags__

    python script/compute_slam_metrics.py \
          --name $__name_keytags__ \
          --gt $__csv_gt_path__ \
          --ts ${__output_poses_subdir__}/${__name_keytags__} \
          --out $__output_metrics_subdir__ \
          --saveImg \
          $__export_animation__
fi

__name_tagsonly__="tagsonly"
python script/compute_poses.py \
    --frames $__lbl_path__ \
    --monoexe $__tslammono_exe__ \
    --calib $__calib_path__ \
    --map $__map_path__ \
    --video $__video_path__ \
    --output $__output_poses_subdir__ \
    --name $__name_tagsonly__ \
    $__export_video__ \
    --onlyTags

python script/compute_slam_metrics.py \
    --name $__name_tagsonly__ \
    --gt $__csv_gt_path__ \
    --ts ${__output_poses_subdir__}/${__name_tagsonly__} \
    --out $__output_metrics_subdir__ \
    --saveImg \
    $__export_animation__

#==========================================================================================
# model evaluation
#==========================================================================================

# TODO: to be added @HB