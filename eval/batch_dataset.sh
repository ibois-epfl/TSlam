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

__workin_dir__="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
print_process "Changing directory to $__workin_dir__"
cd $__workin_dir__

# 3. loop through each folder of the dataset and run the batch_sequence.sh script
# TODO: 4. run the final summary evaluation script script/compute_summary.py to get the final results across the entire dataset

#==========================================================================================
# 00. activate conda env tslameval
#==========================================================================================

__zenodo_url__="https://zenodo.org/record/8275529"  # <---TODO: replace with newest version

#==========================================================================================
# 0. activate conda env tslameval
#==========================================================================================

eval "$(conda shell.bash hook)"
conda activate tslameval
conda info

#==========================================================================================
# 1. check for tslammonocular executable
#==========================================================================================

print_process "Checking for presence of tslammonocular executable.."
__tslammono_exe__=$__workin_dir__/../build/utils/tslam_monocular
if [ ! -f $__tslammono_exe__ ]; then
    print_warn "tslammonocular executable not found."
    print_warn "compiling tslammonocular.."
    cmake -S $__workin_dir__/../ -B $__workin_dir__/../build
    cmake --build $__workin_dir__/../build
    if [ ! -f $__tslammono_exe__ ]; then
        print_error "The compilation of tslam went wrong, process exiting..."
        exit 1
    fi
    __tslammono_exe__ = $__workin_dir__/../build/utils/tslam_monocular
fi
print_success "tslammonocular executable built or found in $__tslammono_exe__"

#==========================================================================================
# 2/3. check/download for dataset / run batch_sequence.sh
#==========================================================================================

__dataset_dir__=$__workin_dir__/dataset
mkdir -p $__dataset_dir__

# clean out the dataset folder
rm -rf $__dataset_dir__/*

# Define the Zenodo URL
print_process "Downloading dataset from Zenodo + extraction + sequence evaluation.."
html_page=$(wget -qO- $__zenodo_url__)
download_links=$(echo $html_page | grep -oP 'href="\Khttps?://[^"]+')
download_links=$(echo $download_links | tr " " "\n" | grep -oP 'https?://[^"]+\.zip')
for link in $download_links; do
    filename_zip=$(basename $link)
    print_process "----------------------------------------"
    print_process "----------------------------------------"
    print_info "Downloading $filename_zip... to $__dataset_dir__"
    wget -O $__dataset_dir__/$filename_zip $link
    print_success "$filename_zip downloaded"
    print_process "----------------------------------------"
    print_process "extracting $filename_zip..."
    unzip -o $__dataset_dir__/$filename_zip -d $__dataset_dir__
    rm $__dataset_dir__/$filename_zip
    print_success "$filename_zip extracted"
    print_process "----------------------------------------"
    print_process "Starting evaluation of $filename_zip..."
    filename_sequence=$(basename $filename_zip .zip)
    ./batch_sequence.sh -s "$__dataset_dir__/$filename_sequence" -e -t
done
print_success "Single sequence evaluation done"

#==========================================================================================
# 4. run summary evaluation script
#==========================================================================================
print_process "Running summary evaluation script.."
# TODO: launch the summary evaluation script
print_success "Summary evaluation script ran"
