# TSlam evaluation

# Run
Run the following command:
```bash
./batch_sequence.sh -s dataset/01 -e
```
- `-s`: path to the folder sequence
- `-e`: to export videos
- `-t`: to compute only tag mode, otherwise tags only and tags + features

# TODO:
- add `scripts/compute_model_metrics.py` to `batch_sequence.sh`
- add `scripts/compute_summary.py` to `batch_dataset.sh`