# ML pipeline (V2 vision perception)

Trains the camera depth-to-sectors network from data the drone logs about
itself: the co-mounted VL53L5CX ToF is the automatic ground-truth teacher
(no manual labeling — research docs 05/07/09).

Both model forks are built and kept alive until the owner decides
(see `docs/HANDBOOK.md` "Forks"):

| fork | model | output | script name |
|---|---|---|---|
| A | SectorNet-8 (S/M) | 8 sectors x 4 distance bins | `sectornet_s`, `sectornet_m` |
| B | uPyD-Net-lite | 12x12 dense depth -> min-pooled to sectors | `mupyd` |

## Workflow

```bash
python -m venv .venv && source .venv/bin/activate   # Python 3.11/3.12
pip install -r requirements.txt

# 1. Fly/drive with the xiao_s3_vision build; SD collects sess_*.bin
#    (logging auto-starts while armed). Pull the card -> ./data/

# 2. Train both forks, compare:
python train.py --data ./data --model sectornet_s --out runs/sec_s
python train.py --data ./data --model mupyd       --out runs/mupyd

# 3. Export the winner (or both) to full-int8 tflite:
python export.py --model sectornet_s --ckpt runs/sec_s/best.pt \
                 --data ./data --out export/

# 4. Validate on held-out logs (fork-decision numbers):
python eval.py --tflite export/tflite/*full_integer_quant.tflite --data ./data

# 5. Embed into firmware + flash:
python gen_c_array.py export/tflite/<model>_full_integer_quant.tflite --version 1
pio run -e xiao_s3_vision -t upload

# 6. Per-environment fine-tune (mandatory before trusting a new venue,
#    doc 07): collect 5-10 min there, then:
python train.py --data ./data_newplace --model sectornet_s \
                --finetune runs/sec_s/best.pt --epochs 120 --out runs/sec_s_ft
```

## Contracts (do not break silently)

- **Record layout** `dataset.py` <-> `src/vision/vision_task.cpp` `LogRecord`
  (9,439 B, 'SEC1'). The struct is static-asserted in firmware.
- **Bin edges** [0.5, 1.0, 2.0] m <-> firmware `kBinDistMm` in
  `src/vision/backend_tflm.cpp` and the governor thresholds (doc 08).
- **Input scaling** float 0..1 (int8 zero-point -128) <-> firmware quantize
  step in `backend_tflm.cpp`.
- **Model version byte** increments every deployment (`gen_c_array.py
  --version`); it is logged into every record for dataset attribution.

## Fork decision criteria (doc 09 s6)

Measure on the same held-out sessions + on-target microbenchmark:
sector accuracy, near-bin recall, off-by-one rate, `invoke()` latency,
arena bytes. Gate for flying at all: >70% sector agreement vs ToF after
fine-tuning; park the vision track if two fine-tune rounds stay below that.
