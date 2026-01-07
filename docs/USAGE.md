# Usage Guide

Run the scheduler:
```
python src/gate_heterogeneous_scheduler.py --config configs/config_warehouse_3x6.yaml
```

Compare with baselines:
```
python src/benchmark_veriros.py --config configs/config_warehouse_3x6.yaml
```

Test scalability:
```
python src/scenario_generator.py
```

See README.md for more details.
