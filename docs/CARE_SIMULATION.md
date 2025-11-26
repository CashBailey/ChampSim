# CARE Policy Quickstart

This guide shows how to clone this ChampSim fork and run simulations with the CARE replacement policy (implemented in `replacement/lru/lru.h` and `replacement/lru/lru.cc`).

## 1. Clone and set up dependencies

```bash
# Clone the repository
git clone https://github.com/ChampSim/ChampSim.git
cd ChampSim

# Initialize submodules and install dependencies via vcpkg
git submodule update --init
vcpkg/bootstrap-vcpkg.sh
vcpkg/vcpkg install
```

## 2. Configure ChampSim for CARE

The CARE policy is the default replacement module for the LLC. To generate build files, run the configuration script (you can optionally copy and edit `champsim_config.json` if you want a custom setup).

```bash
# Use the provided configuration
./config.sh champsim_config.json
```

If you craft your own configuration, ensure the LLC replacement policy remains `"lru"`, which maps to the CARE implementation in `replacement/lru`:

```json
{
  "LLC": {
    "replacement": "lru"
  }
}
```

## 3. Build

Compile ChampSim after configuration:

```bash
make -j$(nproc)
```

The binaries are produced under `bin/`.

## 4. Obtain traces

ChampSim runs on compressed traces. You can download benchmark traces (e.g., DPC-3) or use your own. Store them locally, for example in `~/traces/`.

## 5. Run CARE simulations

Invoke the simulator binary with warmup and simulation lengths plus the path to your trace. For example:

```bash
bin/champsim \
  --warmup-instructions 200000000 \
  --simulation-instructions 500000000 \
  ~/traces/600.perlbench_s-210B.champsimtrace.xz
```

During the run, the CARE replacement logic in `replacement/lru/lru.cc` will manage the LLC.

## 6. Inspecting the CARE implementation

The core CARE logic (including signature history handling, EPV management, and PMC-aware insertion/promotion) lives in:

- `replacement/lru/lru.h`
- `replacement/lru/lru.cc`

Refer to these files if you want to adjust the policy or verify behavior.
