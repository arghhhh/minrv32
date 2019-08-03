
riscv-formal proofs for minrv32
===============================

Quickstart guide:

First install Yosys, SymbiYosys, and the solvers. See
[here](http://symbiyosys.readthedocs.io/en/latest/quickstart.html#installing)
for instructions.
Clone this repository into under cores in riscv-formal
Build the checks and make.

```
python3 ../../checks/genchecks.py
make -C checks -j$(nproc)
```

