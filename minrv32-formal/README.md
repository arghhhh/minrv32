
riscv-formal proofs for minrv32
===============================

Quickstart guide:

First install Yosys, SymbiYosys, and the solvers. See
[here](http://symbiyosys.readthedocs.io/en/latest/quickstart.html#installing)
for instructions.
Clone this repository into a directory beside riscv-formal.  This is to keep the riscv-formal directory completely separate.  Because of this, there is a modified copy of genchecks.py to account for differences in paths.
Build the checks and make.

```
python3 genchecks.py
make -C checks -j$(nproc)
```

